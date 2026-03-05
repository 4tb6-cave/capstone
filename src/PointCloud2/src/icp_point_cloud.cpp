// How to run: ./<executable> /path/to/ply/dir starting_frame ending_frame [voxel_size] [poly_order] [search_radius] [num_points_sor] [std_dev_trim] [enable_mls]

#include <iostream>
#include <sstream>
#include <iomanip>
#include <filesystem>
#include "CLI11.hpp"
#include <vector>
#include <tuple>
#include <atomic>
#include <thread>
#include <chrono>

#include <pcl/io/ply_io.h>
#include <pcl/types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/point_cloud.h>
#include <pcl/console/print.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/processing.h>
#include <pcl/visualization/pcl_visualizer.h>



void print_progress(int current, int total) {
	if (total <= 0) return;
	const int width = 40;
	double ratio = static_cast<double>(current) / static_cast<double>(total);
	if (ratio > 1.0) ratio = 1.0;
	int filled = static_cast<int>(ratio * width);

	std::cout << "\rICP Progress [";
	for (int i = 0; i < width; ++i) {
		std::cout << (i < filled ? '#' : '-');
	}
	std::cout << "] " << std::setw(3) << static_cast<int>(ratio * 100.0)
	          << "% (" << current << "/" << total << ")";
	std::cout.flush();

	if (current >= total) {
		std::cout << std::endl;
	}
}

/**
 * Save Eigen::Matrix4f to file
 */
void save_matrix(std::string filename, const Eigen::Matrix4f &matrix)
{
	const static Eigen::IOFormat csv_format(Eigen::FullPrecision, Eigen::DontAlignCols, ", ", "\n");
	std::ofstream file(filename);
	if (file.is_open())
	{
		file << matrix.format(csv_format);
		file.close();
	}
	else
	{
		std::cerr << "Failed to open " << filename << std::endl;
	}
}

// Global progress monitor (atomic for thread safety)
std::atomic<int> completed_tasks = 0;

// Configuration passed to icp_thread function
typedef struct icp_config
{
	std::string input_dir;
	std::string output_dir;
	bool enable_gicp;
} icp_config_t;

void icp_thread(icp_config_t config, std::vector<std::tuple<int, int>> icp_pairs)
{
	for (auto p : icp_pairs)
	{
		int source_number = std::get<0>(p);
		int target_number = std::get<1>(p);
		// std::cout << "ICP from " << source_number << " to " << target_number << std::endl;

		// open files
		std::ostringstream source_file, target_file;
		source_file << config.input_dir << "/frame" << std::setw(5) << std::setfill('0') << source_number << ".ply";
		target_file << config.input_dir << "/frame" << std::setw(5) << std::setfill('0') << target_number << ".ply";
		// std::cout << "Opening " << source_file.str() << " and " << target_file.str() << std::endl;

		// Initialize point clouds
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

		// Load .ply from command line argument to point cloud variable 
		pcl::io::loadPLYFile(source_file.str(), *source_cloud);
		pcl::io::loadPLYFile(target_file.str(), *target_cloud);

		// Select type of ICP algorithm for two point clouds
		std::shared_ptr<pcl::Registration<pcl::PointXYZRGB, pcl::PointXYZRGB>> icp;
		if (config.enable_gicp)
			icp = std::make_shared<pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB>>();
		else
			icp = std::make_shared<pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB>>();

		icp->setInputSource(source_cloud);
		icp->setInputTarget(target_cloud);

		// Set criteria for ICP termination
		icp->setTransformationEpsilon(1e-8);
		//icp.setMaximumIterations(10);

		// Perform ICP
		icp->align(*source_cloud);

		if (icp->hasConverged()) {
			// std::cout << "ICP has converged, score: " << icp->getFitnessScore() << std::endl;
		}
		else {
			std::cout << "ICP failed to converge! source: " << source_number << ", target: " << target_number << ", score: " << icp->getFitnessScore() << std::endl;
			// break;
		}
	
		Eigen::Matrix4f transform = icp->getFinalTransformation();
		// std::cout << "Transformation between frames:\n" << transform << std::endl;

		// Save transformation to file
		std::ostringstream transform_file;
		transform_file << config.output_dir << "/icp" << std::setw(5) << std::setfill('0') << source_number 
			<< "_" << std::setw(5) << std::setfill('0') << target_number << ".csv";
		save_matrix(transform_file.str(), transform);

		completed_tasks++;
	}
}

int main (int argc, char** argv) {

	pcl::console::setVerbosityLevel(pcl::console::L_ERROR);

	CLI::App app{"ICP Point Cloud Registration"};

	std::string data_dir;
	int starting_frame = 0;
	int ending_frame = 0;
	bool enable_gicp = false;
	std::vector<std::tuple<int, int>> loop_closure_pair;
	int num_threads = 0;

	app.add_option("data_dir", data_dir, "Directory containing data including Filtered_Point_Clouds subdirectory with .PLY files")
		->check(CLI::ExistingDirectory)->required();
	app.add_option("-s,--start", starting_frame, "Starting frame number")
		->default_val(0);
	app.add_option("-e,--end", ending_frame, "Ending frame number")
		->required();
	app.add_flag("-g,--enable_gicp", enable_gicp, "Enable GICP for registration");
	app.add_option("-l,--loop_closure_pair", loop_closure_pair, "Add a pair of frames to perform ICP for loop closure");
	app.add_option("--num_threads", num_threads, "Number of threads to use for ICP (default is equal to number of CPU cores)");

	CLI11_PARSE(app, argc, argv);

	std::string PLY_dir = data_dir + "/Filtered_Point_Clouds";
	std::string output_dir = data_dir + "/transforms";

	if (num_threads == 0)
	{
		num_threads = std::thread::hardware_concurrency();
		std::cout << "Using " << num_threads << " threads" << std::endl;
	}

	// Create output directory in case it does not already exist
	std::filesystem::create_directory(output_dir);

	// Create vector containing all ICP tasks, including loop closure tasks
	std::vector<std::tuple<int, int>> icp_tasks = loop_closure_pair;
	for (int i = starting_frame; i < ending_frame; i++)
	{
		icp_tasks.emplace_back(i, i + 1);
	}

	// Split up all of these tasks and start the threads.
	icp_config_t config;
	config.enable_gicp = enable_gicp;
	config.input_dir = PLY_dir;
	config.output_dir = output_dir;

	std::vector<std::thread> threads;
	int start = 0;
	int jobs_per_thread = icp_tasks.size() / num_threads;
	int jobs_remainder = icp_tasks.size() % num_threads;
	for (int i = 0; i < num_threads; i++)
	{
		int end = start + jobs_per_thread + (i < jobs_remainder ? 1 : 0);
		auto jobs = std::vector<std::tuple<int, int>>(icp_tasks.begin() + start, icp_tasks.begin() + end);
	
		// Print which jobs have been assigned to each thread
		std::cout << "Thread #" << i << ": ";
		for (auto j : jobs)
			std::cout << "(" << std::get<0>(j) << ", " << std::get<1>(j) << ") ";
		std::cout << std::endl;
	
		threads.emplace_back(icp_thread, config, jobs);
		start = end;
	}

	// Print status bar while waiting for tasks to complete
	while (completed_tasks < icp_tasks.size())
	{
		print_progress(completed_tasks, icp_tasks.size());
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}

	for (std::thread& t : threads)
	{
		t.join();
	}

	std::cout << "\nICP complete, output files have been saved to " << output_dir << std::endl;

	return 0;
}