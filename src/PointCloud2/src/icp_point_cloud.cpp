// How to run: ./<executable> /path/to/ply/dir starting_frame ending_frame [voxel_size] [poly_order] [search_radius] [num_points_sor] [std_dev_trim] [enable_mls]

#include <iostream>
#include <sstream>
#include <iomanip>
#include <filesystem>
#include "CLI/CLI.hpp"

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


template <typename Point>
typename pcl::PointCloud<Point>::Ptr sor_filter(
	const typename pcl::PointCloud<Point>::ConstPtr input,
	int mean,
	double std=1) {
		
		typename pcl::PointCloud<Point>::Ptr filter_PC(new pcl::PointCloud<Point>());
		if (input->empty()) return filter_PC;

		typename pcl::StatisticalOutlierRemoval<Point> sor_filter;
		sor_filter.setInputCloud(input);
		sor_filter.setMeanK(mean);
		sor_filter.setStddevMulThresh(std);
		sor_filter.filter(*filter_PC);
		return filter_PC;
	}

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

int main (int argc, char** argv) {

	pcl::console::setVerbosityLevel(pcl::console::L_ERROR);

	CLI::App app{"ICP Point Cloud Registration"};

	std::string PLY_dir;
	int starting_frame = 0;
	int ending_frame = 0;
	float voxel_size = 0.01;
	int poly_order = 0;
	float search_radius = 0.05;
	int num_points_sor = 100;
	float std_dev_trim = 1;
	bool enable_mls = false;
	bool enable_gicp = false;

	app.add_option("PLY_dir", PLY_dir, "Directory containing PLY files")
		->required();
	app.add_option("--start", starting_frame, "Starting frame number")
		->default_val(0);
	app.add_option("--end", ending_frame, "Ending frame number")
		->required();
	app.add_option("--voxel_size", voxel_size, "Voxel size for downsampling")
		->default_val(0.01);
	app.add_option("--poly_order", poly_order, "Polynomial order for MLS")
		->default_val(0);
	app.add_option("--search_radius", search_radius, "Search radius for MLS")
		->default_val(0.05);
	app.add_option("--num_points_sor", num_points_sor, "Number of points for SOR filter")
		->default_val(100);
	app.add_option("--std_dev_trim", std_dev_trim, "Standard deviation for SOR filter")
		->default_val(1);
	app.add_option("--enable_mls", enable_mls, "Enable MLS for smoothing")
		->default_val(false);
	app.add_option("--enable_gicp", enable_gicp, "Enable GICP for registration")
		->default_val(false);

	CLI11_PARSE(app, argc, argv);

    // Check for correct number of arguments
    if (argc < 4) { 
        throw std::runtime_error("Arguments missing!\n");
        return 1;
    }

	// std::string PLY_dir = argv[1];
	// int starting_frame = atoi(argv[2]);
	// int ending_frame = atoi(argv[3]);
	// if (ending_frame <= starting_frame) {
	// 	std::cerr << "Invalid frame range: ending_frame must be > starting_frame\n";
	// 	return 1;
	// }
	// float voxel_size = 0.01;
	// int poly_order = 0;
	// float search_radius = 0.05;
	// int num_points_sor = 100;
	// float std_dev_trim = 1;
	// bool enable_mls = false;
	// if (argc >= 5)
	// 	voxel_size = atof(argv[4]);
	// if (argc >= 6)
	// 	poly_order = atoi(argv[5]);
	// if (argc >= 7)
	// 	search_radius = atof(argv[6]);
	// if (argc >= 8)
	// 	num_points_sor = atoi(argv[7]);
	// if (argc >= 9)
	// 	std_dev_trim = atof(argv[8]);
	// if (argc >= 10)
	// 	enable_mls = (atoi(argv[9]) != 0);


	std::cout << "Starting frame: " << starting_frame << "\nEnding frame: " << ending_frame << "\nVoxel size: " << voxel_size << std::endl;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr full_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	Eigen::Matrix4f full_transform = Eigen::Matrix4f::Identity();

	std::ostringstream first_file;
	first_file << PLY_dir << "/frame" << std::setw(5) << std::setfill('0') << starting_frame << ".ply";
	pcl::io::loadPLYFile(first_file.str(), *full_cloud);

	int total_frames = ending_frame - starting_frame;
	int processed_frames = 0;
	print_progress(processed_frames, total_frames);

	for (int i=starting_frame; i<ending_frame; i++) {

		// open files. TODO: it shouldn't need to open every file twice. I don't know if this is slow enough to worry about.
		std::ostringstream source_file, target_file;
		source_file << PLY_dir << "/frame" << std::setw(5) << std::setfill('0') << i+1 << ".ply";
		target_file << PLY_dir << "/frame" << std::setw(5) << std::setfill('0') << i << ".ply";
		std::cout << "Opening " << source_file.str() << " and " << target_file.str() << std::endl;

		// Initialize point clouds
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_s (new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_t (new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr final_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

		// Load .ply from command line argument to point cloud variable 
		pcl::io::loadPLYFile(source_file.str(), *cloud_s);
		pcl::io::loadPLYFile(target_file.str(), *cloud_t);

		// Run simple ICP algorithm for two point clouds
		std::shared_ptr<pcl::Registration<pcl::PointXYZRGB, pcl::PointXYZRGB>> icp;
		if (enable_gicp) {
			icp = std::make_shared<pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB>>();
		}
		else {
			icp = std::make_shared<pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB>>();
		}
			icp->setInputSource(cloud_s);
			icp->setInputTarget(cloud_t);

			// Set criteria for ICP termination
			icp->setTransformationEpsilon(1e-8);
			//icp.setMaximumIterations(10);

			// Store new aligned point cloud
			icp->align(*final_cloud);

			if (icp->hasConverged()) {
				std::cout << "ICP has converged, score: " << icp->getFitnessScore() << std::endl;
			}
			else {
				std::cout << "ICP failed to converge! (score: " << icp->getFitnessScore() << ") Quitting adding new frames." << std::endl;
				break;
			}
		
			Eigen::Matrix4f transform = icp->getFinalTransformation();
			full_transform = full_transform * transform;
			std::cout << "Transformation between frames:\n" << transform << std::endl;
			std::cout << "Full transformation back to original frame:\n" << full_transform << std::endl;
		
			// Transform the new point cloud and add to the full cloud
			pcl::transformPointCloud(*cloud_t, *cloud_t, full_transform);
			*full_cloud += *cloud_t;

		processed_frames++;
		print_progress(processed_frames, total_frames);

	}

	std::cerr << "PointCloud size before sor: " << full_cloud->width * full_cloud->height << std::endl;
	std::cerr << "Number of points: " << num_points_sor << ", Std dev: " << std_dev_trim << std::endl;
	auto sor_cloud = sor_filter<pcl::PointXYZRGB>(full_cloud, num_points_sor, std_dev_trim);
	std::cerr << "PointCloud size after sor: " << sor_cloud->width * sor_cloud->height << std::endl;

	std::cerr << "PointCloud size before VoxelGrid: " << sor_cloud->width * sor_cloud->height << std::endl;
	std::cerr << "Voxel size: " << voxel_size << std::endl;
	pcl::VoxelGrid<pcl::PointXYZRGB> vg;
	vg.setInputCloud(sor_cloud);
	vg.setLeafSize(voxel_size, voxel_size, voxel_size);
	vg.filter(*sor_cloud);

	std::cerr << "PointCloud size after VoxelGrid: " << sor_cloud->width * sor_cloud->height << std::endl;

	pcl::PointCloud<pcl::Normal>::Ptr normal(new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normal(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	auto normal_cloud = pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal>();
	normal_cloud.setViewPoint(0.0f, 0.0f, -1.0f);
	normal_cloud.setInputCloud(sor_cloud);
	auto tree = std::make_shared<pcl::search::KdTree<pcl::PointXYZRGB>>();
	normal_cloud.setSearchMethod(tree);
	normal_cloud.setKSearch(10); 

	normal_cloud.compute(*normal);
	pcl::concatenateFields(*sor_cloud, *normal, *cloud_with_normal);


	if (enable_mls) {
		// Filter output with Moving Least Squares (CloudSurfaceProcessing subclass)
		std::cout << "polynomial order: " << poly_order << ", search radius: " << search_radius << std::endl;

		// Create a KD-Tree
		pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr treeMLS(new pcl::search::KdTree<pcl::PointXYZRGBNormal>);

		// Output includes normals
		pcl::PointCloud<pcl::PointXYZRGBNormal> mls_points;

		// Init object (CloudSurfaceProcessing subclass)
		pcl::MovingLeastSquares<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> mls;
		mls.setComputeNormals(true);

		// Set parameters
		mls.setInputCloud(cloud_with_normal);
		mls.setPolynomialOrder(poly_order);
		mls.setSearchMethod(treeMLS);
		mls.setSearchRadius(search_radius);

		// Reconstruct
		mls.process(mls_points);
		pcl::io::savePLYFileBinary("smoothed.ply", mls_points);
	}

	// Save the aligned point cloud as a .ply file
    // std::filesystem::create_directory("ICP_Aligned_PC");
	pcl::io::savePLYFileBinary("final_cloud.ply", *sor_cloud);
    pcl::io::savePLYFileBinary("final_cloud_with_normals.ply", *cloud_with_normal);
    std::cout << "Saved frame.\n";

    return 0;
}
