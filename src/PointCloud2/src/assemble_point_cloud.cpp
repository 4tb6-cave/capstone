
#include <iostream>
#include <sstream>
#include <iomanip>
#include <filesystem>
#include "CLI11.hpp"
#include <vector>
#include <tuple>

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
#include <pcl/filters/random_sample.h>


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

/**
 * Load matrix from CSV file
 */
bool loadMatrix4d(const std::string &filename, Eigen::Matrix4d &matrix)
{
	std::ifstream file(filename);
	if (!file.is_open())
	{
		std::cerr << "Failed to open " << filename << std::endl;
		return false;
	}

	std::string line;
	int row = 0;

	while (std::getline(file, line) && row < 4)
	{
		std::stringstream ss(line);
		std::string value;
		int col = 0;

		while (std::getline(ss, value, ',') && col < 4)
		{
			matrix(row, col) = std::stof(value);
			col++;
		}

		if (col != 4)
		{
			std::cerr << "Invalid data in " << filename << ": too many columns" << std::endl;
			return false;
		}
		row++;
	}

	if (row != 4)
	{
		std::cerr << "Invalid data in " << filename << ": too many rows" << std::endl;
		return false;
	}
	return true;
}

int main (int argc, char** argv) {

	pcl::console::setVerbosityLevel(pcl::console::L_ERROR);

	CLI::App app{"Point Cloud Assembly"};

	std::string data_dir;
	int starting_frame = 0;
	int ending_frame = 0;
	float voxel_size = 0.01;
	int poly_order = 0;
	float search_radius = 0.05;
	int num_points_sor = 100;
	float std_dev_trim = 1;
	bool enable_mls = false;
	float random_sample = 1;

	app.add_option("data_dir", data_dir, "Directory containing data, including PLY files in Filtered_Point_Clouds and poses")
		->check(CLI::ExistingDirectory)->required();
	app.add_option("-s,--start", starting_frame, "Starting frame number")
		->default_val(0);
	app.add_option("-e,--end", ending_frame, "Ending frame number")
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
	app.add_flag("--enable_mls", enable_mls, "Enable MLS for smoothing");
	app.add_option("--random_sample", random_sample, "Discard random points before filtering. Default (1.0) keeps all points")
		->check(CLI::Range(0.0, 1.0))->default_val(1);

	CLI11_PARSE(app, argc, argv);

	std::string PLY_dir = data_dir + "/Filtered_Point_Clouds";
	std::string pose_dir = data_dir + "/poses";

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr full_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

	for (int i=starting_frame; i <= ending_frame; i++) {

		// Open files
		std::ostringstream ply_file, transform_file;
		ply_file << PLY_dir << "/frame" << std::setw(5) << std::setfill('0') << i << ".ply";
		transform_file << pose_dir << "/pose" << std::setw(5) << std::setfill('0') << i << ".csv";
		std::cout << "Opening " << ply_file.str() << " and " << transform_file.str() << std::endl;

		// Initialize point clouds
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr new_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

		// Load .ply from command line argument to point cloud variable 
		pcl::io::loadPLYFile(ply_file.str(), *new_cloud);

		Eigen::Matrix4d transform;
		if (!loadMatrix4d(transform_file.str(), transform))
		{
			std::cerr << "Failed to load " << transform_file.str() << ", skipping frame " << i << std::endl;
			continue;
		}
		
		// std::cout << "Transformation for frame " << i << ": \n" << transform << std::endl;
	
		// Transform the new point cloud and add to the full cloud
		Eigen::Matrix4d transform_inverse = transform.inverse();
		pcl::transformPointCloud(*new_cloud, *new_cloud, transform_inverse);
		*full_cloud += *new_cloud;
	}

	// Optional random downsampling (to remove excess points)
	if (random_sample < 1.0)
	{
		int original_size = full_cloud->size();
		pcl::RandomSample<pcl::PointXYZRGB> rs;
		rs.setInputCloud(full_cloud);
		rs.setSample(static_cast<int>(full_cloud->size() * random_sample));
		rs.filter(*full_cloud);
		std::cout << "Random sampling: " << original_size << " --> " << full_cloud->size() << std::endl;
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
		pcl::io::savePLYFileBinary(data_dir + "/smoothed.ply", mls_points);
	}

	// Save the aligned point cloud as a .ply file
    // std::filesystem::create_directory("ICP_Aligned_PC");
	pcl::io::savePLYFileBinary(data_dir + "/final_cloud.ply", *sor_cloud);
    pcl::io::savePLYFileBinary(data_dir + "/final_cloud_with_normals.ply", *cloud_with_normal);
    std::cout << "Saved frame.\n";

    return 0;
}
