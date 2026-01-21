// How to run: ./<executable> /path/to/ply/dir starting_frame ending_frame

#include <iostream>
#include <sstream>
#include <filesystem>
#include <pcl/io/ply_io.h>
#include <pcl/types.h>
#include <pcl/registration/icp.h>

#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/statistical_outlier_removal.h>


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

int main (int argc, char** argv) {

    // Check for correct number of arguments
    if (argc < 5) {
        throw std::runtime_error("Arguments missing!\n");
        return 1;
    }

	std::string PLY_dir = argv[1];
	int starting_frame = atoi(argv[2]);
	int ending_frame = atoi(argv[3]);
	float voxel_size = 0.01;
	int poly_order = 0;
	float search_radius = 0.05;
	int num_points_sor = 100;
	float std_dev_trim = 1;
	if (argc >= 5)
		voxel_size = atof(argv[4]);
	if (argc >= 6)
		poly_order = atoi(argv[5]);
	if (argc >= 7)
		search_radius = atof(argv[6]);
	if (argc >= 8)
		num_points_sor = atoi(argv[7]);
	if (argc >= 9)
		std_dev_trim = atof(argv[8]);


	std::cout << "Starting frame: " << starting_frame << "\nEnding frame: " << ending_frame << "\nVoxel size: " << voxel_size << std::endl;

	pcl::PointCloud<pcl::PointXYZ>::Ptr full_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	Eigen::Matrix4f full_transform = Eigen::Matrix4f::Identity();

	std::ostringstream first_file;
	first_file << PLY_dir << "/frame" << std::setw(5) << std::setfill('0') << starting_frame << ".ply";
	pcl::io::loadPLYFile(first_file.str(), *full_cloud);

	for (int i=starting_frame; i<ending_frame; i++)
	{

	// open files. TODO: it shouldn't need to open every file twice. I don't know if this is slow enough to worry about.
	std::ostringstream source_file, target_file;
	source_file << PLY_dir << "/frame" << std::setw(5) << std::setfill('0') << i+1 << ".ply";
	target_file << PLY_dir << "/frame" << std::setw(5) << std::setfill('0') << i << ".ply";
	std::cout << "Opening " << source_file.str() << " and " << target_file.str() << std::endl;

    // Initialize point clouds
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_s (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_t (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr final_cloud (new pcl::PointCloud<pcl::PointXYZ>);

    // Load .ply from command line argument to point cloud variable 
    pcl::io::loadPLYFile(source_file.str(), *cloud_s);
    pcl::io::loadPLYFile(target_file.str(), *cloud_t);

    // Run simple ICP algorithm for two point clouds
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(cloud_s);
    icp.setInputTarget(cloud_t);

    // Set criteria for ICP termination
    icp.setTransformationEpsilon(1e-8);
    //icp.setMaximumIterations(10);

    // Store new aligned point cloud
    icp.align(*final_cloud);

	if (icp.hasConverged())
	{
		std::cout << "ICP has converged, score: " << icp.getFitnessScore() << std::endl;
	}
	else
	{
		std::cout << "ICP failed to converge! (score: " << icp.getFitnessScore() << ") Quitting adding new frames." << std::endl;
		break;
	}
    
	Eigen::Matrix4f transform = icp.getFinalTransformation();
	full_transform = full_transform * transform;
	std::cout << "Transformation between frames:\n" << transform << std::endl;
	std::cout << "Full transformation back to original frame:\n" << full_transform << std::endl;
    
	// Transform the new point cloud and add to the full cloud
	pcl::transformPointCloud(*cloud_t, *cloud_t, full_transform);
	*full_cloud += *cloud_t;

	// // Visualization
	// printf(  "\nPoint cloud colors :  white  = original point cloud\n"
	//     "                        red  = transformed point cloud\n");
	// pcl::visualization::PCLVisualizer viewer ("Matrix transformation example");
	//  // Define R,G,B colors for the point cloud
	// pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler(cloud_s, 255, 255, 255);
	// // We add the point cloud to the viewer and pass the color handler
	// viewer.addPointCloud (cloud_s, source_cloud_color_handler, "original_cloud");
	// pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler (cloud_t, 230, 20, 20); // Red
	// viewer.addPointCloud (cloud_t, transformed_cloud_color_handler, "transformed_cloud");
	// viewer.addCoordinateSystem (1.0, "cloud", 0);
	// viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
	// viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
	// viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud");
	// //viewer.setPosition(800, 400); // Setting visualiser window position
	// // while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
	//   viewer.spin ();
	// // }

	}

	std::cerr << "PointCloud size before sor: " << full_cloud->width * full_cloud->height << std::endl;
	std::cerr << "Number of points: " << num_points_sor << ", Std dev: " << std_dev_trim << std::endl;
	auto sor_cloud = sor_filter<pcl::PointXYZ>(full_cloud, num_points_sor, std_dev_trim);
	std::cerr << "PointCloud size after sor: " << sor_cloud->width * sor_cloud->height << std::endl;

	std::cerr << "PointCloud size before VoxelGrid: " << sor_cloud->width * sor_cloud->height << std::endl;
	std::cerr << "Voxel size: " << voxel_size << std::endl;
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(sor_cloud);
	sor.setLeafSize(voxel_size, voxel_size, voxel_size);
	sor.filter (*sor_cloud);

	std::cerr << "PointCloud size after VoxelGrid: " << sor_cloud->width * sor_cloud->height << std::endl;


	// Filter output with moving least squares
	std::cout << "polynomial order: " << poly_order << ", search radius: " << search_radius << std::endl;

	// Create a KD-Tree
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

	// Output has the PointNormal type in order to store the normals calculated by MLS
	pcl::PointCloud<pcl::PointNormal> mls_points;

	// Init object (second point type is for the normals, even if unused)
	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;

	mls.setComputeNormals(true);

	// Set parameters
	mls.setInputCloud(sor_cloud);
	mls.setPolynomialOrder(poly_order);
	mls.setSearchMethod(tree);
	mls.setSearchRadius(search_radius);

	// Reconstruct
	mls.process(mls_points);

	// Save the aligned point cloud as a .ply file
    // std::filesystem::create_directory("ICP_Aligned_PC");
	pcl::io::savePLYFileBinary("smoothed.ply", mls_points);
    pcl::io::savePLYFileBinary("not_smoothed.ply", *sor_cloud);
    std::cout << "Saved frame.\n";

    return 0;
}