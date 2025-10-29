// How to run: ./<executable> /path/to/source.ply /path/to/target.ply

#include <iostream>
#include <sstream>
#include <filesystem>
#include <pcl/io/ply_io.h>
#include <pcl/types.h>
#include <pcl/registration/icp.h>

int main (int argc, char** argv) {

    // Check for correct number of arguments
    if (argc < 3) {
        throw std::runtime_error("Arguments missing!\n");
        return 1;
    }

    std::string source_PLY_file = argv[1];
    std::string target_PLY_file = argv[2];

    // Initialize point clouds
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_s (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_t (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr final_cloud (new pcl::PointCloud<pcl::PointXYZ>);

    // Load .ply from command line argument to point cloud variable 
    pcl::io::loadPLYFile(source_PLY_file, *cloud_s);
    pcl::io::loadPLYFile(target_PLY_file, *cloud_t);

    // Run simple ICP algorithm for two point clouds
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(cloud_s);
    icp.setInputTarget(cloud_t);

    // Set criteria for ICP termination
    icp.setTransformationEpsilon(1e-8);
    //icp.setMaximumIterations(10);

    // Store new aligned point cloud
    icp.align(*final_cloud);

    std::cout << "ICP has " << (icp.hasConverged()?"coverged":"not converged") << ", score: " << icp.getFitnessScore() << std::endl;
    std::cout << icp.getFinalTransformation() << std::endl;
    
    // Save the aligned point cloud as a .ply file
    std::filesystem::create_directory("ICP_Aligned_PC");
    pcl::io::savePLYFileBinary("ICP_Aligned_PC/aligned_frame.ply", *final_cloud);
    std::cout << "Saved frame.\n";

    return 0;

}