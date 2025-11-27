// How to run: <executable> path/to/point-cloud/dir

//#include <typeinfo>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <filesystem>
#include <vector>
#include <algorithm>
#include <pcl/io/ply_io.h>
#include <pcl/types.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/console/print.h>


namespace fs = std::filesystem;

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr voxelDownsample(
    typename pcl::PointCloud<PointT>::Ptr inputCloud,
    float leafSize) {

        typename pcl::PointCloud<PointT>::Ptr filteredCloud (new pcl::PointCloud<PointT>());
        typename pcl::VoxelGrid<PointT> voxel;
        voxel.setInputCloud(inputCloud);
        voxel.setLeafSize(leafSize, leafSize, leafSize);
        voxel.filter(*filteredCloud);
        return filteredCloud;
    }


int main (int argc, char** argv) {

    pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
    
    if (argc < 3) {
        std::cerr << ("Arguments missing!\n");
        return 1;
    }
    
    float leafSize = std::stof(argv[2]);

    std::string pcDir = argv[1];
    std::string sourcePLYFile;
    std::string targetPLYFile;

    std::string outputDir = "Aligned ICP Point Clouds";
    fs::create_directory(outputDir);

    if (!(fs::is_directory(pcDir))) {
        std::cout << std::string(pcDir) <<" is not a directory\n";
        return 1;
    }
    
    std::vector<std::string> frames;
    
    for (auto const& plyFile : fs::directory_iterator{pcDir}) {
        if (plyFile.path().extension()==".ply") {
            frames.push_back(plyFile.path());
        }
    }
    std::sort (frames.begin(), frames.end());

    for (auto i : frames) {
        std::cout << i << "\n";
    }
    
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudS (new pcl::PointCloud<pcl::PointXYZRGBNormal>());
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudT (new pcl::PointCloud<pcl::PointXYZRGBNormal>());
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr alignedCloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>());        
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr GlobalCloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>());

    pcl::IterativeClosestPointWithNormals<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> icp;

    Eigen::Matrix4f tGlobal = Eigen::Matrix4f::Identity();
    
    for (size_t i = 0; i<frames.size()-1; i++) {
        std::cout << "This is the i-th frame: \t" << frames[i] << std::endl;
        std::cout << "This is the [i+1]-th frame: \t" << frames[i+1] << std::endl;
        std::cout << "===================================\n";
        sourcePLYFile = frames[i];
        targetPLYFile = frames[i+1];

        std::cout << sourcePLYFile << std::endl;

        if (pcl::io::loadPLYFile(sourcePLYFile, *cloudS) != 0) {
            std::cerr << "Error Loading in: " << sourcePLYFile << "\n";
            return 1;
        }
        if (pcl::io::loadPLYFile(targetPLYFile, *cloudT) != 0) {
            std::cerr << "Error Loading in: " << targetPLYFile << "\n";
            return 1;
        }

        icp.setInputSource(cloudS);
        icp.setInputTarget(cloudT);
        icp.setTransformationEpsilon(1e-8);
        icp.setMaximumIterations(60);
        //icp.setMaxCorrespondenceDistance(5*leafSize);
        icp.setEuclideanFitnessEpsilon(1e-8);
        icp.align(*alignedCloud);

        if (icp.hasConverged()) {

            std::cout << "ICP has converged for point cloud: " << i << " -> " << (i+1) << "\t| Score: " << icp.getFitnessScore() << "\n";
            Eigen::Matrix4f tRel = icp.getFinalTransformation();
            tGlobal = tRel * tGlobal;
            pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr currCloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>());
            
            pcl::transformPointCloud<pcl::PointXYZRGBNormal>(*cloudS, *currCloud, tGlobal, 1);

            *GlobalCloud += *currCloud;

            GlobalCloud = voxelDownsample<pcl::PointXYZRGBNormal>(GlobalCloud, leafSize);

            std::ostringstream oss;
            oss << outputDir << "/AlignedFrame" << std::setfill('0') << std::setw(5) << i << ".ply";
            pcl::io::savePLYFileBinary(oss.str(), *alignedCloud);

        }
        else {
            std::cout << "ICP has not converged!\n";
        }
        
    }

    std::ostringstream oss;
    oss << outputDir << "/FinalRegistration.ply";
    pcl::io::savePLYFileBinary(oss.str(), *GlobalCloud);

    return 0;

    

}
