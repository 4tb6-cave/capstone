#include <iostream>
#include <sstream>
#include <filesystem>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/pcl_base.h>


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

template <typename Point>
typename pcl::PointCloud<Point>::Ptr z_crop(
	const typename pcl::PointCloud<Point>::ConstPtr input) {

		typename pcl::PointCloud<Point>::Ptr filter_PC(new pcl::PointCloud<Point>);
		if(input->empty()) return filter_PC;

		typename pcl::PassThrough<Point> PT_filter;
		PT_filter.setInputCloud(input);
		PT_filter.setFilterFieldName("z");
		PT_filter.setFilterLimits(0.0255, 0.254);
		PT_filter.filter(*filter_PC);
		return filter_PC;

	}

int main(int argc, char** argv){
	if (argc < 3) {
		std::cerr << "Missing Arguments!\n"; // Command line argument foramt: ./<executable> path/to/bag/folder <int meanSOR = 200>
			return 1;
	}

	// Initialize bag path, output folder and topic to observed.	
	std::string bag_path = argv[1];
	std::string output_folder = "Extracted_Point_Clouds";
	std::string filtered_folder = "Filtered_Point_Clouds";
	std::string topic = "/cloud";

	std::filesystem::create_directory(output_folder);
	std::filesystem::create_directory(filtered_folder);
	// Read the specified bag folder
	std::cout << "Reading bag: " << argv[1] << std::endl;
	rosbag2_cpp::Reader reader; 
	reader.open(bag_path);
	std::size_t frame_id = 0;

	int mean = std::stoi(argv[2]);
	double std = 1;
	std::cout << "SOR Filter with mean: " << mean << std::endl;
	
	while(reader.has_next()) { // Read until message in bag has ended
		auto bag_msg = reader.read_next();

		if (bag_msg->topic_name == topic){ // Check if the recorded topic in bag matches with actual topic to be observed
			auto msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
			rclcpp::Serialization<sensor_msgs::msg::PointCloud2> ser_data;
			rclcpp::SerializedMessage ser_msg(*bag_msg->serialized_data);
			ser_data.deserialize_message(&ser_msg, msg.get());
			
			// Initialize an empty point cloud to store data from the ROS bag message
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::fromROSMsg(*msg, *cloud);
			
			auto sor_cloud = sor_filter<pcl::PointXYZ>(cloud, mean, std);
			auto filtered_cloud = z_crop<pcl::PointXYZ>(sor_cloud);

			// Writes and saves data to separate .ply files
			std::ostringstream oss, oss_f;

			oss << output_folder << "/frame" << frame_id << ".ply";
			pcl::io::savePLYFileBinary(oss.str(), *cloud);
			std::cout << "Saved " << oss.str() << " (" << (*cloud).size() << " points)\n";

			oss_f << filtered_folder << "/frame" << frame_id++ << ".ply";
			pcl::io::savePLYFileBinary(oss_f.str(), *filtered_cloud);
			std::cout << "Saved " << oss_f.str() << " (" << (*filtered_cloud).size() << " points)\n";

		}
	}
	std::cout << "Saved " << frame_id << "frames\n";
	return 0;

}
