#include <iostream>
#include <sstream>
#include <iomanip>
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
#include <pcl/features/normal_3d.h>

// Time stamp conversion function
double timeStampToSec(const builtin_interfaces::msg::Time& t) {
  return double(t.sec) + 1e-9 * double(t.nanosec);
}

// Statistical Outlier Removal filter
template <typename Point>
typename pcl::PointCloud<Point>::Ptr sor_filter(
	const typename pcl::PointCloud<Point>::ConstPtr input,
	int mean,
	double std) {
		
		typename pcl::PointCloud<Point>::Ptr filter_PC(new pcl::PointCloud<Point>());
		if (input->empty()) return filter_PC;

		typename pcl::StatisticalOutlierRemoval<Point> sor_filter;
		sor_filter.setInputCloud(input);
		sor_filter.setMeanK(mean);
		sor_filter.setStddevMulThresh(std);
		sor_filter.filter(*filter_PC);
		return filter_PC;
	}

// Z-axis cropping filter
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
	if (argc < 2) {
		std::cerr << "Missing Arguments! --> ./point_cloud2_extract <path/to/bag/folder> [Mean for SOR] [Std. Dev for SOR] [cloud_topic]\n"; // CLI arguments
			return 1;
	}

	// Initialize bag path, output folder and topic to observed.	
	std::string bag_path = argv[1];
	std::string output_folder = "Extracted_Point_Clouds_(" + bag_path + ")";
	std::string filtered_folder = "Filtered_Point_Clouds_(" + bag_path + ")";

	int mean = (argc >= 3) ? std::stoi(argv[2]) : 150;
	double std = (argc >= 4) ? std::stoi(argv[3]) : 1.0;
	std::string topic = (argc >= 5) ? argv[4] : "/cloud";

	std::filesystem::create_directory(output_folder);
	std::filesystem::create_directory(filtered_folder);

	std::ofstream time_stamps(bag_path + "_time_stamps.csv");
	if (!time_stamps.is_open()) {
		std::cerr << "Failed to open output file: time_stamps.csv\n";
		return 1;
	}
	time_stamps << "ID,Time\n" ;
	time_stamps << std::fixed << std::setprecision(9);

	// Read the specified bag folder
	std::cout << "Reading bag: " << argv[1] << std::endl;
	rosbag2_cpp::Reader reader; 
	reader.open(bag_path);
	std::size_t frame_id = 0;

	std::cout << "SOR Filter with mean: " << mean << std::endl;
	
	while(reader.has_next()) { // Read until message in bag has ended
		auto bag_msg = reader.read_next();

		if (bag_msg->topic_name == topic){ // Check if the recorded topic in bag matches with actual topic to be observed
			auto msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
			rclcpp::Serialization<sensor_msgs::msg::PointCloud2> ser_data;
			rclcpp::SerializedMessage ser_msg(*bag_msg->serialized_data);
			ser_data.deserialize_message(&ser_msg, msg.get());
			
      double t = timeStampToSec(msg->header.stamp);
      time_stamps << frame_id << "," << t << "\n";
			// Initialize an empty point cloud to store data from the ROS bag message
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
			//pcl::PointCloud<pcl::Normal>::Ptr normal(new pcl::PointCloud<pcl::Normal>);
			//pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normal(new pcl::PointCloud<pcl::PointXYZRGBNormal>);

			pcl::fromROSMsg(*msg, *cloud);
			
			auto sor_cloud = sor_filter<pcl::PointXYZRGB>(cloud, mean, std);
			auto filtered_cloud = z_crop<pcl::PointXYZRGB>(sor_cloud);
			
			//auto normal_cloud = pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal>();
			//normal_cloud.setInputCloud(filtered_cloud);
			//auto tree = std::make_shared<pcl::search::KdTree<pcl::PointXYZRGB>>();
			//normal_cloud.setSearchMethod(tree);
			//normal_cloud.setKSearch(20); 

			//normal_cloud.compute(*normal);
			//pcl::concatenateFields(*filtered_cloud, *normal, *cloud_with_normal);

			// Writes and saves data to separate .ply files
			std::ostringstream oss, oss_f;

			oss << output_folder << "/frame" << std::setfill('0') << std::setw(5) << frame_id << ".ply";
			pcl::io::savePLYFileBinary(oss.str(), *cloud);
			//std::cout << "Saved " << oss.str() << " (" << (*cloud).size() << " points)\n";

			oss_f << filtered_folder << "/frame" << std::setfill('0') << std::setw(5) << frame_id++ << ".ply";
			pcl::io::savePLYFileBinary(oss_f.str(), *filtered_cloud);
			//std::cout << "Saved " << oss_f.str() << " (" << (*cloud_with_normal).size() << " points)\n";

		}
	}
	std::cout << "Saved " << frame_id << " frames\n";
	return 0;

}
