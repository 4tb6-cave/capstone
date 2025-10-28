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

int main(int argc, char** argv){
	if (argc < 2) {
		std::cerr << "Missing Arguments!\n"; // Command line argument foramt: ./<executable> path/to/bag/folder
			return 1;
	}
	// Initialize bag path, output folder and topic to observed.	
	std::string bag_path = argv[1];
	std::string output_folder = "Extracted point clouds";
	std::string topic = "/cloud";

	std::filesystem::create_directory(output_folder);
	// Read the specified bag folder
	std::cout << "Reading bag: " << argv[1] << std::endl;
	rosbag2_cpp::Reader reader; 
	reader.open(bag_path);
	
	std::size_t frame_id = 0;
	while(reader.has_next()) { // Read until message in bag has ended
		auto bag_msg = reader.read_next();

		if (bag_msg->topic_name == topic){ // Check if the recorded topic in bag matches with actual topic to be observed
			auto msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
			rclcpp::Serialization<sensor_msgs::msg::PointCloud2> ser_data;
			rclcpp::SerializedMessage ser_msg(*bag_msg->serialized_data);
			ser_data.deserialize_message(&ser_msg, msg.get());
			// Initialize an empty point cloud to store data from the ROS bag message
			pcl::PointCloud<pcl::PointXYZ> cloud;
			pcl::fromROSMsg(*msg, cloud);

			// Writes and saves data to separate .ply files
			std::ostringstream oss;
			oss << output_folder << "/frame" << frame_id++ << ".ply";

			pcl::io::savePLYFileBinary(oss.str(), cloud);
			std::cout << "Saved " << oss.str() << "(" << cloud.size() << " points)\n";

		}
	}
	std::cout << "Saved " << frame_id << "frames\n";
	return 0;

}
