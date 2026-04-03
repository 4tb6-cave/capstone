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

#include "CLI11.hpp"

#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgcodecs.hpp>

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

	CLI::App app{"Extract point clouds from ROS2 bag"};

	std::string bag_path;
	std::string output_dir;
	bool disable_sor = false;
	int sor_num_points = 150;
	double sor_std_dev = 1.0;
	std::string topic = "/cloud";
	std::string image_topic = "/depth";

	app.add_option("bag_path", bag_path, "Directory containing ROS2 bag")
		->check(CLI::ExistingDirectory)->required();
	app.add_option("output_dir", output_dir, "Directory in which to save output results")
		->check(CLI::ExistingDirectory)->required();
	app.add_flag("--disable_sor", disable_sor, "Disable SOR filter")
		->default_val(false);
	app.add_option("--sor_num_points", sor_num_points, "Number of points for SOR filter")
		->default_val(150);
	app.add_option("--sor_std_dev", sor_std_dev, "Standard deviation for SOR filter")
		->default_val(1.0);
	app.add_option("--topic", topic, "Topic name of point cloud messages")
		->default_val("/cloud");
	app.add_option("--image_topic", image_topic, "Topic name of image messages")
    	->default_val("/image");

	CLI11_PARSE(app, argc, argv);

	// Initialize output folders.	
	std::string output_folder = output_dir + "/Extracted_Point_Clouds";
	std::string filtered_folder = output_dir + "/Filtered_Point_Clouds";
	std::string image_folder = output_dir + "/Extracted_Images";
	std::filesystem::create_directory(image_folder);
	std::filesystem::create_directory(output_folder);
	std::filesystem::create_directory(filtered_folder);

	std::string time_stamp_filename = output_dir + "/time_stamps.csv";
	std::ofstream time_stamps(time_stamp_filename);
	if (!time_stamps.is_open()) {
		std::cerr << "Failed to open output file: " << time_stamp_filename << std::endl;
		return 1;
	}
	time_stamps << "ID,Time\n" ;
	time_stamps << std::fixed << std::setprecision(9);

	// Read the specified bag folder
	std::cout << "Reading bag: " << argv[1] << std::endl;
	rosbag2_cpp::Reader reader; 
	reader.open(bag_path);
	std::size_t frame_id = 0;

	std::cout << "SOR Filter with std dev: " << sor_std_dev << ", num points: " << sor_num_points << std::endl;
	
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

			auto filtered_cloud = z_crop<pcl::PointXYZRGB>(cloud);
			if (!disable_sor)
			{
				*filtered_cloud = *(sor_filter<pcl::PointXYZRGB>(filtered_cloud, sor_num_points, sor_std_dev));
			}

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

		// ------------------ IMAGE ------------------
		else if (bag_msg->topic_name == image_topic) {
			auto img_msg = std::make_shared<sensor_msgs::msg::Image>();
			rclcpp::Serialization<sensor_msgs::msg::Image> ser_data;
			rclcpp::SerializedMessage ser_msg(*bag_msg->serialized_data);
			ser_data.deserialize_message(&ser_msg, img_msg.get());

			try {
				// Convert to OpenCV image
				cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img_msg, img_msg->encoding);

				// Build filename
				static size_t image_id = 0;
				std::ostringstream oss;
				oss << image_folder << "/image" << std::setfill('0') << std::setw(5) << image_id++ << ".png";

				// Save image
				cv::imwrite(oss.str(), cv_ptr->image);

			} catch (cv_bridge::Exception& e) {
				std::cerr << "cv_bridge exception: " << e.what() << std::endl;
			}
		}
	}
	std::cout << "Saved " << frame_id << " frames\n";
	return 0;

}
