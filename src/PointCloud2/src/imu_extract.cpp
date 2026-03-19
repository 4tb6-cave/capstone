#include <iostream>
#include <sstream>
#include <fstream>
#include <iomanip>
#include <filesystem>

#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>

#include <sensor_msgs/msg/imu.hpp>

#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>

double timeStampToSec(const builtin_interfaces::msg::Time& t) {
  return double(t.sec) + 1e-9 * double(t.nanosec);
}
int main (int argc, char** argv) {

  if (argc < 3) { // How to run: ./executable <path/to/bag/folder> <path/to/output> [imu_topic]
    std::cerr << "Missing Arguments! | <required> ; [optional] --> ./executable <path/to/bag/folder> <path/to/output> [imu_topic]\n";
    return 1;
  }
  
  std::string bag_path = argv[1];
  std::string output_path = argv[2];
  std::string imu_topic = (argc >= 4) ? std::string(argv[3]) : "/imu";
 
  std::filesystem::create_directory(output_path);
  std::string output_filename = output_path + "/imu.csv";
  std::ofstream imu_file(output_filename);
  if (!imu_file.is_open()) {
    std::cerr << "Failed to open output file: " << output_filename << std::endl;
    return 1;
  }
  imu_file << "time,o_x,o_y,o_z,o_w,av_x,av_y,av_z,la_x,la_y,la_z\n";
  imu_file << std::fixed << std::setprecision(9);

    std::cout << "Reading bag: " << bag_path << std::endl;

    rosbag2_cpp::Reader reader;
    reader.open(bag_path);

    std::size_t imu_id = 0;

    rclcpp::Serialization<sensor_msgs::msg::Imu> imu_ser;
	double t_last = 0;

    while(reader.has_next()){
        auto bag_msg = reader.read_next();

        if(bag_msg->topic_name == imu_topic) {
            sensor_msgs::msg::Imu imu_msg;
            rclcpp::SerializedMessage ser_msg(*bag_msg->serialized_data);
            imu_ser.deserialize_message(&ser_msg, &imu_msg);

            double t = timeStampToSec(imu_msg.header.stamp);

			if (t < t_last)
				std::cout << std::fixed << std::setprecision(9) << "out of order! t=" << t << ", t_last=" << t_last << std::endl;
			
			t_last = t;

            imu_file << t << "," << imu_msg.orientation.x << "," << imu_msg.orientation.y << "," << imu_msg.orientation.z << "," << imu_msg.orientation.w << ","
                << imu_msg.angular_velocity.x << "," << imu_msg.angular_velocity.y << "," << imu_msg.angular_velocity.z << ","
                << imu_msg.linear_acceleration.x << "," << imu_msg.linear_acceleration.y << "," << imu_msg.linear_acceleration.z << "\n";

            imu_id++;
        }
    }
    std::cout << "IMU file created: " << output_filename << ", saved " << imu_id << " measurements" << std::endl;
    return 0;

}
