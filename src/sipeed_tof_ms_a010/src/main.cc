#include <cv_bridge/cv_bridge.h>

#include <sys/socket.h>   // socket(), connect()
#include <sys/un.h>      // struct sockaddr_un
#include <unistd.h>      // close()

#include <chrono>
#include <iostream>
#include <sstream>        // Added for stringstream
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>

// Add this include for file stream operations
#include <fstream>
// Add this include for hex printing helper
#include <cstdio>

#include "cJSON.h"
#include "frame_struct.h"
#include "serial.hh"

extern frame_t *handle_process(std::string s);

using namespace std::chrono_literals;

// Helper function to print string content as hex bytes with spaces
void printHex(const std::string& str) {
    for (unsigned char c : str) {
        printf("%02X ", c);
    }
}

class SipeedTOF_MSA010_Publisher : public rclcpp::Node {
 private:
  Serial *pser;
  float uvf_parms[4];

public:
  SipeedTOF_MSA010_Publisher() : Node("sipeed_tof_ms_a010") {
      std::string s;
      this->declare_parameter("device", "/dev/ttyS0");
      this->declare_parameter("output_topic_num", "one");
      rclcpp::Parameter device_param = this->get_parameter("device");
      rclcpp::Parameter output_topic_num_param = this->get_parameter("output_topic_num");

      std::cout << "Device Param: " << device_param.as_string() << std::endl;
      std::cout << "Output Topic Param: " << output_topic_num_param.as_string() << std::endl;

      std::string output_pointcloud_topic = "cloud_" + output_topic_num_param.as_string();
      std::string output_depth_topic = "depth_" + output_topic_num_param.as_string();
      output_frame_id = "tof_" + output_topic_num_param.as_string();

      s = device_param.as_string();
      std::cout << "use device: " << s << std::endl;
      pser = new Serial(s);

      auto run_tof_cmd = [&](const std::string& cmd, const std::string& search_str = "", int max_retries = 15) -> bool {
        s.clear();

        for (int attempts_counter = 0; attempts_counter < max_retries; attempts_counter++) {
          // Send command only on every other attempt (0, 2, 4...)
          if (!cmd.empty() && attempts_counter % 2 == 0) {
              std::cout << "sending command: " << cmd << std::endl;
              *pser << cmd;
              std::this_thread::sleep_for(std::chrono::milliseconds(50));
          }

          if (search_str.empty()) return true;

          *pser >> s;

          std::cout << "(Attempt " << attempts_counter << "/" << max_retries << ")" << std::endl;
#ifdef DEBUG
          std::cout << "Raw Response: " << s << ", in hex: ";
          printHex(s);
          std::cout << std::endl;
#endif

          if (s.find(search_str) != std::string::npos) {
              return true;
          }

          std::cout << "Error checking response for: " << cmd << std::endl;
          std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }

        return false;
      };


      // --- Initialization Sequence ---

      // Reboot device serial port
      std::cout << "Rebooting tof sensor..." << std::endl;
      if (!run_tof_cmd("AT+DISP=1\r", "OK")) {
          RCLCPP_ERROR(this->get_logger(), "Failed to reboot device");
          return; // Exit constructor early on failure (if safe) or throw exception
      }
      std::cout << "Rebooted tof machine successfully" << std::endl;

      // Check device connectivity
      std::cout << "Checking device connection..." << std::endl;
      if (!run_tof_cmd("AT\r", "OK")) {
          RCLCPP_ERROR(this->get_logger(), "Failed to check device");
          return;
      }


      coeff_retry:
      pser->clearBuffer();  // Clear any previous data
      run_tof_cmd("AT+COEFF?\r");
      // Get Coefficients - use buffered read instead of retry loop
      std::cout << "Getting coefficients..." << std::endl;
      if (!pser->readUntilComplete(s, 2000)) {  // 1 second timeout for full JSON
          RCLCPP_ERROR(this->get_logger(), "Failed to get coefficient (timeout)");
          return;
      }

      std::cout << "Coefficients received:" << s << std::endl;

      auto coeffs = s;

      //Verify OK response follows
      // if (!run_tof_cmd("", "OK\n", 5)) {
      if (!run_tof_cmd("AT\r", "OK", 3)) {
          RCLCPP_WARN(this->get_logger(), "Expected OK not found after COEFF query, retrying...");
          goto coeff_retry;
      }

      std::cout << "coeff check passed" << std::endl;

      if (coeffs.length() > 0) {
          cJSON *cparms = cJSON_ParseWithLength((const char *)coeffs.c_str(), coeffs.length());
          if (!cparms) {
              RCLCPP_ERROR(this->get_logger(), "Failed to parse coefficient JSON");
              return;
          }

          uvf_parms[0] = ((float)((cJSON_GetObjectItem(cparms, "fx")->valueint) / 262144.0f));
          uvf_parms[1] = ((float)((cJSON_GetObjectItem(cparms, "fy")->valueint) / 262144.0f));
          uvf_parms[2] = ((float)((cJSON_GetObjectItem(cparms, "u0")->valueint) / 262144.0f));
          uvf_parms[3] = ((float)((cJSON_GetObjectItem(cparms, "v0")->valueint) / 262144.0f));

          cJSON_Delete(cparms); // Clean up cJSON object
      } else {
          RCLCPP_ERROR(this->get_logger(), "Coefficient string is empty");
      }

      std::cout << "fx: " << uvf_parms[0] << std::endl;
      std::cout << "fy: " << uvf_parms[1] << std::endl;
      std::cout << "u0: " << uvf_parms[2] << std::endl;
      std::cout << "v0: " << uvf_parms[3] << std::endl;

      // set quant
      if (!run_tof_cmd("AT+UNIT=5\r", "OK")) {
          RCLCPP_WARN(this->get_logger(), "Failed to set unit");
          return;
      }

      if (!run_tof_cmd("AT+FPS=7\r", "OK")) {
          RCLCPP_WARN(this->get_logger(), "Failed to set FPS");
          return;
      }

      std::cout << "Increasing baud rate..." << std::endl;

      *pser << "AT+BAUD=5\r"; // 5 is 921600

      int baud = 921600;
      if (pser->setBaudrate(baud)) {
        RCLCPP_INFO(this->get_logger(), "Host baudrate updated to %d", baud);
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "Failed to set host baudrate!");
        return;
      }

      *pser >> s;
      s.clear(); // clear s since we miss the last message (or just read garbage)


      if (!run_tof_cmd("AT+DISP=4\r", "OK")) { // mfw hours wasted because i did not tell the sensor to output point clouds on uart
          RCLCPP_WARN(this->get_logger(), "Failed to enable UART");
          return;
      }

      /* do not delete it. It is waiting */
      *pser >> s;
#ifdef DEBUG
      std::cout << "Raw received (" << s.length() << " bytes): ";
      printHex(s);
      std::cout << std::endl;
#endif

      // --- Setup Publishers & Timer ---
      publisher_depth = this->create_publisher<sensor_msgs::msg::Image>(output_depth_topic, 10);
      publisher_pointcloud = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_pointcloud_topic, 10);
      timer_ = this->create_wall_timer(30ms, std::bind(&SipeedTOF_MSA010_Publisher::timer_callback, this));

      fps_check_timer_ = this->create_wall_timer(1s, std::bind(&SipeedTOF_MSA010_Publisher::check_fps, this));
  }


  ~SipeedTOF_MSA010_Publisher() {
      if (pser) {
          delete pser;
      }
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_depth;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_pointcloud;
  std::string output_frame_id;

  // FPS tracking variables
  std::deque<double> recent_frame_times;
  const int max_frames_to_track = 20;
  rclcpp::TimerBase::SharedPtr fps_check_timer_;
  double last_fps_print_time = 0.0;

  // Helper to send IMU trigger via Unix Socket
  static void send_imu_trigger() {
      const char *socket_path = "/record/imu_trigger";
      int sockfd = socket(AF_UNIX, SOCK_STREAM, 0);
      if (sockfd == -1) return;

      struct sockaddr_un addr{};
      addr.sun_family = AF_UNIX;
      strncpy(addr.sun_path, socket_path, sizeof(addr.sun_path)-1);

      if (connect(sockfd, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) == -1) {
          close(sockfd);
          return;
      }

      const char *msg = "trigger\n";
      write(sockfd, msg, strlen(msg));

      close(sockfd);
  }

  void timer_callback() {
    std::string s;
    std::stringstream sstream;
    frame_t *f = nullptr;
    int consecutive_empty = 0;
    const int max_consecutive_empty = 10;

    // Clear buffer before reading to ensure clean state
    pser->clearBuffer();

    while (true) {
        // Read from serial with timeout
        *pser >> s;

        if (s.empty()) {
            consecutive_empty++;

            // If we've had too many empty reads, sensor may be unresponsive
            if (consecutive_empty > max_consecutive_empty) {
                RCLCPP_WARN(this->get_logger(), "ToF sensor not responding after %d attempts",
                          consecutive_empty);
                consecutive_empty = 0;
                return;  // Exit and try next timer cycle
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;  // Try again
        }

        consecutive_empty = 0;
        break;  // Got valid data, proceed with processing
    }

    // Debug: Print raw response
#ifdef DEBUG
    std::cout << "Raw received (" << s.length() << " bytes): ";
    printHex(s);
    std::cout << std::endl;
#endif

    // Process the frame data
    f = handle_process(s);
    if (!f) {
        // RCLCPP_WARN(this->get_logger(), "Failed to process frame, retrying...");
        return;  // Invalid frame, skip this cycle
    }
    send_imu_trigger(); // to make it even more accurate, call it as soon as frame is validated but before handle_process parses the entire string

    uint8_t rows, cols, *depth;
    rows = f->frame_head.resolution_rows;
    cols = f->frame_head.resolution_cols;
    depth = f->payload;

    cv::Mat md(rows, cols, CV_8UC1, depth);
    sstream << md.size();

    std_msgs::msg::Header header;
    header.stamp = this->get_clock()->now();
    header.frame_id = output_frame_id;

    sensor_msgs::msg::Image msg_depth =
        *cv_bridge::CvImage(header, "mono8", md).toImageMsg().get();

#ifdef DEBUG
    RCLCPP_INFO(this->get_logger(), "Publishing: depth:%s", sstream.str().c_str());
#endif

    publisher_depth->publish(msg_depth);

#ifdef DEBUG
    RCLCPP_INFO(this->get_logger(), "Processing frame: rows=%d, cols=%d", (int)rows, (int)cols);
#endif

    sensor_msgs::msg::PointCloud2 pcmsg;
    pcmsg.header = header;
    pcmsg.height = rows;
    pcmsg.width = cols;
    pcmsg.is_bigendian = false;
    pcmsg.point_step = 16; // x, y, z (float), rgb (uint32)
    pcmsg.row_step = pcmsg.point_step * rows;
    pcmsg.is_dense = false;

    // Calculate expected data size for logging
    size_t expected_data_size = static_cast<size_t>(rows) * cols * pcmsg.point_step;

#ifdef DEBUG
    RCLCPP_INFO(this->get_logger(), "Allocating point cloud buffer: %zu bytes", expected_data_size);
#endif

    pcmsg.data.resize(expected_data_size, 0x00);

    // Safety Check: Ensure data allocation succeeded (rare but possible on low memory)
    if (pcmsg.data.size() != expected_data_size) {
        RCLCPP_ERROR(this->get_logger(), "Point cloud buffer resize failed. Expected %zu got %zu",
                     expected_data_size, pcmsg.data.size());
        free(f);
        return;
    }

    uint8_t *ptr = pcmsg.data.data();

    // Check calibration parameters to avoid division by zero (NaN/Inf)
    float fox = uvf_parms[0];
    float foy = uvf_parms[1];
    float u0 = uvf_parms[2];
    float v0 = uvf_parms[3];

    if (fox == 0.0f || foy == 0.0f) {
        RCLCPP_ERROR(this->get_logger(), "Calibration parameters fx or fy are zero! Using defaults.");
        fox = 1.0f;
        foy = 1.0f;
    }

    // Initialize fields correctly using an index counter instead of .size() to avoid OOB write
    int field_idx = 0;
    auto add_field = [&](const std::string& name, int offset, uint8_t datatype, int count) {
        if (field_idx >= static_cast<int>(pcmsg.fields.size())) {
            RCLCPP_WARN(this->get_logger(), "Field index overflow! Expected %d fields", field_idx);
            return;
        }
        pcmsg.fields[field_idx].name = name;
        pcmsg.fields[field_idx].offset = offset;
        pcmsg.fields[field_idx].datatype = datatype;
        pcmsg.fields[field_idx].count = count;
        field_idx++;
    };

    // Pre-allocate fields to prevent reallocation during loop (though lambda handles it now)
    pcmsg.fields.resize(4);
    // RCLCPP_INFO(this->get_logger(), "Setting up point cloud fields...");

    add_field("x", 0, sensor_msgs::msg::PointField::FLOAT32, 1);
    add_field("y", 4, sensor_msgs::msg::PointField::FLOAT32, 1);
    add_field("z", 8, sensor_msgs::msg::PointField::FLOAT32, 1);
    add_field("rgb", 12, sensor_msgs::msg::PointField::UINT32, 1);

    // RCLCPP_INFO(this->get_logger(), "Starting point cloud generation loop.");

    for (size_t j = 0; j < pcmsg.height; j++) {
      for (size_t i = 0; i < pcmsg.width; i++) {
        float cx = (((float)i) - u0) / fox;
        float cy = (((float)j) - v0) / foy;

        // Safety check: ensure depth index is within payload bounds of the frame struct
        size_t depth_idx = j * pcmsg.width + i;
        if (depth_idx >= f->frame_head.frame_data_len - FRAME_HEAD_DATA_SIZE) {
            RCLCPP_WARN(this->get_logger(), "Depth index %zu out of bounds for payload length. Skipping.", depth_idx);
            break; // Or continue, depending on desired behavior for corrupted frames
        }

        float dst = ((float)depth[depth_idx]) / 1000.0f;

        float x = dst * cx;
        float y = dst * cy;
        float z = dst;

        *((float *)(ptr + 0)) = x;
        *((float *)(ptr + 4)) = y;
        *((float *)(ptr + 8)) = z;

        // Color lookup with bounds check for LUT index (though depth is uint8_t)
        const uint8_t *color = color_lut_jet[depth[depth_idx]];
        uint32_t color_r = color[0];
        uint32_t color_g = color[1];
        uint32_t color_b = color[2];

        *((uint32_t *)(ptr + 12)) = (color_r << 16) | (color_g << 8) | (color_b << 0);

        ptr += pcmsg.point_step;
      }
	  publisher_pointcloud->publish(pcmsg);
    }

    // RCLCPP_INFO(this->get_logger(), "Point cloud generation complete. Publishing.");

    // Save point cloud to PLY file
    std::string record_dir = "/record/";
    auto now = std::chrono::system_clock::now();
    auto ms_since_epoch =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            now.time_since_epoch()).count();

    std::stringstream filename;
    filename << record_dir << "pointcloud_" << ms_since_epoch << ".ply";

    savePointCloudAsPLY(pcmsg, filename.str());

    double frame_time = this->get_clock()->now().nanoseconds();
    add_frame_time(frame_time);

    RCLCPP_INFO(this->get_logger(), "Saved pointcloud to %s", filename.str().c_str());

    // Clean up allocated memory
    free(f);
  }


  void savePointCloudAsPLY(const sensor_msgs::msg::PointCloud2& pcmsg, const std::string& filename) {
      std::ofstream out(filename, std::ios::out | std::ios::binary);
      if (!out.is_open()) {
          RCLCPP_ERROR(this->get_logger(), "Failed to open %s", filename.c_str());
          return;
      }

      out << "ply\n";
      out << "format binary_little_endian 1.0\n";
      out << "element vertex " << pcmsg.width * pcmsg.height << "\n";
      out << "property float x\n";
      out << "property float y\n";
      out << "property float z\n";
      out << "property uchar red\n";
      out << "property uchar green\n";
      out << "property uchar blue\n";
      out << "end_header\n";

      const uint8_t* ptr = pcmsg.data.data();
      for (size_t j = 0; j < pcmsg.height; j++) {
          for (size_t i = 0; i < pcmsg.width; i++) {
              float x = *((float*)(ptr + 0));
              float y = *((float*)(ptr + 4));
              float z = *((float*)(ptr + 8));

              uint32_t rgb = *((uint32_t*)(ptr + 12));
              uint8_t r = (rgb >> 16) & 0xFF;
              uint8_t g = (rgb >> 8) & 0xFF;
              uint8_t b = rgb & 0xFF;

              out.write(reinterpret_cast<const char*>(&x), sizeof(float));
              out.write(reinterpret_cast<const char*>(&y), sizeof(float));
              out.write(reinterpret_cast<const char*>(&z), sizeof(float));
              out.write(reinterpret_cast<const char*>(&r), sizeof(uint8_t));
              out.write(reinterpret_cast<const char*>(&g), sizeof(uint8_t));
              out.write(reinterpret_cast<const char*>(&b), sizeof(uint8_t));

              ptr += pcmsg.point_step;
          }
      }
      out.close();
  }

    void check_fps() {
    auto current_time = this->get_clock()->now().nanoseconds();

    if (current_time - last_fps_print_time < 10e8) { // Only print every 1 second
      return;
    }

    last_fps_print_time = current_time;

    RCLCPP_INFO(this->get_logger(),
                "FPS Tracking: Publishing %.2f FPS (based on last %d frames)",
                calculate_average_fps(), max_frames_to_track);
  }

  double calculate_average_fps() {
    if (recent_frame_times.size() < 2) {
      return 0.0;
    }

    // Get the time difference between first and most recent frame in window
    auto first_time = recent_frame_times.front();
    auto last_time = recent_frame_times.back();

    double time_diff_seconds = (last_time - first_time) / 1e9;

    if (time_diff_seconds <= 0.0) {
      return 0.0;
    }

    // FPS = number of intervals / total time
    int interval_count = static_cast<int>(recent_frame_times.size()) - 1;
    double fps = interval_count / time_diff_seconds;

    RCLCPP_INFO(this->get_logger(),
                "Frame times: %zu frames, Time diff: %.3f s",
                recent_frame_times.size(), time_diff_seconds);

    return fps;
  }

  void add_frame_time(double timestamp) {
    // Add current frame time to deque
    recent_frame_times.push_back(timestamp);

    // Remove oldest if exceeding max count
    while (recent_frame_times.size() > static_cast<size_t>(max_frames_to_track)) {
      recent_frame_times.pop_front();
    }
  }

  const uint8_t color_lut_jet[256][3] =
  {
      {128, 0, 0},     {132, 0, 0},     {136, 0, 0},     {140, 0, 0},
      {144, 0, 0},     {148, 0, 0},     {152, 0, 0},     {156, 0, 0},
      {160, 0, 0},     {164, 0, 0},     {168, 0, 0},     {172, 0, 0},
      {176, 0, 0},     {180, 0, 0},     {184, 0, 0},     {188, 0, 0},
      {192, 0, 0},     {196, 0, 0},     {200, 0, 0},     {204, 0, 0},
      {208, 0, 0},     {212, 0, 0},     {216, 0, 0},     {220, 0, 0},
      {224, 0, 0},     {228, 0, 0},     {232, 0, 0},     {236, 0, 0},
      {240, 0, 0},     {244, 0, 0},     {248, 0, 0},     {252, 0, 0},
      {255, 0, 0},     {255, 4, 0},     {255, 8, 0},     {255, 12, 0},
      {255, 16, 0},    {255, 20, 0},    {255, 24, 0},    {255, 28, 0},
      {255, 32, 0},    {255, 36, 0},    {255, 40, 0},    {255, 44, 0},
      {255, 48, 0},    {255, 52, 0},    {255, 56, 0},    {255, 60, 0},
      {255, 64, 0},    {255, 68, 0},    {255, 72, 0},    {255, 76, 0},
      {255, 80, 0},    {255, 84, 0},    {255, 88, 0},    {255, 92, 0},
      {255, 96, 0},    {255, 100, 0},   {255, 104, 0},   {255, 108, 0},
      {255, 112, 0},   {255, 116, 0},   {255, 120, 0},   {255, 124, 0},
      {255, 128, 0},   {255, 132, 0},   {255, 136, 0},   {255, 140, 0},
      {255, 144, 0},   {255, 148, 0},   {255, 152, 0},   {255, 156, 0},
      {255, 160, 0},   {255, 164, 0},   {255, 168, 0},   {255, 172, 0},
      {255, 176, 0},   {255, 180, 0},   {255, 184, 0},   {255, 188, 0},
      {255, 192, 0},   {255, 196, 0},   {255, 200, 0},   {255, 204, 0},
      {255, 208, 0},   {255, 212, 0},   {255, 216, 0},   {255, 220, 0},
      {255, 224, 0},   {255, 228, 0},   {255, 232, 0},   {255, 236, 0},
      {255, 240, 0},   {255, 244, 0},   {255, 248, 0},   {255, 252, 0},
      {254, 255, 1},   {250, 255, 6},   {246, 255, 10},  {242, 255, 14},
      {238, 255, 18},  {234, 255, 22},  {230, 255, 26},  {226, 255, 30},
      {222, 255, 34},  {218, 255, 38},  {214, 255, 42},  {210, 255, 46},
      {206, 255, 50},  {202, 255, 54},  {198, 255, 58},  {194, 255, 62},
      {190, 255, 66},  {186, 255, 70},  {182, 255, 74},  {178, 255, 78},
      {174, 255, 82},  {170, 255, 86},  {166, 255, 90},  {162, 255, 94},
      {158, 255, 98},  {154, 255, 102}, {150, 255, 106}, {146, 255, 110},
      {142, 255, 114}, {138, 255, 118}, {134, 255, 122}, {130, 255, 126},
      {126, 255, 130}, {122, 255, 134}, {118, 255, 138}, {114, 255, 142},
      {110, 255, 146}, {106, 255, 150}, {102, 255, 154}, {98, 255, 158},
      {94, 255, 162},  {90, 255, 166},  {86, 255, 170},  {82, 255, 174},
      {78, 255, 178},  {74, 255, 182},  {70, 255, 186},  {66, 255, 190},
      {62, 255, 194},  {58, 255, 198},  {54, 255, 202},  {50, 255, 206},
      {46, 255, 210},  {42, 255, 214},  {38, 255, 218},  {34, 255, 222},
      {30, 255, 226},  {26, 255, 230},  {22, 255, 234},  {18, 255, 238},
      {14, 255, 242},  {10, 255, 246},  {6, 255, 250},   {2, 255, 254},
      {0, 252, 255},   {0, 248, 255},   {0, 244, 255},   {0, 240, 255},
      {0, 236, 255},   {0, 232, 255},   {0, 228, 255},   {0, 224, 255},
      {0, 220, 255},   {0, 216, 255},   {0, 212, 255},   {0, 208, 255},
      {0, 204, 255},   {0, 200, 255},   {0, 196, 255},   {0, 192, 255},
      {0, 188, 255},   {0, 184, 255},   {0, 180, 255},   {0, 176, 255},
      {0, 172, 255},   {0, 168, 255},   {0, 164, 255},   {0, 160, 255},
      {0, 156, 255},   {0, 152, 255},   {0, 148, 255},   {0, 144, 255},
      {0, 140, 255},   {0, 136, 255},   {0, 132, 255},   {0, 128, 255},
      {0, 124, 255},   {0, 120, 255},   {0, 116, 255},   {0, 112, 255},
      {0, 108, 255},   {0, 104, 255},   {0, 100, 255},   {0, 96, 255},
      {0, 92, 255},    {0, 88, 255},    {0, 84, 255},    {0, 80, 255},
      {0, 76, 255},    {0, 72, 255},    {0, 68, 255},    {0, 64, 255},
      {0, 60, 255},    {0, 56, 255},    {0, 52, 255},    {0, 48, 255},
      {0, 44, 255},    {0, 40, 255},    {0, 36, 255},    {0, 32, 255},
      {0, 28, 255},    {0, 24, 255},    {0, 20, 255},    {0, 16, 255},
      {0, 12, 255},    {0, 8, 255},     {0, 4, 255},     {0, 0, 255},
      {0, 0, 252},     {0, 0, 248},     {0, 0, 244},     {0, 0, 240},
      {0, 0, 236},     {0, 0, 232},     {0, 0, 228},     {0, 0, 224},
      {0, 0, 220},     {0, 0, 216},     {0, 0, 212},     {0, 0, 208},
      {0, 0, 204},     {0, 0, 200},     {0, 0, 196},     {0, 0, 192},
      {0, 0, 188},     {0, 0, 184},     {0, 0, 180},     {0, 0, 176},
      {0, 0, 172},     {0, 0, 168},     {0, 0, 164},     {0, 0, 160},
      {0, 0, 156},     {0, 0, 152},     {0, 0, 148},     {0, 0, 144},
      {0, 0, 140},     {0, 0, 136},     {0, 0, 132},     {0, 0, 128}
  };
};

int main(int argc, char const *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SipeedTOF_MSA010_Publisher>());
  rclcpp::shutdown();

  return 0;
}

