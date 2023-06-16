#ifndef KINOVA_VISION_HPP
#define KINOVA_VISION_HPP

#include <gst/gst.h>
#include <gst/app/gstappsink.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/srv/set_camera_info.hpp>
#include <image_transport/image_transport.hpp>
#include <camera_info_manager/camera_info_manager.hpp>

namespace CameraTypes
{
enum CameraType
{
  Unknown = 0,
  Color = 1,
  Depth = 2,
};
}

class Vision : public rclcpp::Node
{
public:
  explicit Vision(const rclcpp::NodeOptions &options);
  ~Vision();

  void run();
  void quit();

private:
  bool configure();
  bool initialize();
  bool start();
  bool loadCameraInfo();
  bool publish();
  void stop();
  bool changePipelineState(GstState state);

private:
  // ROS elements
  camera_info_manager::CameraInfoManager camera_info_manager_;
  image_transport::CameraPublisher camera_publisher_;

  // Gstreamer elements
  GstElement* gst_pipeline_;
  GstElement* gst_sink_;

  // General Gstreamer configuration
  std::string camera_config_;
  std::string camera_name_;
  std::string camera_info_;
  std::string frame_id_;
  std::string image_encoding_;
  std::string base_frame_id_;

  bool is_started_;
  bool stop_requested_;
  bool quit_requested_;
  int retry_count_;
  int camera_type_;
  double time_offset_;
  int image_width_;
  int image_height_;
  int pixel_size_;
  bool use_gst_timestamps_;
};

#endif
