#include "vision.h"

#include <rclcpp/rclcpp.hpp>
#include <csignal>

std::shared_ptr<Vision> g_vision;

void sigintHandler(int signal)
{
  rclcpp::shutdown();
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  // Override the default ROS sigint handler.
  std::signal(SIGINT, sigintHandler);

  auto options = rclcpp::NodeOptions().use_intra_process_comms(true);
  g_vision = std::make_shared<Vision>(options);
  g_vision->run();

  return 0;
}
