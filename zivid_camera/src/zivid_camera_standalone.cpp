// Copyright (c) 2020 Norwegian University of Science and Technology
// Copyright (c) 2019, Zivid AS
// Use of this source code is governed by the BSD 3-Clause license, see LICENSE

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <zivid_camera/zivid_camera.hpp>

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::spin(std::make_shared<zivid_camera::ZividCamera>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}