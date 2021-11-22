// Copyright (c) 2020 Norwegian University of Science and Technology
// Copyright (c) 2019, Zivid AS
// Use of this source code is governed by the BSD 3-Clause license, see LICENSE

#include <rclcpp/rclcpp.hpp>
#include <zivid_camera/zivid_camera.h>

int main(int argc, char * argv[])
{
  // Force flush of the stdout buffer.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;

  auto zivid_camera = std::make_shared<zivid_camera::ZividCamera>(options);
  exec.add_node(zivid_camera->get_node_base_interface());
  // exec.add_node(zivid_camera->get_parameter_server_node());
  //exec.add_node(zivid_camera->get_image_transport_node());

  exec.spin();

  rclcpp::shutdown();

  return 0;
}