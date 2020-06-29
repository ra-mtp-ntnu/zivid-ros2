// Copyright (c) 2020 Norwegian University of Science and Technology
// Copyright (c) 2019, Zivid AS
// Use of this source code is governed by the BSD 3-Clause license, see LICENSE

#pragma once

#include <mutex>

#include <Zivid/Application.h>
#include <Zivid/Camera.h>
#include <Zivid/Image.h>
#include <Zivid/Settings.h>
#include <Zivid/Settings2D.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/distortion_models.hpp>
#include <sensor_msgs/image_encodings.hpp>

// #include <image_transport/image_transport.h>

#include <zivid_interfaces/srv/camera_info_model_name.hpp>
#include <zivid_interfaces/srv/camera_info_serial_number.hpp>
#include <zivid_interfaces/srv/capture.hpp>
#include <zivid_interfaces/srv/capture2_d.hpp>
#include <zivid_interfaces/srv/capture_assistant_suggest_settings.hpp>
#include <zivid_interfaces/srv/is_connected.hpp>

#include <zivid_conversions/zivid_conversions.hpp>

#include <zivid_camera/visibility_control.h>

namespace Zivid
{
class Settings;
}

namespace zivid_camera
{
enum class CameraStatus
{
  Idle,
  Connected,
  Disconnected
};

struct ZividCameraOptions
{
  std::string node_name = "zivid_camera";
  rclcpp::QoS qos_profile = rclcpp::SystemDefaultsQoS();
};

ZIVID_CAMERA_PUBLIC
class ZividCamera : public rclcpp_lifecycle::LifecycleNode
{
public:
  ZIVID_CAMERA_PUBLIC
  explicit ZividCamera(const rclcpp::NodeOptions& options);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State& state);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State& state);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State& state);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State& state);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State& state);

  const rclcpp::Node::SharedPtr get_parameter_server_node()
  {
    return parameter_server_node_;
  }

  // const rclcpp::Node::SharedPtr get_image_transport_node()
  // {
  //   return image_transport_node_;
  // }

private:
  void publishFrame(Zivid::Frame&& frame);

  std_msgs::msg::Header makeHeader();

  CameraStatus camera_status_;

  rclcpp::Service<zivid_interfaces::srv::CameraInfoSerialNumber>::SharedPtr camera_info_serial_number_service_;
  rclcpp::Service<zivid_interfaces::srv::CameraInfoModelName>::SharedPtr camera_info_model_name_service_;
  rclcpp::Service<zivid_interfaces::srv::Capture>::SharedPtr capture_service_;
  rclcpp::Service<zivid_interfaces::srv::Capture2D>::SharedPtr capture_2d_service_;
  rclcpp::Service<zivid_interfaces::srv::CaptureAssistantSuggestSettings>::SharedPtr
      capture_assistant_suggest_settings_service_;
  rclcpp::Service<zivid_interfaces::srv::IsConnected>::SharedPtr is_connected_service_;

  void
  cameraInfoModelNameServiceHandler(const std::shared_ptr<rmw_request_id_t> request_header,
                                    const std::shared_ptr<zivid_interfaces::srv::CameraInfoModelName::Request> request,
                                    std::shared_ptr<zivid_interfaces::srv::CameraInfoModelName::Response> response);

  void cameraInfoSerialNumberServiceHandler(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<zivid_interfaces::srv::CameraInfoSerialNumber::Request> request,
      std::shared_ptr<zivid_interfaces::srv::CameraInfoSerialNumber::Response> response);

  void captureServiceHandler(const std::shared_ptr<rmw_request_id_t> request_header,
                             const std::shared_ptr<zivid_interfaces::srv::Capture::Request> request,
                             std::shared_ptr<zivid_interfaces::srv::Capture::Response> response);

  void capture2DServiceHandler(const std::shared_ptr<rmw_request_id_t> request_header,
                               const std::shared_ptr<zivid_interfaces::srv::Capture2D::Request> request,
                               std::shared_ptr<zivid_interfaces::srv::Capture2D::Response> response);

  void captureAssistantSuggestSettingsServiceHandler(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<zivid_interfaces::srv::CaptureAssistantSuggestSettings::Request> request,
      std::shared_ptr<zivid_interfaces::srv::CaptureAssistantSuggestSettings::Response> response);

  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_publisher_;
  // image_transport::CameraPublisher color_image_publisher_;
  // image_transport::CameraPublisher depth_image_publisher_;
  // rclcpp::Node::SharedPtr image_transport_node_;

  rclcpp::Node::SharedPtr parameter_server_node_;
  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_handler_;
  rcl_interfaces::msg::SetParametersResult parameterEventHandler(std::vector<rclcpp::Parameter> parameters);

  template <rclcpp::ParameterType ParameterType, typename ZividSettingsType, typename ZividSettingsOrAcquisition>
  rcl_interfaces::msg::SetParametersResult setParameter(const rclcpp::Parameter& parameter,
                                                        std::vector<ZividSettingsOrAcquisition>& settings);

  template <rclcpp::ParameterType ParameterType, typename ZividSettingsType, typename ZividSettingsOrAcquisition>
  rcl_interfaces::msg::SetParametersResult setParameter(const rclcpp::Parameter& parameter, ZividSettingsOrAcquisition& settings);
  std::mutex parameter_mutex_;

  Zivid::Application zivid_;
  Zivid::Camera camera_;

  Zivid::Settings settings_;
  Zivid::Settings2D settings2d_;

  std::string serial_number_;
  bool file_camera_mode_{ false };
  std::string frame_id_{ "zivid_optical_frame" };
  std::string settings_path_;
  std::string settings2d_path_;
  bool enabled_{ false };
};

}  // namespace zivid_camera