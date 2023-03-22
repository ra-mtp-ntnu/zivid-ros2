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

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/distortion_models.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <image_transport/image_transport.hpp>

#include <zivid_interfaces/srv/camera_info_model_name.hpp>
#include <zivid_interfaces/srv/camera_info_serial_number.hpp>
#include <zivid_interfaces/srv/capture.hpp>
#include <zivid_interfaces/srv/capture2_d.hpp>
#include <zivid_interfaces/srv/capture_assistant_suggest_settings.hpp>
#include <zivid_interfaces/srv/is_connected.hpp>
#include <zivid_interfaces/srv/load_settings_from_file.hpp>
#include <zivid_interfaces/srv/load_settings2_d_from_file.hpp>

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

class ZividCamera : public rclcpp::Node
{
public:
  ZIVID_CAMERA_PUBLIC
  explicit ZividCamera(const rclcpp::NodeOptions& options);

private:
  void publishFrame(Zivid::Frame&& frame);
  bool shouldPublishPointsXYZ() const;

  std_msgs::msg::Header makeHeader();
  void publishPointCloudXYZ(const std_msgs::msg::Header& header, const Zivid::PointCloud& point_cloud);
  void publishPointCloudXYZRGBA(const std_msgs::msg::Header& header, const Zivid::PointCloud& point_cloud);
  void publishColorImage(const std_msgs::msg::Header& header,
                         const sensor_msgs::msg::CameraInfo::ConstSharedPtr& camera_info,
                         const Zivid::PointCloud& point_cloud);
  void publishDepthImage(const std_msgs::msg::Header& header,
                         const sensor_msgs::msg::CameraInfo::ConstSharedPtr& camera_info,
                         const Zivid::PointCloud& point_cloud);

  CameraStatus camera_status_;

  rclcpp::Service<zivid_interfaces::srv::CameraInfoSerialNumber>::SharedPtr camera_info_serial_number_service_;
  rclcpp::Service<zivid_interfaces::srv::CameraInfoModelName>::SharedPtr camera_info_model_name_service_;
  rclcpp::Service<zivid_interfaces::srv::Capture>::SharedPtr capture_service_;
  rclcpp::Service<zivid_interfaces::srv::Capture2D>::SharedPtr capture_2d_service_;
  rclcpp::Service<zivid_interfaces::srv::LoadSettingsFromFile>::SharedPtr load_settings_from_file_service_;
  rclcpp::Service<zivid_interfaces::srv::LoadSettings2DFromFile>::SharedPtr load_settings_2d_from_file_service_;

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

  void loadSettingsFromFileServiceHandler(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<zivid_interfaces::srv::LoadSettingsFromFile::Request> request,
      std::shared_ptr<zivid_interfaces::srv::LoadSettingsFromFile::Response> response);

  void loadSettings2DFromFileServiceHandler(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<zivid_interfaces::srv::LoadSettings2DFromFile::Request> request,
      std::shared_ptr<zivid_interfaces::srv::LoadSettings2DFromFile::Response> response);

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_xyz_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_xyzrgba_publisher_;
  image_transport::CameraPublisher color_image_publisher_;
  image_transport::CameraPublisher depth_image_publisher_;
  image_transport::CameraPublisher snr_image_publisher_;

  bool use_latched_publisher_for_points_xyz_{ false };

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
