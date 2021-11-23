// Copyright (c) 2020 Norwegian University of Science and Technology
// Copyright (c) 2019, Zivid AS
// Use of this source code is governed by the BSD 3-Clause license, see LICENSE

#include <chrono>
#include <memory>
#include <sstream>
#include <regex>

#include <Zivid/Firmware.h>
#include <Zivid/Settings2D.h>
#include <Zivid/Version.h>
#include <Zivid/CaptureAssistant.h>
#include <Zivid/Experimental/Calibration.h>

#include <zivid_camera/zivid_camera.hpp>

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace
{
std::string toString(zivid_camera::CameraStatus camera_status)
{
  switch (camera_status)
  {
    case zivid_camera::CameraStatus::Connected:
      return "Connected";
    case zivid_camera::CameraStatus::Disconnected:
      return "Disconnected";
    case zivid_camera::CameraStatus::Idle:
      return "Idle";
  }
  return "N/A";
}

static bool endsWith(const std::string& str, const std::string& suffix)
{
  return str.size() >= suffix.size() && 0 == str.compare(str.size() - suffix.size(), suffix.size(), suffix);
}

static bool startsWith(const std::string& str, const std::string& prefix)
{
  return str.size() >= prefix.size() && 0 == str.compare(0, prefix.size(), prefix);
}

}  // namespace

namespace zivid_camera
{
ZividCamera::ZividCamera(const rclcpp::NodeOptions& options) : rclcpp::Node("zivid_camera", options)
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  RCLCPP_INFO_STREAM(this->get_logger(), "Built towards Zivid API version " << ZIVID_CORE_VERSION);
  RCLCPP_INFO_STREAM(this->get_logger(), "Running with Zivid API version " << Zivid::Version::coreLibraryVersion());
  if (Zivid::Version::coreLibraryVersion() != ZIVID_CORE_VERSION)
  {
    throw std::runtime_error("Zivid library mismatch! The running Zivid Core version does not match the "
                             "version this ROS driver was built towards. Hint: Try to clean and re-build your"
                             "project from scratch.");
  }

  this->declare_parameter<std::string>("serial_number", serial_number_);
  this->declare_parameter<std::string>("settings_path", settings_path_);
  this->declare_parameter<std::string>("settings2d_path", settings2d_path_);
  this->declare_parameter<std::string>("frame_id", frame_id_);
  this->declare_parameter<std::string>("file_camera_path", "");

  bool update_firmware_automatically = true;
  this->declare_parameter<bool>("update_firmware_automatically", update_firmware_automatically);

  serial_number_ = this->get_parameter("serial_number").as_string();
  frame_id_ = this->get_parameter("frame_id").as_string();
  settings_path_ = this->get_parameter("settings_path").as_string();
  settings2d_path_ = this->get_parameter("settings2d_path").as_string();

  std::string file_camera_path = this->get_parameter("file_camera_path").as_string();
  file_camera_mode_ = !file_camera_path.empty();

  update_firmware_automatically = this->get_parameter("update_firmware_automatically").as_bool();

  if (file_camera_mode_)
  {
    RCLCPP_INFO(this->get_logger(), "Creating file camera from file '%s'", file_camera_path.c_str());
    camera_ = zivid_.createFileCamera(file_camera_path);
  }
  else
  {
    auto cameras = zivid_.cameras();
    RCLCPP_INFO_STREAM(this->get_logger(), cameras.size() << " camera(s) found");
    if (cameras.empty())
    {
      throw std::runtime_error("No cameras found. Ensure that the camera is connected to your PC.");
    }
    else if (serial_number_.empty())
    {
      camera_ = [&]() {
        RCLCPP_INFO(this->get_logger(), "Selecting first available camera");
        for (auto& c : cameras)
        {
          if (c.state().isAvailable())
            return c;
        }
        throw std::runtime_error("No available cameras found. Is the camera in use by another process?");
      }();
    }
    else
    {
      if (serial_number_.find(":") == 0)
      {
        serial_number_ = serial_number_.substr(1);
      }
      camera_ = [&]() {
        RCLCPP_INFO(this->get_logger(), "Searching for camera with serial number '%s' ...", serial_number_.c_str());
        for (auto& c : cameras)
        {
          if (c.info().serialNumber() == Zivid::CameraInfo::SerialNumber(serial_number_))
            return c;
        }
        throw std::runtime_error("No camera found with serial number '" + serial_number_ + "'");
      }();
    }

    if (!Zivid::Firmware::isUpToDate(camera_))
    {
      if (update_firmware_automatically)
      {
        RCLCPP_INFO(this->get_logger(), "The camera firmware is not up-to-date, and update_firmware_automatically is "
                                        "true, starting update");
        Zivid::Firmware::update(camera_, [this](double progress, const std::string& state) {
          RCLCPP_INFO(this->get_logger(), "  [%.0f%%] %s", progress, state.c_str());
        });
        RCLCPP_INFO(this->get_logger(), "Firmware update completed");
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "The camera firmware is not up-to-date, and update_firmware_automatically is "
                                         "false. Throwing error.");
        throw std::runtime_error("The firmware on camera '" + camera_.info().serialNumber().value() +
                                 "' is not up to date. The launch parameter update_firmware_automatically "
                                 "is set to false. Please update the firmware on this camera manually.");
      }
    }
  }

  settings_ = Zivid::Settings{ settings_path_ };
  settings2d_ = Zivid::Settings2D{ settings2d_path_ };

  camera_info_serial_number_service_ = this->create_service<zivid_interfaces::srv::CameraInfoSerialNumber>(
      "camera_info/serial_number", std::bind(&ZividCamera::cameraInfoSerialNumberServiceHandler, this, _1, _2, _3));

  camera_info_model_name_service_ = this->create_service<zivid_interfaces::srv::CameraInfoModelName>(
      "camera_info/model_name", std::bind(&ZividCamera::cameraInfoModelNameServiceHandler, this, _1, _2, _3));

  capture_service_ = create_service<zivid_interfaces::srv::Capture>(
      "capture", std::bind(&ZividCamera::captureServiceHandler, this, _1, _2, _3));

  capture_2d_service_ = create_service<zivid_interfaces::srv::Capture2D>(
      "capture_2d", std::bind(&ZividCamera::capture2DServiceHandler, this, _1, _2, _3));

  load_settings_from_file_service_ = create_service<zivid_interfaces::srv::LoadSettingsFromFile>(
      "load_settings_from_file", std::bind(&ZividCamera::loadSettingsFromFileServiceHandler, this, _1, _2, _3));

  load_settings_2d_from_file_service_ = create_service<zivid_interfaces::srv::LoadSettings2DFromFile>(
      "load_settings_2d_from_file", std::bind(&ZividCamera::loadSettings2DFromFileServiceHandler, this, _1, _2, _3));

  auto qos = rclcpp::SystemDefaultsQoS();
  points_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("points", qos);

  RCLCPP_INFO_STREAM(this->get_logger(), camera_);
  if (!file_camera_mode_)
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "Connecting to camera '" << camera_.info().serialNumber() << "'");
    camera_.connect();
  }
  RCLCPP_INFO_STREAM(this->get_logger(), "Connected to camera '" << camera_.info().serialNumber() << "'");

  color_image_publisher_ = image_transport::create_camera_publisher(this, "color/image_color");
  depth_image_publisher_ = image_transport::create_camera_publisher(this, "depth/image");
  snr_image_publisher_ = image_transport::create_camera_publisher(this, "snr/image");

  RCLCPP_INFO(this->get_logger(), "Zivid camera driver is now ready!");
}

std_msgs::msg::Header ZividCamera::makeHeader()
{
  std_msgs::msg::Header header;
  header.stamp = get_clock()->now();
  header.frame_id = frame_id_;
  return header;
}

void ZividCamera::publishFrame(Zivid::Frame&& frame)
{
  const auto header = makeHeader();
  const auto point_cloud = frame.pointCloud();
  points_publisher_->publish(std::move(zivid_conversions::makePointCloud2(header, point_cloud)));
}

void ZividCamera::cameraInfoModelNameServiceHandler(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<zivid_interfaces::srv::CameraInfoModelName::Request> request,
    std::shared_ptr<zivid_interfaces::srv::CameraInfoModelName::Response> response)
{
  (void)request_header;
  (void)request;
  response->model_name = camera_.info().modelName().toString();
}

void ZividCamera::cameraInfoSerialNumberServiceHandler(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<zivid_interfaces::srv::CameraInfoSerialNumber::Request> request,
    std::shared_ptr<zivid_interfaces::srv::CameraInfoSerialNumber::Response> response)
{
  (void)request_header;
  (void)request;
  response->serial_number = camera_.info().serialNumber().toString();
}

void ZividCamera::captureServiceHandler(const std::shared_ptr<rmw_request_id_t> request_header,
                                        const std::shared_ptr<zivid_interfaces::srv::Capture::Request> request,
                                        std::shared_ptr<zivid_interfaces::srv::Capture::Response> response)
{
  (void)request_header;
  (void)request;
  (void)response;

  publishFrame(camera_.capture(settings_));
}

void ZividCamera::capture2DServiceHandler(const std::shared_ptr<rmw_request_id_t> request_header,
                                          const std::shared_ptr<zivid_interfaces::srv::Capture2D::Request> request,
                                          std::shared_ptr<zivid_interfaces::srv::Capture2D::Response> response)
{
  (void)request_header;
  (void)request;
  (void)response;

  auto frame = camera_.capture(settings2d_);
  const auto header = makeHeader();
  auto image = frame.imageRGBA();
  const auto camera_info = zivid_conversions::makeCameraInfo(header, image.width(), image.height(),
                                                             Zivid::Experimental::Calibration::intrinsics(camera_));
  color_image_publisher_.publish(zivid_conversions::makeColorImage(header, image), camera_info);
}

void ZividCamera::loadSettingsFromFileServiceHandler(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<zivid_interfaces::srv::LoadSettingsFromFile::Request> request,
    std::shared_ptr<zivid_interfaces::srv::LoadSettingsFromFile::Response> response)
{
  settings_ = Zivid::Settings{ request->file_path.c_str() };
}

void ZividCamera::loadSettings2DFromFileServiceHandler(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<zivid_interfaces::srv::LoadSettings2DFromFile::Request> request,
    std::shared_ptr<zivid_interfaces::srv::LoadSettings2DFromFile::Response> response)
{
  settings2d_ = Zivid::Settings2D{ request->file_path.c_str() };
}

}  // namespace zivid_camera

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(zivid_camera::ZividCamera)