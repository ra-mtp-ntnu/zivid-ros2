// Copyright (c) 2020 Norwegian University of Science and Technology
// Copyright (c) 2019, Zivid AS
// Use of this source code is governed by the BSD 3-Clause license, see LICENSE

#include <chrono>
#include <memory>
#include <sstream>
#include <regex>

#include <zivid_camera/zivid_camera.h>

#include <Zivid/HDR.h>
#include <Zivid/Firmware.h>
#include <Zivid/Frame2D.h>
#include <Zivid/Settings2D.h>
#include <Zivid/Version.h>
#include <Zivid/CaptureAssistant.h>

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
ZividCamera::ZividCamera(const rclcpp::NodeOptions& options) : rclcpp_lifecycle::LifecycleNode("zivid_camera", options)
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  RCLCPP_INFO_STREAM(this->get_logger(), "Built towards Zivid API version " << ZIVID_VERSION);
  RCLCPP_INFO_STREAM(this->get_logger(), "Running with Zivid API version " << Zivid::Version::libraryVersion());
  if (Zivid::Version::libraryVersion() != ZIVID_VERSION)
  {
    throw std::runtime_error("Zivid library mismatch! The running Zivid Core version does not match the "
                             "version this ROS driver was built towards. Hint: Try to clean and re-build your project "
                             "from scratch.");
  }

  image_transport_node_ = rclcpp::Node::make_shared("image_transport_node");
  parameter_server_node_ = rclcpp::Node::make_shared("zivid_parameter_server");

  this->declare_parameter<std::string>("zivid.camera.serial_number", serial_number_);
  this->declare_parameter<int>("zivid.camera.num_capture_frames", num_capture_frames_);
  this->declare_parameter<std::string>("zivid.camera.frame_id", frame_id_);
  this->declare_parameter<std::string>("zivid.camera.file_camera_path", "");
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ZividCamera::on_configure(const rclcpp_lifecycle::State& state)
{


  serial_number_ = this->get_parameter("zivid.camera.serial_number").as_string();
  num_capture_frames_ = this->get_parameter("zivid.camera.num_capture_frames").as_int();
  frame_id_ = this->get_parameter("zivid.camera.frame_id").as_string();

  std::string file_camera_path = this->get_parameter("zivid.camera.file_camera_path").as_string();
  file_camera_mode_ = !file_camera_path.empty();

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
      throw std::runtime_error("No cameras found. Ensure that the camera is connected to the USB3 port on your PC.");
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
          if (c.serialNumber() == Zivid::SerialNumber(serial_number_))
            return c;
        }
        throw std::runtime_error("No camera found with serial number '" + serial_number_ + "'");
      }();
    }

    if (!Zivid::Firmware::isUpToDate(camera_))
    {
      RCLCPP_INFO(this->get_logger(), "The camera firmware is not up-to-date, starting update");
      Zivid::Firmware::update(camera_, [this](double progress, const std::string& state) {
        RCLCPP_INFO(this->get_logger(), "  [%.0f%%] %s", progress, state.c_str());
      });
      RCLCPP_INFO(this->get_logger(), "Firmware update completed");
    }
  }

  // Get current settings
  Zivid::Settings settings = camera_.settings();
  const std::string name{ this->get_name() };
  parameter_server_node_->declare_parameter(name + ".capture.general.blue_balance", settings.blueBalance().value());
  parameter_server_node_->declare_parameter(name + ".capture.general.red_balance", settings.redBalance().value());
  parameter_server_node_->declare_parameter(name + ".capture.general.filters.contrast."
                                                   "enabled",
                                            settings.filters().contrast().isEnabled().value());
  parameter_server_node_->declare_parameter(name + ".capture.general.filters.contrast."
                                                   "threshold",
                                            settings.filters().contrast().threshold().value());
  parameter_server_node_->declare_parameter(name + ".capture.general.filters.gaussian."
                                                   "enabled",
                                            settings.filters().gaussian().isEnabled().value());
  parameter_server_node_->declare_parameter(name + ".capture.general.filters.gaussian."
                                                   "sigma",
                                            settings.filters().gaussian().sigma().value());
  parameter_server_node_->declare_parameter(name + ".capture.general.filters.outlier."
                                                   "enabled",
                                            settings.filters().outlier().isEnabled().value());
  parameter_server_node_->declare_parameter(name + ".capture.general.filters.outlier."
                                                   "threshold",
                                            settings.filters().outlier().threshold().value());
  parameter_server_node_->declare_parameter(name + ".capture.general.filters.reflection."
                                                   "enabled",
                                            settings.filters().reflection().isEnabled().value());
  parameter_server_node_->declare_parameter(name + ".capture.general.filters.saturated."
                                                   "enabled",
                                            settings.filters().saturated().isEnabled().value());

  for (size_t i = 0; i < num_capture_frames_; ++i)
  {
    std::string name = this->get_name();
    parameter_server_node_->declare_parameter(name + ".capture.frame_" + std::to_string(i) + ".enabled", true);
    parameter_server_node_->declare_parameter(name + ".capture.frame_" + std::to_string(i) + ".bidirectional",
                                              settings.bidirectional().value());
    parameter_server_node_->declare_parameter(name + ".capture.frame_" + std::to_string(i) + ".brightness",
                                              settings.brightness().value());
    parameter_server_node_->declare_parameter(name + ".capture.frame_" + std::to_string(i) + ".exposure_time",
                                              static_cast<int>(settings.exposureTime().value().count()));
    parameter_server_node_->declare_parameter(name + ".capture.frame_" + std::to_string(i) + ".gain",
                                              settings.gain().value());
    parameter_server_node_->declare_parameter(name + ".capture.frame_" + std::to_string(i) + ".iris",
                                              static_cast<int>(settings.iris().value()));
    hdr_settings_.push_back(settings);
  }
  on_set_parameters_callback_handler_ =
      parameter_server_node_->add_on_set_parameters_callback(std::bind(&ZividCamera::parameterEventHandler, this, _1));

  camera_info_serial_number_service_ = this->create_service<zivid_interfaces::srv::CameraInfoSerialNumber>(
      "camera_info/serial_number", std::bind(&ZividCamera::cameraInfoSerialNumberServiceHandler, this, _1, _2, _3));

  camera_info_model_name_service_ = this->create_service<zivid_interfaces::srv::CameraInfoModelName>(
      "camera_info/model_name", std::bind(&ZividCamera::cameraInfoModelNameServiceHandler, this, _1, _2, _3));

  capture_service_ = create_service<zivid_interfaces::srv::Capture>(
      "capture", std::bind(&ZividCamera::captureServiceHandler, this, _1, _2, _3));

  capture_2d_service_ = create_service<zivid_interfaces::srv::Capture2D>(
      "capture_2d", std::bind(&ZividCamera::capture2DServiceHandler, this, _1, _2, _3));

  capture_assistant_suggest_settings_service_ = create_service<zivid_interfaces::srv::CaptureAssistantSuggestSettings>(
      "capture_assistant/suggest_settings",
      std::bind(&ZividCamera::captureAssistantSuggestSettingsServiceHandler, this, _1, _2, _3));

  color_image_publisher_ = image_transport::create_camera_publisher(image_transport_node_.get(), "color/"
                                                                                                 "image_color");
  depth_image_publisher_ = image_transport::create_camera_publisher(image_transport_node_.get(), "depth/image_raw");
  auto qos = rclcpp::SystemDefaultsQoS();
  points_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("points", qos);

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ZividCamera::on_activate(const rclcpp_lifecycle::State& state)
{
  RCLCPP_INFO_STREAM(this->get_logger(), camera_);
  if (!file_camera_mode_)
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "Connecting to camera '" << camera_.serialNumber() << "'");
    camera_.connect();
  }
  RCLCPP_INFO_STREAM(this->get_logger(), "Connected to camera '" << camera_.serialNumber() << "'");

  points_publisher_->on_activate();
  enabled_ = true;

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ZividCamera::on_deactivate(const rclcpp_lifecycle::State& state)
{
  if (!file_camera_mode_)
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "Disconnecting from camera '" << camera_.serialNumber() << "'");
    camera_.disconnect();
  }
  RCLCPP_INFO_STREAM(this->get_logger(), "Disconnected from camera '" << camera_.serialNumber() << "'");

  points_publisher_->on_deactivate();
  enabled_ = false;

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ZividCamera::on_cleanup(const rclcpp_lifecycle::State& state)
{
  image_transport_node_.reset();
  parameter_server_node_.reset();
  
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ZividCamera::on_shutdown(const rclcpp_lifecycle::State& state)
{
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
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
  const auto point_cloud = frame.getPointCloud();
  points_publisher_->publish(std::move(zivid_conversions::makePointCloud2(header, point_cloud)));
}

void ZividCamera::cameraInfoModelNameServiceHandler(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<zivid_interfaces::srv::CameraInfoModelName::Request> request,
    std::shared_ptr<zivid_interfaces::srv::CameraInfoModelName::Response> response)
{
  (void)request_header;
  if (!enabled_)
  {
    RCLCPP_WARN(this->get_logger(), "Trying to call the 'camera_model/model_name' service, but the service "
                                    "is not activated");
    return;
  }
  response->model_name = camera_.modelName();
}

void ZividCamera::cameraInfoSerialNumberServiceHandler(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<zivid_interfaces::srv::CameraInfoSerialNumber::Request> request,
    std::shared_ptr<zivid_interfaces::srv::CameraInfoSerialNumber::Response> response)
{
  (void)request_header;
  if (!enabled_)
  {
    RCLCPP_WARN(this->get_logger(), "Trying to call the 'camera_info/serial_number' service, but the service "
                                    "is not activated");
    return;
  }
  response->serial_number = camera_.serialNumber().toString();
}

void ZividCamera::captureServiceHandler(const std::shared_ptr<rmw_request_id_t> request_header,
                                        const std::shared_ptr<zivid_interfaces::srv::Capture::Request> request,
                                        std::shared_ptr<zivid_interfaces::srv::Capture::Response> response)
{
  (void)request_header;
  if (!enabled_)
  {
    RCLCPP_WARN(this->get_logger(), "Trying to call the 'capture' service, but the service is not activated");
    return;
  }
  std::lock_guard<std::mutex> parameter_lock_guard{ parameter_mutex_ };
  const auto settings = hdr_settings_;
  publishFrame(Zivid::HDR::capture(camera_, hdr_settings_));
}

void ZividCamera::capture2DServiceHandler(const std::shared_ptr<rmw_request_id_t> request_header,
                                          const std::shared_ptr<zivid_interfaces::srv::Capture2D::Request> request,
                                          std::shared_ptr<zivid_interfaces::srv::Capture2D::Response> response)
{
  (void)request_header;

  if (!enabled_)
  {
    RCLCPP_WARN(this->get_logger(), "Trying to call the 'capture_2d' service, but the service is not "
                                    "activated");
    return;
  }
  std::lock_guard<std::mutex> parameter_lock_guard{ parameter_mutex_ };

  Zivid::Settings2D settings2D;
  auto frame2D = camera_.capture2D(settings2D);
  const auto header = makeHeader();
  auto image = frame2D.image<Zivid::RGBA8>();
  const auto camera_info =
      zivid_conversions::makeCameraInfo(header, image.width(), image.height(), camera_.intrinsics());
  color_image_publisher_.publish(zivid_conversions::makeColorImage(header, image), camera_info);
}

void ZividCamera::captureAssistantSuggestSettingsServiceHandler(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<zivid_interfaces::srv::CaptureAssistantSuggestSettings::Request> request,
    std::shared_ptr<zivid_interfaces::srv::CaptureAssistantSuggestSettings::Response> response)
{
  (void)request_header;

  if (!enabled_)
  {
    RCLCPP_WARN(this->get_logger(), "Trying to call the 'capture_assistant/suggest_settings' service, but the service "
                                    "is not activated");
    return;
  }
}
template <rclcpp::ParameterType ParameterType, typename ZividSettingsType>
rcl_interfaces::msg::SetParametersResult ZividCamera::setParameter(const rclcpp::Parameter& parameter,
                                                                   std::vector<Zivid::Settings>& settings)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (auto& s : settings)
  {
    result = setParameter<ParameterType, ZividSettingsType>(parameter, s);
    if (!result.successful)
    {
      return result;
    }
  }
  return result;
}

template <rclcpp::ParameterType ParameterType, typename ZividSettingsType>
rcl_interfaces::msg::SetParametersResult ZividCamera::setParameter(const rclcpp::Parameter& parameter,
                                                                   Zivid::Settings& settings)
{
  using VT = typename ZividSettingsType::ValueType;
  rcl_interfaces::msg::SetParametersResult result;
  const std::string parameter_name = parameter.get_name();
  try
  {
    const auto value = parameter.get_value<ParameterType>();
    if constexpr (std::is_same_v<VT, std::chrono::microseconds>)
    {
      settings.set(ZividSettingsType{ std::chrono::microseconds(value) });
    }
    else if constexpr (std::is_same_v<VT, std::size_t>)
    {
      settings.set(ZividSettingsType{ static_cast<size_t>(value) });
    }
    else
    {
      settings.set(ZividSettingsType{ value });
    }
    result.successful = true;
    return result;
  }
  catch (const std::exception& e)
  {
    std::stringstream reason;
    reason << "The parameter '" << parameter_name << "' could not be set: " << e.what();
    RCLCPP_WARN_STREAM(parameter_server_node_->get_logger(), reason.str());
    result.successful = false;
    result.reason = reason.str();
    return result;
  }
}

rcl_interfaces::msg::SetParametersResult ZividCamera::parameterEventHandler(std::vector<rclcpp::Parameter> parameters)
{
  std::lock_guard<std::mutex> parameter_lock_guard{ parameter_mutex_ };

  rcl_interfaces::msg::SetParametersResult result;

  const std::string node_name{ this->get_name() };

  for (const auto& changed_parameter : parameters)
  {
    const std::string changed_parameter_name = changed_parameter.get_name();

    if (changed_parameter_name == node_name + ".capture.general.blue_balance")
    {
      return setParameter<rclcpp::ParameterType::PARAMETER_DOUBLE, Zivid::Settings::BlueBalance>(changed_parameter,
                                                                                                 hdr_settings_);
    }
    else if (changed_parameter_name == node_name + ".capture.general.red_balance")
    {
      return setParameter<rclcpp::ParameterType::PARAMETER_DOUBLE, Zivid::Settings::RedBalance>(changed_parameter,
                                                                                                hdr_settings_);
    }
    else if (changed_parameter_name == node_name + ".capture.general.filters.contrast.enabled")
    {
      return setParameter<rclcpp::ParameterType::PARAMETER_BOOL, Zivid::Settings::Filters::Contrast::Enabled>(
          changed_parameter, hdr_settings_);
    }
    else if (changed_parameter_name == node_name + ".capture.general.filters.contrast.threshold")
    {
      return setParameter<rclcpp::ParameterType::PARAMETER_DOUBLE, Zivid::Settings::Filters::Contrast::Threshold>(
          changed_parameter, hdr_settings_);
    }
    else if (changed_parameter_name == node_name + ".capture.general.filters.gaussian.enabled")
    {
      return setParameter<rclcpp::ParameterType::PARAMETER_BOOL, Zivid::Settings::Filters::Gaussian::Enabled>(
          changed_parameter, hdr_settings_);
    }
    else if (changed_parameter_name == node_name + ".capture.general.filters.gaussian.sigma")
    {
      return setParameter<rclcpp::ParameterType::PARAMETER_DOUBLE, Zivid::Settings::Filters::Gaussian::Sigma>(
          changed_parameter, hdr_settings_);
    }
    else if (changed_parameter_name == node_name + ".capture.general.filters.outlier.enabled")
    {
      return setParameter<rclcpp::ParameterType::PARAMETER_BOOL, Zivid::Settings::Filters::Outlier::Enabled>(
          changed_parameter, hdr_settings_);
    }
    else if (changed_parameter_name == node_name + ".capture.general.filters.outlier.threshold")
    {
      return setParameter<rclcpp::ParameterType::PARAMETER_DOUBLE, Zivid::Settings::Filters::Outlier::Threshold>(
          changed_parameter, hdr_settings_);
    }
    else if (changed_parameter_name == node_name + ".capture.general.filters.reflection.enabled")
    {
      return setParameter<rclcpp::ParameterType::PARAMETER_BOOL, Zivid::Settings::Filters::Reflection::Enabled>(
          changed_parameter, hdr_settings_);
    }
    else if (changed_parameter_name == node_name + ".capture.general.filters.saturated.enabled")
    {
      return setParameter<rclcpp::ParameterType::PARAMETER_BOOL, Zivid::Settings::Filters::Saturated::Enabled>(
          changed_parameter, hdr_settings_);
    }
    else
    {
      std::string prefix = node_name + ".capture.frame_";
      if (startsWith(changed_parameter_name, prefix))
      {
        std::regex frame_num_regex(".*([0-9]+).*");
        std::smatch match;
        if (std::regex_search(changed_parameter_name.begin(), changed_parameter_name.end(), match, frame_num_regex))
        {
          int frame_num = std::stoi(match[1]);
          if (endsWith(changed_parameter_name, "bidirectional"))
          {
            return setParameter<rclcpp::ParameterType::PARAMETER_BOOL, Zivid::Settings::Bidirectional>(
                changed_parameter, hdr_settings_[frame_num]);
          }
          else if (endsWith(changed_parameter_name, "brightness"))
          {
            return setParameter<rclcpp::PARAMETER_DOUBLE, Zivid::Settings::Brightness>(changed_parameter,
                                                                                       hdr_settings_[frame_num]);
          }
          else if (endsWith(changed_parameter_name, "exposure_time"))
          {
            return setParameter<rclcpp::PARAMETER_INTEGER, Zivid::Settings::ExposureTime>(changed_parameter,
                                                                                          hdr_settings_[frame_num]);
          }
          else if (endsWith(changed_parameter_name, "gain"))
          {
            return setParameter<rclcpp::PARAMETER_DOUBLE, Zivid::Settings::Gain>(changed_parameter,
                                                                                 hdr_settings_[frame_num]);
          }
          else if (endsWith(changed_parameter_name, "iris"))
          {
            return setParameter<rclcpp::PARAMETER_INTEGER, Zivid::Settings::Iris>(changed_parameter,
                                                                                  hdr_settings_[frame_num]);
          }
        }
      }
    }
  }
}

}  // namespace zivid_camera

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(zivid_camera::ZividCamera)