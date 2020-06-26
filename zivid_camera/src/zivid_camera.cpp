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

#include <zivid_camera/zivid_camera.h>

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

  RCLCPP_INFO_STREAM(this->get_logger(), "Built towards Zivid API version " << ZIVID_CORE_VERSION);
  RCLCPP_INFO_STREAM(this->get_logger(), "Running with Zivid API version " << Zivid::Version::coreLibraryVersion());
  if (Zivid::Version::coreLibraryVersion() != ZIVID_CORE_VERSION)
  {
    throw std::runtime_error("Zivid library mismatch! The running Zivid Core version does not match the "
                             "version this ROS driver was built towards. Hint: Try to clean and re-build your"
                             "project from scratch.");
  }

  image_transport_node_ = rclcpp::Node::make_shared("image_transport_node");
  parameter_server_node_ = rclcpp::Node::make_shared("zivid_parameter_server", rclcpp::NodeOptions().allow_undeclared_parameters(true));

  this->declare_parameter<std::string>("zivid.camera.serial_number", serial_number_);
  this->declare_parameter<std::string>("zivid.camera.settings_path", settings_path_);
  this->declare_parameter<std::string>("zivid.camera.settings2d_path", settings2d_path_);
  this->declare_parameter<std::string>("zivid.camera.frame_id", frame_id_);
  this->declare_parameter<std::string>("zivid.camera.file_camera_path", "");
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ZividCamera::on_configure(const rclcpp_lifecycle::State& state)
{
  serial_number_ = this->get_parameter("zivid.camera.serial_number").as_string();
  frame_id_ = this->get_parameter("zivid.camera.frame_id").as_string();
  settings_path_ = this->get_parameter("zivid.camera.settings_path").as_string();
  settings2d_path_ = this->get_parameter("zivid.camera.settings2d_path").as_string();

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
          if (c.info().serialNumber() == Zivid::CameraInfo::SerialNumber(serial_number_))
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

  // Get settings_ from file
  settings_ = Zivid::Settings{ settings_path_ };
  std::string settings_parameter_prefix{ "zivid.camera.settings" };
  parameter_server_node_->declare_parameter(settings_parameter_prefix + ".processing.color.balance.blue",
                                            settings_.processing().color().balance().blue().value());
  parameter_server_node_->declare_parameter(settings_parameter_prefix + ".processing.color.balance.green",
                                            settings_.processing().color().balance().red().value());
  parameter_server_node_->declare_parameter(settings_parameter_prefix + ".processing.color.balance.red",
                                            settings_.processing().color().balance().red().value());
  parameter_server_node_->declare_parameter(
      settings_parameter_prefix + ".processing.filters.experimental.contrast_distortion.correction.enabled",
      settings_.processing().filters().experimental().contrastDistortion().correction().isEnabled().value());
  parameter_server_node_->declare_parameter(
      settings_parameter_prefix + ".processing.filters.experimental.contrast_distortion.correction.strength",
      settings_.processing().filters().experimental().contrastDistortion().correction().strength().value());
  parameter_server_node_->declare_parameter(
      settings_parameter_prefix + ".processing.filters.experimental.contrast_distortion.removal.enabled",
      settings_.processing().filters().experimental().contrastDistortion().removal().isEnabled().value());
  parameter_server_node_->declare_parameter(
      settings_parameter_prefix + ".processing.filters.experimental.contrast_distortion.removal.threshold",
      settings_.processing().filters().experimental().contrastDistortion().removal().threshold().value());

  parameter_server_node_->declare_parameter(settings_parameter_prefix + ".processing.filters.noise.removal.enabled",
                                            settings_.processing().filters().noise().removal().isEnabled().value());
  parameter_server_node_->declare_parameter(settings_parameter_prefix + ".processing.filters.noise.removal.threshold",
                                            settings_.processing().filters().noise().removal().threshold().value());

  parameter_server_node_->declare_parameter(settings_parameter_prefix + ".processing.filters.outlier.removal.enabled",
                                            settings_.processing().filters().outlier().removal().isEnabled().value());
  parameter_server_node_->declare_parameter(settings_parameter_prefix + ".processing.filters.outlier.removal.threshold",
                                            settings_.processing().filters().outlier().removal().threshold().value());

  parameter_server_node_->declare_parameter(
      settings_parameter_prefix + ".processing.filters.reflection.removal."
                                  "enabled",
      settings_.processing().filters().reflection().removal().isEnabled().value());

  parameter_server_node_->declare_parameter(
      settings_parameter_prefix + ".processing.filters.smoothing.gaussian."
                                  "enabled",
      settings_.processing().filters().smoothing().gaussian().isEnabled().value());
  parameter_server_node_->declare_parameter(settings_parameter_prefix + ".processing.filters.smoothing.gaussian.sigma",
                                            settings_.processing().filters().smoothing().gaussian().sigma().value());

  const auto acquisitions = settings_.acquisitions();
  for (size_t i = 0; i < acquisitions.size(); ++i)
  {
    const auto acquisition = acquisitions[i];
    parameter_server_node_->declare_parameter(settings_parameter_prefix + ".acquisitions.acquisition_" +
                                                  std::to_string(i) + ".aperture",
                                              acquisition.aperture().value());
    parameter_server_node_->declare_parameter(settings_parameter_prefix + ".acquisitions.acquisition_" +
                                                  std::to_string(i) + ".brightness",
                                              acquisition.brightness().value());
    parameter_server_node_->declare_parameter(settings_parameter_prefix + ".acquisitions.acquisition_" +
                                                  std::to_string(i) + ".exposure_time",
                                              static_cast<int>(acquisition.exposureTime().value().count()));
    parameter_server_node_->declare_parameter(settings_parameter_prefix + ".acquisitions.acquisition_" +
                                                  std::to_string(i) + ".gain",
                                              acquisition.gain().value());
    parameter_server_node_->declare_parameter(settings_parameter_prefix + ".acquisitions.acquisition_" +
                                                  std::to_string(i) + ".patterns.sine.bidirectional",
                                              acquisition.patterns().sine().bidirectional().value());
  }

  settings2d_ = Zivid::Settings2D{ settings2d_path_ };

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
  //  depth_image_publisher_ = image_transport::create_camera_publisher(image_transport_node_.get(), "depth/image_raw");
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
    RCLCPP_INFO_STREAM(this->get_logger(), "Connecting to camera '" << camera_.info().serialNumber() << "'");
    camera_.connect();
  }
  RCLCPP_INFO_STREAM(this->get_logger(), "Connected to camera '" << camera_.info().serialNumber() << "'");

  points_publisher_->on_activate();
  enabled_ = true;

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ZividCamera::on_deactivate(const rclcpp_lifecycle::State& state)
{
  if (!file_camera_mode_)
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "Disconnecting from camera '" << camera_.info().serialNumber() << "'");
    camera_.disconnect();
  }
  RCLCPP_INFO_STREAM(this->get_logger(), "Disconnected from camera '" << camera_.info().serialNumber() << "'");

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
  const auto point_cloud = frame.pointCloud();
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
  response->model_name = camera_.info().modelName().toString();
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
  response->serial_number = camera_.info().serialNumber().toString();
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
  publishFrame(camera_.capture(settings_));
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
  auto frame = camera_.capture(settings2d_);
  const auto header = makeHeader();
  auto image = frame.imageRGBA();
  const auto camera_info = zivid_conversions::makeCameraInfo(header, image.width(), image.height(),
                                                             Zivid::Experimental::Calibration::intrinsics(camera_));
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

  const auto max_capture_time =
      std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::seconds(request->max_capture_time.sec)) +
      std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::nanoseconds(request->max_capture_time.nanosec));

  const auto ambient_light_frequency = [&request]() {
    switch (request->ambient_light_frequency)
    {
      case zivid_interfaces::srv::CaptureAssistantSuggestSettings::Request::AMBIENT_LIGHT_FREQUENCY_NONE:
        return Zivid::CaptureAssistant::SuggestSettingsParameters::AmbientLightFrequency::none;
      case zivid_interfaces::srv::CaptureAssistantSuggestSettings::Request::AMBIENT_LIGHT_FREQUENCY_50HZ:
        return Zivid::CaptureAssistant::SuggestSettingsParameters::AmbientLightFrequency::hz50;
      case zivid_interfaces::srv::CaptureAssistantSuggestSettings::Request::AMBIENT_LIGHT_FREQUENCY_60HZ:
        return Zivid::CaptureAssistant::SuggestSettingsParameters::AmbientLightFrequency::hz60;
    }
    throw std::runtime_error("Unhandled AMBIENT_LIGHT_FREQUENCY value: " +
                             std::to_string(request->ambient_light_frequency));
  }();

  const auto suggestSettingsParameters = Zivid::CaptureAssistant::SuggestSettingsParameters{
    ambient_light_frequency, Zivid::CaptureAssistant::SuggestSettingsParameters::MaxCaptureTime{ max_capture_time }
  };

  settings_ = Zivid::CaptureAssistant::suggestSettings(camera_, suggestSettingsParameters);

  std::string settings_parameter_prefix{ "zivid.camera.settings" };
  parameter_server_node_->set_parameter(rclcpp::Parameter(settings_parameter_prefix + ".processing.color.balance.blue",
                                                          settings_.processing().color().balance().blue().value()));
  parameter_server_node_->set_parameter(rclcpp::Parameter(settings_parameter_prefix + ".processing.color.balance.green",
                                                          settings_.processing().color().balance().red().value()));
  parameter_server_node_->set_parameter(rclcpp::Parameter(settings_parameter_prefix + ".processing.color.balance.red",
                                                          settings_.processing().color().balance().red().value()));
  parameter_server_node_->set_parameter(rclcpp::Parameter(
      settings_parameter_prefix + ".processing.filters.experimental.contrast_distortion.correction.enabled",
      settings_.processing().filters().experimental().contrastDistortion().correction().isEnabled().value()));
  parameter_server_node_->set_parameter(rclcpp::Parameter(
      settings_parameter_prefix + ".processing.filters.experimental.contrast_distortion.correction.strength",
      settings_.processing().filters().experimental().contrastDistortion().correction().strength().value()));
  parameter_server_node_->set_parameter(rclcpp::Parameter(
      settings_parameter_prefix + ".processing.filters.experimental.contrast_distortion.removal.enabled",
      settings_.processing().filters().experimental().contrastDistortion().removal().isEnabled().value()));
  parameter_server_node_->set_parameter(rclcpp::Parameter(
      settings_parameter_prefix + ".processing.filters.experimental.contrast_distortion.removal.threshold",
      settings_.processing().filters().experimental().contrastDistortion().removal().threshold().value()));

  parameter_server_node_->set_parameter(
      rclcpp::Parameter(settings_parameter_prefix + ".processing.filters.noise.removal.enabled",
                        settings_.processing().filters().noise().removal().isEnabled().value()));
  parameter_server_node_->set_parameter(
      rclcpp::Parameter(settings_parameter_prefix + ".processing.filters.noise.removal.threshold",
                        settings_.processing().filters().noise().removal().threshold().value()));

  parameter_server_node_->set_parameter(
      rclcpp::Parameter(settings_parameter_prefix + ".processing.filters.outlier.removal.enabled",
                        settings_.processing().filters().outlier().removal().isEnabled().value()));
  parameter_server_node_->set_parameter(
      rclcpp::Parameter(settings_parameter_prefix + ".processing.filters.outlier.removal.threshold",
                        settings_.processing().filters().outlier().removal().threshold().value()));

  parameter_server_node_->set_parameter(
      rclcpp::Parameter(settings_parameter_prefix + ".processing.filters.reflection.removal."
                                                    "enabled",
                        settings_.processing().filters().reflection().removal().isEnabled().value()));

  parameter_server_node_->set_parameter(
      rclcpp::Parameter(settings_parameter_prefix + ".processing.filters.smoothing.gaussian."
                                                    "enabled",
                        settings_.processing().filters().smoothing().gaussian().isEnabled().value()));
  parameter_server_node_->set_parameter(
      rclcpp::Parameter(settings_parameter_prefix + ".processing.filters.smoothing.gaussian.sigma",
                        settings_.processing().filters().smoothing().gaussian().sigma().value()));

  const auto acquisitions = settings_.acquisitions();
  for (size_t i = 0; i < acquisitions.size(); ++i)
  {
    const auto acquisition = acquisitions[i];
    parameter_server_node_->set_parameter(
        rclcpp::Parameter(settings_parameter_prefix + ".acquisitions.acquisition_" + std::to_string(i) + ".aperture",
                          acquisition.aperture().value()));
    parameter_server_node_->set_parameter(
        rclcpp::Parameter(settings_parameter_prefix + ".acquisitions.acquisition_" + std::to_string(i) + ".brightness",
                          acquisition.brightness().value()));
    parameter_server_node_->set_parameter(rclcpp::Parameter(
        settings_parameter_prefix + ".acquisitions.acquisition_" + std::to_string(i) + ".exposure_time",
        static_cast<int>(acquisition.exposureTime().value().count())));
    parameter_server_node_->set_parameter(
        rclcpp::Parameter(settings_parameter_prefix + ".acquisitions.acquisition_" + std::to_string(i) + ".gain",
                          acquisition.gain().value()));
    parameter_server_node_->set_parameter(rclcpp::Parameter(settings_parameter_prefix + ".acquisitions.acquisition_" +
                                                                std::to_string(i) + ".patterns.sine.bidirectional",
                                                            acquisition.patterns().sine().bidirectional().value()));
  }
}

template <rclcpp::ParameterType ParameterType, typename ZividSettingsType, typename ZividSettingsOrAcquisition>
rcl_interfaces::msg::SetParametersResult ZividCamera::setParameter(const rclcpp::Parameter& parameter,
                                                                   std::vector<ZividSettingsOrAcquisition>& settings)
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

template <rclcpp::ParameterType ParameterType, typename ZividSettingsType, typename ZividSettingsOrAcquisition>
rcl_interfaces::msg::SetParametersResult ZividCamera::setParameter(const rclcpp::Parameter& parameter,
                                                                   ZividSettingsOrAcquisition& settings)
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
    std::string settings_parameter_prefix{ "zivid.camera.settings" };
    const std::string& changed_parameter_name = changed_parameter.get_name();
    if (changed_parameter_name == settings_parameter_prefix + ".processing.color.balance.blue")
    {
      return setParameter<rclcpp::ParameterType::PARAMETER_DOUBLE, Zivid::Settings::Processing::Color::Balance::Blue>(
          changed_parameter, settings_);
    }
    else if (changed_parameter_name == settings_parameter_prefix + ".processing.color.balance.red")
    {
      return setParameter<rclcpp::ParameterType::PARAMETER_DOUBLE, Zivid::Settings::Processing::Color::Balance::Red>(
          changed_parameter, settings_);
    }
    else if (changed_parameter_name == settings_parameter_prefix + ".processing.color.balance.green")
    {
      return setParameter<rclcpp::ParameterType::PARAMETER_DOUBLE, Zivid::Settings::Processing::Color::Balance::Green>(
          changed_parameter, settings_);
    }
    else if (changed_parameter_name == settings_parameter_prefix + ".processing.filters.experimental.contrast_"
                                                                   "distortion.correction."
                                                                   "enabled")
    {
      return setParameter<rclcpp::ParameterType::PARAMETER_BOOL,
                          Zivid::Settings::Processing::Filters::Experimental::ContrastDistortion::Correction::Enabled>(
          changed_parameter, settings_);
    }
    else if (changed_parameter_name == settings_parameter_prefix + ".processing.filters.experimental.contrast_"
                                                                   "distortion.correction."
                                                                   "strength")
    {
      return setParameter<rclcpp::ParameterType::PARAMETER_DOUBLE,
                          Zivid::Settings::Processing::Filters::Experimental::ContrastDistortion::Correction::Strength>(
          changed_parameter, settings_);
    }
    else if (changed_parameter_name == settings_parameter_prefix + ".processing.filters.experimental.contrast_"
                                                                   "distortion.removal."
                                                                   "enabled")
    {
      return setParameter<rclcpp::ParameterType::PARAMETER_BOOL,
                          Zivid::Settings::Processing::Filters::Experimental::ContrastDistortion::Removal::Enabled>(
          changed_parameter, settings_);
    }
    else if (changed_parameter_name == settings_parameter_prefix + ".processing.filters.experimental.contrast_"
                                                                   "distortion.removal."
                                                                   "threshold")
    {
      return setParameter<rclcpp::ParameterType::PARAMETER_DOUBLE,
                          Zivid::Settings::Processing::Filters::Experimental::ContrastDistortion::Removal::Threshold>(
          changed_parameter, settings_);
    }
    else if (changed_parameter_name == settings_parameter_prefix + ".processing.filters.noise.removal.enabled")
    {
      return setParameter<rclcpp::ParameterType::PARAMETER_BOOL,
                          Zivid::Settings::Processing::Filters::Noise::Removal::Enabled>(changed_parameter, settings_);
    }
    else if (changed_parameter_name == settings_parameter_prefix + ".processing.filters.noise.removal.threshold")
    {
      return setParameter<rclcpp::ParameterType::PARAMETER_DOUBLE,
                          Zivid::Settings::Processing::Filters::Noise::Removal::Threshold>(changed_parameter,
                                                                                           settings_);
    }
    else if (changed_parameter_name == settings_parameter_prefix + ".processing.filters.outlier.removal.enabled")
    {
      return setParameter<rclcpp::ParameterType::PARAMETER_BOOL,
                          Zivid::Settings::Processing::Filters::Outlier::Removal::Enabled>(changed_parameter,
                                                                                           settings_);
    }
    else if (changed_parameter_name == settings_parameter_prefix + ".processing.filters.outlier.removal.threshold")
    {
      return setParameter<rclcpp::ParameterType::PARAMETER_DOUBLE,
                          Zivid::Settings::Processing::Filters::Outlier::Removal::Threshold>(changed_parameter,
                                                                                             settings_);
    }
    else if (changed_parameter_name == settings_parameter_prefix + ".processing.filters.reflection.removal.enabled")
    {
      return setParameter<rclcpp::ParameterType::PARAMETER_BOOL,
                          Zivid::Settings::Processing::Filters::Reflection::Removal::Enabled>(changed_parameter,
                                                                                              settings_);
    }
    else if (changed_parameter_name == settings_parameter_prefix + ".processing.filters.smoothing.gaussian.enabled")
    {
      return setParameter<rclcpp::ParameterType::PARAMETER_BOOL,
                          Zivid::Settings::Processing::Filters::Smoothing::Gaussian::Enabled>(changed_parameter,
                                                                                              settings_);
    }
    else if (changed_parameter_name == settings_parameter_prefix + ".processing.filters.smoothing.gaussian.sigma")
    {
      return setParameter<rclcpp::ParameterType::PARAMETER_DOUBLE,
                          Zivid::Settings::Processing::Filters::Smoothing::Gaussian::Sigma>(changed_parameter,
                                                                                            settings_);
    }
    else
    {
      const std::string prefix = settings_parameter_prefix + ".acquisitions.acquisition_";
      if (startsWith(changed_parameter_name, prefix))
      {
        std::regex frame_num_regex(".*([0-9]+).*");
        std::smatch match;
        if (std::regex_search(changed_parameter_name.begin(), changed_parameter_name.end(), match, frame_num_regex))
        {
          int frame_num = std::stoi(match[1]);

          if (endsWith(changed_parameter_name, "aperture"))
          {
            return setParameter<rclcpp::ParameterType::PARAMETER_DOUBLE, Zivid::Settings::Acquisition::Aperture>(
                changed_parameter, settings_.acquisitions()[frame_num]);
          }
          else if (endsWith(changed_parameter_name, "brightness"))
          {
            return setParameter<rclcpp::ParameterType::PARAMETER_DOUBLE, Zivid::Settings::Acquisition::Brightness>(
                changed_parameter, settings_.acquisitions()[frame_num]);
          }
          else if (endsWith(changed_parameter_name, "exposure_time"))
          {
            return setParameter<rclcpp::ParameterType::PARAMETER_INTEGER, Zivid::Settings::Acquisition::ExposureTime>(
                changed_parameter, settings_.acquisitions()[frame_num]);
          }
          else if (endsWith(changed_parameter_name, "gain"))
          {
            return setParameter<rclcpp::ParameterType::PARAMETER_DOUBLE, Zivid::Settings::Acquisition::Gain>(
                changed_parameter, settings_.acquisitions()[frame_num]);
          }
          else if (endsWith(changed_parameter_name, "patterns.sine.bidirectional"))
          {
            return setParameter<rclcpp::ParameterType::PARAMETER_BOOL,
                                Zivid::Settings::Acquisition::Patterns::Sine::Bidirectional>(
                changed_parameter, settings_.acquisitions()[frame_num]);
          }
        }
      }
    }
  }
}

}  // namespace zivid_camera

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(zivid_camera::ZividCamera)