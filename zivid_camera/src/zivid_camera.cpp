// Copyright (c) 2020 Norwegian University of Science and Technology
// Copyright (c) 2019, Zivid AS
// Use of this source code is governed by the BSD 3-Clause license, see LICENSE

#include <chrono>
#include <memory>
#include <sstream>
#include <regex>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <Zivid/Firmware.h>
#include <Zivid/Settings2D.h>
#include <Zivid/Version.h>
#include <Zivid/CaptureAssistant.h>
#include <Zivid/Experimental/Calibration.h>

#include <zivid_camera/zivid_camera.hpp>
// #include <zivid_conversions/zivid_conversions.hpp>

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace
{

sensor_msgs::msg::PointField createPointField(std::string name, uint32_t offset, uint8_t datatype, uint32_t count)
{
  sensor_msgs::msg::PointField point_field;
  point_field.name = name;
  point_field.offset = offset;
  point_field.datatype = datatype;
  point_field.count = count;
  return point_field;
}

bool big_endian()
{
  return false;
}

template <class T>
void fillCommonMsgFields(T& msg, const std_msgs::msg::Header& header, std::size_t width, std::size_t height)
{
  msg.header = header;
  msg.height = static_cast<uint32_t>(height);
  msg.width = static_cast<uint32_t>(width);
  msg.is_bigendian = big_endian();
}

sensor_msgs::msg::Image::SharedPtr makeImage(const std_msgs::msg::Header& header, const std::string& encoding,
                                             std::size_t width, std::size_t height)
{
  auto image = std::make_shared<sensor_msgs::msg::Image>();
  fillCommonMsgFields(*image, header, width, height);
  image->encoding = encoding;
  const auto bytes_per_pixel = static_cast<std::size_t>(sensor_msgs::image_encodings::numChannels(encoding) *
                                                        sensor_msgs::image_encodings::bitDepth(encoding) / 8);
  image->step = static_cast<uint32_t>(bytes_per_pixel * width);
  return image;
}

sensor_msgs::msg::Image::ConstSharedPtr makeColorImage(const std_msgs::msg::Header& header,
                                                       const Zivid::Image<Zivid::ColorRGBA>& image)
{
  auto msg = std::make_shared<sensor_msgs::msg::Image>();
  fillCommonMsgFields(*msg, header, image.width(), image.height());
  msg->encoding = sensor_msgs::image_encodings::RGBA8;
  constexpr uint32_t bytes_per_pixel = 4U;
  msg->step = static_cast<uint32_t>(bytes_per_pixel * image.width());
  const auto uint8_data_ptr = reinterpret_cast<const uint8_t*>(image.data());
  msg->data = std::vector<uint8_t>(uint8_data_ptr, uint8_data_ptr + image.size() * sizeof(Zivid::ColorRGBA));
  return msg;
}

template <typename ZividDataType>
sensor_msgs::msg::Image::SharedPtr makePointCloudImage(const Zivid::PointCloud& point_cloud,
                                                       const std_msgs::msg::Header& header, const std::string& encoding)
{
  auto image = makeImage(header, encoding, point_cloud.width(), point_cloud.height());
  image->data.resize(image->step * image->height);
  point_cloud.copyData<ZividDataType>(reinterpret_cast<ZividDataType*>(image->data.data()));
  return image;
}

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

sensor_msgs::msg::CameraInfo::ConstSharedPtr makeCameraInfo(const std_msgs::msg::Header& header, std::size_t width,
                                                            std::size_t height,
                                                            const Zivid::CameraIntrinsics& intrinsics)
{
  auto msg = std::make_shared<sensor_msgs::msg::CameraInfo>();
  msg->header = header;
  msg->width = static_cast<uint32_t>(width);
  msg->height = static_cast<uint32_t>(height);
  msg->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

  // k1, k2, t1, t2, k3
  const auto distortion = intrinsics.distortion();
  msg->d.resize(5);
  msg->d[0] = distortion.k1().value();
  msg->d[1] = distortion.k2().value();
  msg->d[2] = distortion.p1().value();
  msg->d[3] = distortion.p2().value();
  msg->d[4] = distortion.k3().value();

  // Intrinsic camera matrix for the raw (distorted) images.
  //     [fx  0 cx]
  // K = [ 0 fy cy]
  //     [ 0  0  1]
  const auto camera_matrix = intrinsics.cameraMatrix();
  msg->k[0] = camera_matrix.fx().value();
  msg->k[2] = camera_matrix.cx().value();
  msg->k[4] = camera_matrix.fy().value();
  msg->k[5] = camera_matrix.cy().value();
  msg->k[8] = 1;

  // R (identity)
  msg->r[0] = 1;
  msg->r[4] = 1;
  msg->r[8] = 1;

  // Projection/camera matrix
  //     [fx'  0  cx' Tx]
  // P = [ 0  fy' cy' Ty]
  //     [ 0   0   1   0]
  msg->p[0] = camera_matrix.fx().value();
  msg->p[2] = camera_matrix.cx().value();
  msg->p[5] = camera_matrix.fy().value();
  msg->p[6] = camera_matrix.cy().value();
  msg->p[10] = 1;

  return msg;
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
  this->declare_parameter<bool>("use_latched_publisher_for_points_xyz", use_latched_publisher_for_points_xyz_);

  bool update_firmware_automatically = true;
  this->declare_parameter<bool>("update_firmware_automatically", update_firmware_automatically);

  serial_number_ = this->get_parameter("serial_number").as_string();
  frame_id_ = this->get_parameter("frame_id").as_string();
  settings_path_ = this->get_parameter("settings_path").as_string();
  settings2d_path_ = this->get_parameter("settings2d_path").as_string();

  use_latched_publisher_for_points_xyz_ = this->get_parameter("use_latched_publisher_for_points_xyz").as_bool();

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

  capture_service_ = create_service<std_srvs::srv::Empty>(
      "capture", std::bind(&ZividCamera::captureServiceHandler, this, _1, _2, _3));

  capture_2d_service_ = create_service<std_srvs::srv::Empty>(
      "capture_2d", std::bind(&ZividCamera::capture2DServiceHandler, this, _1, _2, _3));

  load_settings_from_file_service_ = create_service<zivid_interfaces::srv::LoadSettingsFromFile>(
      "load_settings_from_file", std::bind(&ZividCamera::loadSettingsFromFileServiceHandler, this, _1, _2, _3));

  load_settings_2d_from_file_service_ = create_service<zivid_interfaces::srv::LoadSettings2DFromFile>(
      "load_settings_2d_from_file", std::bind(&ZividCamera::loadSettings2DFromFileServiceHandler, this, _1, _2, _3));

  rclcpp::QoS qos_default{ rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default) };

  if (use_latched_publisher_for_points_xyz_)
  {
    qos_default.durability(rclcpp::DurabilityPolicy::TransientLocal);
  }
  points_xyz_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("points/xyz", qos_default);
  points_xyzrgba_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("points/xyzrgba", qos_default);

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

  normals_xyz_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("normals/xyz", qos_default);

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
  const bool publish_points_xyz = shouldPublishPointsXYZ();

  const auto header = makeHeader();
  auto point_cloud = frame.pointCloud();

  // Transform point cloud from millimeters (Zivid SDK) to meter (ROS).
  const float scale = 0.001f;
  const auto transformationMatrix = Zivid::Matrix4x4{ scale, 0, 0, 0, 0, scale, 0, 0, 0, 0, scale, 0, 0, 0, 0, 1 };
  point_cloud.transform(transformationMatrix);

  if (publish_points_xyz)
  {
    publishPointCloudXYZ(header, point_cloud);
  }

  publishPointCloudXYZRGBA(header, point_cloud);

  const auto intrinsics = Zivid::Experimental::Calibration::intrinsics(camera_);
  const auto camera_info = makeCameraInfo(header, point_cloud.width(), point_cloud.height(), intrinsics);

  publishColorImage(header, camera_info, point_cloud);
  publishDepthImage(header, camera_info, point_cloud);
  publishSnrImage(header, camera_info, point_cloud);
  publishNormalsXYZ(header, camera_info, point_cloud);
}

bool ZividCamera::shouldPublishPointsXYZ() const
{
  return points_xyz_publisher_->get_subscription_count() > 0 || use_latched_publisher_for_points_xyz_;
}

void ZividCamera::publishPointCloudXYZ(const std_msgs::msg::Header& header, const Zivid::PointCloud& point_cloud)
{
  // We are using the Zivid::XYZW type here for compatibility with the pcl::PointXYZ type, which contains an
  // padding float for performance reasons. We could use the "pcl_conversion" utility functions to construct
  // the PointCloud2 message. However, those are observed to add significant overhead due to extra unnecssary
  // copies of the data.
  using ZividDataType = Zivid::PointXYZW;
  auto msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
  fillCommonMsgFields(*msg, header, point_cloud.width(), point_cloud.height());
  msg->fields.reserve(3);
  msg->fields.push_back(createPointField("x", 0, sensor_msgs::msg::PointField::FLOAT32, 1));
  msg->fields.push_back(createPointField("y", 4, sensor_msgs::msg::PointField::FLOAT32, 1));
  msg->fields.push_back(createPointField("z", 8, sensor_msgs::msg::PointField::FLOAT32, 1));
  msg->is_dense = false;
  msg->point_step = sizeof(ZividDataType);
  msg->row_step = msg->point_step * msg->width;
  msg->data.resize(msg->row_step * msg->height);
  point_cloud.copyData<ZividDataType>(reinterpret_cast<ZividDataType*>(msg->data.data()));
  points_xyz_publisher_->publish(std::move(msg));
}

void ZividCamera::publishPointCloudXYZRGBA(const std_msgs::msg::Header& header, const Zivid::PointCloud& point_cloud)
{
  using ZividDataType = Zivid::PointXYZColorBGRA;
  auto msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
  fillCommonMsgFields(*msg, header, point_cloud.width(), point_cloud.height());
  msg->fields.reserve(4);
  msg->fields.push_back(createPointField("x", 0, sensor_msgs::msg::PointField::FLOAT32, 1));
  msg->fields.push_back(createPointField("y", 4, sensor_msgs::msg::PointField::FLOAT32, 1));
  msg->fields.push_back(createPointField("z", 8, sensor_msgs::msg::PointField::FLOAT32, 1));
  msg->fields.push_back(createPointField("rgba", 12, sensor_msgs::msg::PointField::FLOAT32, 1));
  msg->is_dense = false;
  msg->point_step = sizeof(ZividDataType);
  msg->row_step = msg->point_step * msg->width;
  msg->data.resize(msg->row_step * msg->height);
  point_cloud.copyData<ZividDataType>(reinterpret_cast<ZividDataType*>(msg->data.data()));
  points_xyzrgba_publisher_->publish(std::move(msg));
}

void ZividCamera::publishDepthImage(const std_msgs::msg::Header& header,
                                    const sensor_msgs::msg::CameraInfo::ConstSharedPtr& camera_info,
                                    const Zivid::PointCloud& point_cloud)
{
  auto image = makePointCloudImage<Zivid::PointZ>(point_cloud, header, sensor_msgs::image_encodings::TYPE_32FC1);
  depth_image_publisher_.publish(image, camera_info);
}

void ZividCamera::publishColorImage(const std_msgs::msg::Header& header,
                                    const sensor_msgs::msg::CameraInfo::ConstSharedPtr& camera_info,
                                    const Zivid::PointCloud& point_cloud)
{
  auto image = makePointCloudImage<Zivid::ColorRGBA>(point_cloud, header, sensor_msgs::image_encodings::RGBA8);
  color_image_publisher_.publish(image, camera_info);
}

void ZividCamera::publishSnrImage(const std_msgs::msg::Header& header,
                                  const sensor_msgs::msg::CameraInfo::ConstSharedPtr& camera_info,
                                  const Zivid::PointCloud& point_cloud)
{
  auto image = makePointCloudImage<Zivid::SNR>(point_cloud, header, sensor_msgs::image_encodings::TYPE_32FC1);
  snr_image_publisher_.publish(image, camera_info);
}

void ZividCamera::publishNormalsXYZ(const std_msgs::msg::Header& header,
                                    const sensor_msgs::msg::CameraInfo::ConstSharedPtr& camera_info,
                                    const Zivid::PointCloud& point_cloud)
{
  using ZividDataType = Zivid::NormalXYZ;
  auto msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
  fillCommonMsgFields(*msg, header, point_cloud.width(), point_cloud.height());
  msg->fields.reserve(3);
  msg->fields.push_back(createPointField("normal_x", 0, sensor_msgs::msg::PointField::FLOAT32, 1));
  msg->fields.push_back(createPointField("normal_y", 4, sensor_msgs::msg::PointField::FLOAT32, 1));
  msg->fields.push_back(createPointField("normal_z", 8, sensor_msgs::msg::PointField::FLOAT32, 1));
  msg->is_dense = false;
  msg->point_step = sizeof(ZividDataType);
  msg->row_step = msg->point_step * msg->width;
  msg->data.resize(msg->row_step * msg->height);
  point_cloud.copyData<ZividDataType>(reinterpret_cast<ZividDataType*>(msg->data.data()));
  normals_xyz_publisher_->publish(std::move(msg));
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
                                        const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                                        std::shared_ptr<std_srvs::srv::Empty::Response> response)
{
  (void)request_header;
  (void)request;
  (void)response;

  publishFrame(camera_.capture(settings_));
}

void ZividCamera::capture2DServiceHandler(const std::shared_ptr<rmw_request_id_t> request_header,
                                          const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                                          std::shared_ptr<std_srvs::srv::Empty::Response> response)
{
  (void)request_header;
  (void)request;
  (void)response;

  auto frame = camera_.capture(settings2d_);
  const auto header = makeHeader();
  auto image = frame.imageRGBA();
  const auto camera_info =
      makeCameraInfo(header, image.width(), image.height(), Zivid::Experimental::Calibration::intrinsics(camera_));
  color_image_publisher_.publish(makeColorImage(header, image), camera_info);
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
