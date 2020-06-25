// Copyright (c) 2020 Norwegian University of Science and Technology
// Copyright (c) 2019, Zivid AS
// Use of this source code is governed by the BSD 3-Clause license, see LICENSE

#pragma once

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/distortion_models.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <Zivid/Image.h>
#include <Zivid/PointCloud.h>
#include <Zivid/CameraIntrinsics.h>

#include "zivid_conversions/visibility_control.h"

namespace zivid_conversions
{
inline sensor_msgs::msg::PointField createPointField(std::string name, uint32_t offset, uint8_t datatype,
                                                     uint32_t count)
{
  sensor_msgs::msg::PointField point_field;
  point_field.name = name;
  point_field.offset = offset;
  point_field.datatype = datatype;
  point_field.count = count;
  return point_field;
}

inline bool big_endian()
{
  return false;
}

template <class T>
inline void fillCommonMsgFields(T& msg, const std_msgs::msg::Header& header, std::size_t width, std::size_t height)
{
  msg.header = header;
  msg.height = static_cast<uint32_t>(height);
  msg.width = static_cast<uint32_t>(width);
  msg.is_bigendian = big_endian();
}

sensor_msgs::msg::PointCloud2::UniquePtr makePointCloud2(const std_msgs::msg::Header& header,
                                                         const Zivid::PointCloud& point_cloud);

sensor_msgs::msg::Image::ConstSharedPtr makeColorImage(const std_msgs::msg::Header& header,
                                                       const Zivid::PointCloud& point_cloud);

sensor_msgs::msg::Image::ConstSharedPtr makeColorImage(const std_msgs::msg::Header& header,
                                                       const Zivid::Image<Zivid::ColorRGBA>& image);

sensor_msgs::msg::Image::ConstSharedPtr makeDepthImage(const std_msgs::msg::Header& header,
                                                       const Zivid::PointCloud& point_cloud);

sensor_msgs::msg::CameraInfo::ConstSharedPtr makeCameraInfo(const std_msgs::msg::Header& header, std::size_t width,
                                                            std::size_t height,
                                                            const Zivid::CameraIntrinsics& intrinsics);

}  // namespace zivid_conversions