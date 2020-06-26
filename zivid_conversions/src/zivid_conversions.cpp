// Copyright (c) 2020 Norwegian University of Science and Technology
// Copyright (c) 2019, Zivid AS
// Use of this source code is governed by the BSD 3-Clause license, see LICENSE

#include "zivid_conversions/zivid_conversions.hpp"

namespace zivid_conversions
{
sensor_msgs::msg::PointCloud2::UniquePtr makePointCloud2(const std_msgs::msg::Header& header,
                                                         const Zivid::PointCloud& point_cloud)
{
  auto msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
  fillCommonMsgFields(*msg, header, point_cloud.width(), point_cloud.height());
  msg->point_step = sizeof(Zivid::PointXYZColorRGBA);
  msg->row_step = msg->point_step * msg->width;
  msg->is_dense = false;

  msg->fields.reserve(5);
  msg->fields.push_back(createPointField("x", 0, 7, 1));
  msg->fields.push_back(createPointField("y", 4, 7, 1));
  msg->fields.push_back(createPointField("z", 8, 7, 1));
  msg->fields.push_back(createPointField("rgba", 12, 7, 1));

  auto data = point_cloud.copyData<Zivid::PointXYZColorRGBA>().data();

  msg->data = std::vector<uint8_t>(reinterpret_cast<const uint8_t*>(data),
                                   reinterpret_cast<const uint8_t*>(data + point_cloud.size()));

#pragma omp parallel for default(none) shared(point_cloud, msg)
  for (std::size_t i = 0; i < point_cloud.size(); i++)
  {
    uint8_t* point_ptr = &(msg->data[i * sizeof(Zivid::PointXYZColorRGBA)]);
    float* x_ptr = reinterpret_cast<float*>(&(point_ptr[0]));
    float* y_ptr = reinterpret_cast<float*>(&(point_ptr[4]));
    float* z_ptr = reinterpret_cast<float*>(&(point_ptr[8]));

    // Convert from mm to m
    *x_ptr *= 0.001f;
    *y_ptr *= 0.001f;
    *z_ptr *= 0.001f;

    // Swap blue and red channels
    uint8_t b = point_ptr[12];
    point_ptr[12] = point_ptr[14];
    point_ptr[14] = b;
  }
  return msg;
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
//
// sensor_msgs::msg::Image::ConstSharedPtr makeDepthImage(const std_msgs::msg::Header& header,
//                                                       const Zivid::PointCloud& point_cloud)
//{
//  auto msg = std::make_shared<sensor_msgs::msg::Image>();
//  fillCommonMsgFields(*msg, header, point_cloud.width(), point_cloud.height());
//  msg->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
//  msg->step = static_cast<uint32_t>(4 * point_cloud.width());
//  msg->data.resize(msg->step * msg->height);
//
//#pragma omp parallel for
//  for (std::size_t i = 0; i < point_cloud.size(); i++)
//  {
//    float* image_data = reinterpret_cast<float*>(&msg->data[4 * i]);
//    // Convert from mm to m
//    *image_data = point_cloud(i).z * 0.001f;
//  }
//  return msg;
//}
//
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

}  // namespace zivid_conversions
