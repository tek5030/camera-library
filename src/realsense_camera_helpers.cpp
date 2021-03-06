//#pragma once

#include "librealsense2/rs.hpp"
#include "opencv2/imgproc.hpp"
#include <exception>

// Convert rs2::frame to cv::Mat
inline cv::Mat frame_to_mat(const rs2::frame& f)
{
  using namespace cv;
  using namespace rs2;

  auto vf = f.as<video_frame>();
  const int w = vf.get_width();
  const int h = vf.get_height();

  if (f.get_profile().format() == RS2_FORMAT_BGR8)
  {
    cv::Mat mat(h, w, CV_8UC3);
    std::memcpy(mat.data, f.get_data(), mat.total() * mat.elemSize());
    return mat;
  }
  else if (f.get_profile().format() == RS2_FORMAT_RGB8)
  {
    cv::Mat mat(h, w, CV_8UC3);
    std::memcpy(mat.data, f.get_data(), mat.total() * mat.elemSize());
    cvtColor(mat, mat, COLOR_RGB2BGR);
    return mat;
  }
  else if (f.get_profile().format() == RS2_FORMAT_Z16)
  {
    cv::Mat mat(h, w, CV_16UC1);
    std::memcpy(mat.data, f.get_data(), mat.total() * mat.elemSize());
    return mat;
  }
  else if (f.get_profile().format() == RS2_FORMAT_Y8)
  {
    cv::Mat mat(h, w, CV_8UC1);
    std::memcpy(mat.data, f.get_data(), mat.total() * mat.elemSize());
    return mat;
  }
  else if (f.get_profile().format() == RS2_FORMAT_Y16)
  {
    cv::Mat mat(h, w, CV_16UC1);
    std::memcpy(mat.data, f.get_data(), mat.total() * mat.elemSize());
    return mat;
  }

  throw std::runtime_error("Frame format is not supported yet!");
}

// Converts depth frame to a matrix of doubles with distances in meters
inline cv::Mat depth_frame_to_meters(const rs2::pipeline& pipe, const rs2::depth_frame& f)
{
  using namespace cv;
  using namespace rs2;

  Mat dm = frame_to_mat(f);
  dm.convertTo(dm, CV_64F);
  auto depth_scale = pipe.get_active_profile()
    .get_device()
    .first<depth_sensor>()
    .get_depth_scale();
  dm = dm * depth_scale;
  return dm;
}
