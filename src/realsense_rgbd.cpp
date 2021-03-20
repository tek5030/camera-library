#include "tek5030/realsense_rgbd.h"
#include "realsense_camera_helpers.cpp"

#include "librealsense2/rs.hpp"

namespace tek5030::RealSense
{
RGBDCamera::RGBDCamera()
    : pipe_{std::make_shared<rs2::pipeline>()}
{
  const cv::Size RGB_size{1920, 1080};
  rs2::config cfg;
  cfg.enable_stream(RS2_STREAM_COLOR, RGB_size.width, RGB_size.height, RS2_FORMAT_BGR8);
  cfg.enable_stream(RS2_STREAM_DEPTH, RS2_FORMAT_Z16);

  pipe_->start(cfg);
}

RGBDCamera::~RGBDCamera()
{
  pipe_->stop();
}

RGBDImage RGBDCamera::getRGBDImage() const
{
  /* https://github.com/IntelRealSense/librealsense/wiki/Projection-in-RealSense-SDK-2.0#frame-alignment */

  rs2::align align(RS2_STREAM_COLOR);
  const rs2::frameset data = pipe_->wait_for_frames();
  const auto aligned_frames = align.process(data);
  const rs2::video_frame color_frame = aligned_frames.first(RS2_STREAM_COLOR);
  const rs2::depth_frame aligned_depth_frame = aligned_frames.get_depth_frame();

  return {
    frame_to_mat(color_frame).clone(),
    depth_frame_to_meters(*pipe_, aligned_depth_frame).clone(),
    aligned_frames.get_timestamp()
  };
}

double RGBDCamera::getFrameRate() const
{
  const auto stream = pipe_->get_active_profile().get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
  return stream.fps();
}

cv::Size RGBDCamera::getResolution() const
{
  const auto stream = pipe_->get_active_profile().get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
  return {stream.width(), stream.height()};
}

cv::Matx33f RGBDCamera::K() const
{
  const auto stream = pipe_->get_active_profile().get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
  const auto i = stream.get_intrinsics();
  cv::Matx33f K{
      i.fx, 0, i.ppx,
      0, i.fy, i.ppy,
      0, 0, 1
  };

  return K;
}

cv::Vec5f RGBDCamera::distortion() const
{
  const auto stream = pipe_->get_active_profile().get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
  return cv::Vec5f(stream.get_intrinsics().coeffs);
}

bool RGBDCamera::operator>>(RGBDImage& frame) const
{
  frame = getRGBDImage();
  return !frame.color.empty();
}

void RGBDCamera::setLaserMode(LaserMode mode)
{
  const auto cam = pipe_->get_active_profile().get_device().first<rs2::depth_sensor>();
  if (cam.supports(RS2_OPTION_EMITTER_ENABLED))
  {
    cam.set_option(RS2_OPTION_EMITTER_ENABLED, static_cast<float>(mode));
  }
}
}
