#include "tek5030/realsense_single.h"
#include "realsense_camera_helpers.cpp"

#include <librealsense2/rs.hpp>

namespace
{
using namespace tek5030::RealSense;
enum StreamIndex : int { LEFT = 1, RIGHT = 2, COLOR = -1};

rs2::video_stream_profile getVideoStreamProfile(const rs2::pipeline& pipeline, const SingleStreamCamera::CameraStream& cam);
}

namespace tek5030::RealSense
{
SingleStreamCamera::SingleStreamCamera(const CameraStream& active_stream)
    : pipe_{std::make_shared<rs2::pipeline>()}
    , active_stream_{active_stream}
{
  const cv::Size RGB_size{1920,1080};
  rs2::config cfg;
  cfg.enable_stream(RS2_STREAM_COLOR, RGB_size.width, RGB_size.height, RS2_FORMAT_BGR8);
  cfg.enable_stream(RS2_STREAM_INFRARED, StreamIndex::LEFT,  RS2_FORMAT_Y8);
  cfg.enable_stream(RS2_STREAM_INFRARED, StreamIndex::RIGHT, RS2_FORMAT_Y8);
  cfg.enable_stream(RS2_STREAM_DEPTH, RS2_FORMAT_Z16);

  pipe_->start(cfg);
}

SingleStreamCamera::~SingleStreamCamera()
{
  pipe_->stop();
}

cv::Mat SingleStreamCamera::getFrame() const
{
  const rs2::frameset data = pipe_->wait_for_frames();

  switch (active_stream_)
  {
    case SingleStreamCamera::CameraStream::LEFT:
      return frame_to_mat(data.get_infrared_frame(StreamIndex::LEFT)).clone();
    case SingleStreamCamera::CameraStream::RIGHT:
      return frame_to_mat(data.get_infrared_frame(StreamIndex::RIGHT)).clone();
    case SingleStreamCamera::CameraStream::DEPTH:
      return frame_to_mat(data.get_depth_frame()).clone();
    case SingleStreamCamera::CameraStream::COLOR:
    default:
      return frame_to_mat(data.get_color_frame()).clone();
  }
}

double SingleStreamCamera::getFrameRate() const
{
  const auto stream = getVideoStreamProfile(*pipe_, active_stream_);
  return stream.fps();
}

cv::Size SingleStreamCamera::getResolution() const
{
  const auto stream = getVideoStreamProfile(*pipe_, active_stream_);
  return {stream.width(), stream.height()};
}

cv::Matx33f SingleStreamCamera::K() const
{
  const auto i = getVideoStreamProfile(*pipe_, active_stream_).get_intrinsics();
  cv::Matx33f K{
      i.fx, 0, i.ppx,
      0, i.fy, i.ppy,
      0, 0, 1
  };

  return K;
}

cv::Vec5f SingleStreamCamera::distortion() const
{
  return cv::Vec5f(getVideoStreamProfile(*pipe_, active_stream_).get_intrinsics().coeffs);
}

bool SingleStreamCamera::operator>>(cv::Mat& frame) const
{
  frame = getFrame();
  return !frame.empty();
}

void SingleStreamCamera::setActiveStream(const CameraStream& cam)
{ active_stream_ = cam; }

void SingleStreamCamera::setLaserMode(LaserMode mode)
{
  const auto cam = pipe_->get_active_profile().get_device().first<rs2::depth_sensor>();
  if (cam.supports(RS2_OPTION_EMITTER_ENABLED))
  {
    cam.set_option(RS2_OPTION_EMITTER_ENABLED, static_cast<float>(mode));
  }
}
}

namespace
{

rs2::video_stream_profile getVideoStreamProfile(const rs2::pipeline& pipeline, const SingleStreamCamera::CameraStream& cam)
{
  switch (cam)
  {
    case SingleStreamCamera::CameraStream::LEFT:
      return pipeline.get_active_profile().get_stream(RS2_STREAM_INFRARED, StreamIndex::LEFT).as<rs2::video_stream_profile>();
    case SingleStreamCamera::CameraStream::RIGHT:
      return pipeline.get_active_profile().get_stream(RS2_STREAM_INFRARED, StreamIndex::RIGHT).as<rs2::video_stream_profile>();
    case SingleStreamCamera::CameraStream::DEPTH:
      return pipeline.get_active_profile().get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
    case SingleStreamCamera::CameraStream::COLOR:
    default:
      return pipeline.get_active_profile().get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
  }
}
}
