#include "tek5030/realsense_stereo.h"
#include "realsense_camera_helpers.cpp"

#include "librealsense2/rs.hpp"

namespace
{
using namespace tek5030::RealSense;
enum StreamIndex : int { LEFT = 1, RIGHT = 2};

rs2::video_stream_profile getVideoStreamProfile(const rs2::pipeline& pipeline, const StereoCamera::CameraStream& cam);
}

namespace tek5030::RealSense
{
StereoCamera::StereoCamera(CaptureMode capture_mode)
    : pipe_{std::make_shared<rs2::pipeline>()}
{
  setCaptureMode(capture_mode);
}

StereoCamera::~StereoCamera()
{
  pipe_->stop();
}

StereoPair StereoCamera::getStereoPair() const
{
  const rs2::frameset data = pipe_->wait_for_frames();
  const rs2::frame left = data.get_infrared_frame(StreamIndex::LEFT);
  const rs2::frame right = data.get_infrared_frame(StreamIndex::RIGHT);
  return {frame_to_mat(left).clone(), frame_to_mat(right).clone()};
}

StampedStereoPair StereoCamera::getStampedStereoPair() const
{
  const rs2::frameset data = pipe_->wait_for_frames();
  const rs2::frame left = data.get_infrared_frame(StreamIndex::LEFT);
  const rs2::frame right = data.get_infrared_frame(StreamIndex::RIGHT);
  const double usec_timestamp = data.get_frame_metadata(rs2_frame_metadata_value::RS2_FRAME_METADATA_FRAME_TIMESTAMP);
  return {{frame_to_mat(left).clone(), frame_to_mat(right).clone()}, usec_timestamp};
}

double StereoCamera::getFrameRate(CameraStream cam) const
{
  const auto stream = getVideoStreamProfile(*pipe_, cam);
  return stream.fps();
}

cv::Size StereoCamera::getResolution(CameraStream cam) const
{
  const auto stream = getVideoStreamProfile(*pipe_, cam);
  return {stream.width(), stream.height()};
}

cv::Matx33f StereoCamera::K(CameraStream cam) const
{
  const auto i = getVideoStreamProfile(*pipe_, cam).get_intrinsics();
  cv::Matx33f K{
      i.fx, 0, i.ppx,
      0, i.fy, i.ppy,
      0, 0, 1
  };

  return K;
}

cv::Vec5f StereoCamera::distortion(CameraStream cam) const
{
  return cv::Vec5f(getVideoStreamProfile(*pipe_, cam).get_intrinsics().coeffs);
}

cv::Affine3f StereoCamera::pose() const
{
  const rs2::pipeline_profile selection = pipe_->get_active_profile();
  const auto left_stream = selection.get_stream(RS2_STREAM_INFRARED, StreamIndex::LEFT);
  const auto right_stream = selection.get_stream(RS2_STREAM_INFRARED, StreamIndex::RIGHT);
  const auto e = left_stream.get_extrinsics_to(right_stream);

  const cv::Matx33f R = cv::Matx33f{e.rotation}.t();
  const cv::Vec3f t{e.translation};

  return {R, t};
}

bool StereoCamera::operator>>(StampedStereoPair& pair) const
{
  pair = getStampedStereoPair();
  return !(pair.stereo_pair.left.empty() || pair.stereo_pair.right.empty());
}

void StereoCamera::setLaserMode(LaserMode mode)
{
  const auto cam = pipe_->get_active_profile().get_device().first<rs2::depth_sensor>();
  if (cam.supports(RS2_OPTION_EMITTER_ENABLED))
  {
    cam.set_option(RS2_OPTION_EMITTER_ENABLED, static_cast<float>(mode));
  }
}

void StereoCamera::setCaptureMode(const CaptureMode& capture_mode)
{
  const cv::Size IR_size{1280,800};
  rs2::config cfg;
  cfg.disable_all_streams();

  switch (capture_mode)
  {
    case StereoCamera::CaptureMode::UNRECTIFIED:
    // Use Y16 mode for unrectified frames.
    cfg.enable_stream(RS2_STREAM_INFRARED, StreamIndex::LEFT,  IR_size.width, IR_size.height, RS2_FORMAT_Y16);
    cfg.enable_stream(RS2_STREAM_INFRARED, StreamIndex::RIGHT, IR_size.width, IR_size.height, RS2_FORMAT_Y16);
    break;

    case StereoCamera::CaptureMode::RECTIFIED:
    default:
    // Use Y8 mode for rectified frames.
    cfg.enable_stream(RS2_STREAM_INFRARED, StreamIndex::LEFT,  IR_size.width, IR_size.height, RS2_FORMAT_Y8);
    cfg.enable_stream(RS2_STREAM_INFRARED, StreamIndex::RIGHT, IR_size.width, IR_size.height, RS2_FORMAT_Y8);
  }

  try
  { pipe_->stop();}
  catch(...){}

  if (!pipe_->start(cfg))
  { throw std::runtime_error("configuring pipeline failed"); }
}

}

namespace
{
rs2::video_stream_profile getVideoStreamProfile(const rs2::pipeline& pipeline, const tek5030::RealSense::StereoCamera::CameraStream& cam)
{
  switch (cam)
  {
    case StereoCamera::CameraStream::RIGHT:
      return pipeline.get_active_profile().get_stream(RS2_STREAM_INFRARED, StreamIndex::RIGHT).as<rs2::video_stream_profile>();
    case StereoCamera::CameraStream::LEFT:
    default:
      return pipeline.get_active_profile().get_stream(RS2_STREAM_INFRARED, StreamIndex::LEFT).as<rs2::video_stream_profile>();
  }
}
}
