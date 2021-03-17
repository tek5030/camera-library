#pragma once

#include "opencv2/core.hpp"
#include "opencv2/core/affine.hpp"
#include <memory>

namespace cv
{ using Vec5f = cv::Vec<float, 5>; }

namespace rs2 { class pipeline; }

namespace tek5030::RealSense
{
struct RGBDImage
{
  cv::Mat color;
  cv::Mat depth;
  double usec_timestamp;
};

/// \brief Interact with a Intel RealSense Camera as a RGBD camera.
class RGBDCamera
{
public:
  enum class LaserMode {OFF, ON};

  /// \brief Create a new instance of a RGBDCamera.
  explicit RGBDCamera();

  /// \brief Stops the stream.
  ~RGBDCamera();

  /// \brief Retrieve a RGBD frame from the camera.
  /// \return The captured frame.
  RGBDImage getRGBDImage() const;

  /// \brief Return the frame rate for the camera.
  double getFrameRate() const;

  /// \brief Return the current resolution for the camera.
  cv::Size getResolution() const;

  /// \brief Get the intrinsic camera parameters of the camera.
  /// The parameters are stored in the device and can be read out.
  /// \return The K-matrix.
  cv::Matx33f K() const;

  /// \brief Get the distortion parameters of the camera.
  /// \return The distortion parameters.
  cv::Vec5f distortion() const;

  /// \brief Set the laser projector on or off.
  /// \param mode On or off.
  void setLaserMode(LaserMode mode);

  /// \brief Stream operator returning a RGBD frame.
  /// \param frame The RGBDImage in which to store the frame.
  bool operator>>(RGBDImage& frame) const;

private:
  std::shared_ptr<rs2::pipeline> pipe_;
};
}
