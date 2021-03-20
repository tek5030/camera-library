#pragma once

#include "opencv2/core.hpp"
#include "opencv2/core/affine.hpp"
#include <memory>

namespace cv
{ using Vec5f = cv::Vec<float, 5>; }

namespace rs2 { class pipeline; }

namespace tek5030::RealSense
{
/// \brief Interact with a Intel RealSense Camera as a single stream camera.
class SingleStreamCamera
{
public:
  enum class CameraStream {LEFT, RIGHT, COLOR, DEPTH};
  enum class LaserMode {OFF, ON};

  /// \brief Create a new instance of a SingleStreamCamera.
  /// \param active_stream Define the active stream.
  /// \param enable_projector Turn the IR-projector on or off. Default is off.
  /// \see RealSenseCamera::operator>>(cv::Mat&)
  explicit SingleStreamCamera(const CameraStream& active_stream);

  /// \brief Stops the streams.
  ~SingleStreamCamera();

  /// \return An image from the active stream.
  cv::Mat getFrame() const;

  /// \brief Return the frame rate for the active stream.
  double getFrameRate() const;

  /// \brief Return the current resolution for the active stream.
  cv::Size getResolution() const;

  /// \brief Get the intrinsic camera parameters of a camera.
  /// The parameters are stored in the device and can be read out.
  /// \param cam Left or right IR-camera.
  /// \return The K-matrix.
  cv::Matx33f K() const;

  /// \brief Get the distortion parameters of a camera.
  /// \param cam Left or right IR-camera.
  /// \return The distortion parameters.
  cv::Vec5f distortion() const;

  /// \brief Stream operator returning a frame from the 'active_stream_',
  /// configured in the constructor or with setActiveStream.
  /// \param frame The cv::Mat in which to store the frame.
  bool operator>>(cv::Mat& frame) const;

  /// \brief Set the active stream.
  void setActiveStream(const CameraStream& cam);

  /// \brief Set the laser projector on or off.
  /// \param mode On or off.
  void setLaserMode(LaserMode mode);

private:
  std::shared_ptr<rs2::pipeline> pipe_;
  CameraStream active_stream_;
};
}