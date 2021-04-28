#pragma once

#include "tek5030/stereo_pair.h"
#include "opencv2/core.hpp"
#include "opencv2/core/affine.hpp"
#include <memory>

namespace cv
{ using Vec5f = cv::Matx<float, 5, 1>; }

namespace rs2 { class pipeline; }

namespace tek5030::RealSense
{
/// \brief Interact with a Intel RealSense Camera as a stereo camera.
class StereoCamera
{
public:
  enum class CameraStream {LEFT, RIGHT};
  enum class CaptureMode {RECTIFIED, UNRECTIFIED};
  enum class LaserMode {OFF, ON};

  /// \brief Create a new instance of a StereoCamera.
  /// \param capture_mode Rectified or unrectified data. Use unrectified when you want to perform custom calibration.
  explicit StereoCamera(CaptureMode capture_mode = CaptureMode::RECTIFIED);

  /// \brief Stops the stream.
  ~StereoCamera();

  /// \brief Retrieve a pair of frames from the camera.
  /// The frames are captured simultaneously from the left and right IR-camera.
  /// \return The captured pair.
  [[nodiscard]] StereoPair getStereoPair() const;

  /// \brief Retrieve a timestamped pair of frames from the camera.
  /// The frames are captured simultaneously from the left and right IR-camera.
  /// \return The captured pair.
  [[nodiscard]] StampedStereoPair getStampedStereoPair() const;

  /// \brief Return the frame rate for the given camera.
  /// \param cam Left or right IR-camera.
  [[nodiscard]] double getFrameRate(CameraStream cam) const;

  /// \brief The current resolution for the given camera.
  /// \param cam Left or right IR-camera.
  [[nodiscard]] cv::Size getResolution(CameraStream cam) const;

  /// \brief Get the intrinsic camera parameters of a camera.
  /// The parameters are stored in the device and can be read out.
  /// \param cam Left or right IR-camera.
  /// \return The K-matrix.
  [[nodiscard]] cv::Matx33f K(CameraStream cam) const;

  /// \brief Get the distortion parameters of a camera.
  /// \param cam Left or right IR-camera.
  /// \return The distortion parameters.
  [[nodiscard]] cv::Vec5f distortion(CameraStream cam) const;

  /// \return The rotation and translation (the extrinsic parameters) of the right IR-camera relative to the left IR-camera.
  [[nodiscard]] cv::Affine3f pose() const;

  /// \brief Set the laser projector on or off.
  /// \param mode On or off.
  void setLaserMode(LaserMode mode);

  /// \brief Stream operator returning a timestamped stereo pair.
  /// \param pair The StampedStereoPair in which to store the pair.
  bool operator>>(StampedStereoPair& pair) const;

private:
  std::shared_ptr<rs2::pipeline> pipe_;
};
}