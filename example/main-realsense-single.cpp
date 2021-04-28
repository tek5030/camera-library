#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "tek5030/realsense_single.h"
#include <iostream>

int main() try
{
  using namespace tek5030;
  using namespace tek5030::RealSense;

  SingleStreamCamera cam(SingleStreamCamera::CameraStream::COLOR);

  std::cout << "Connected to RealSense camera: " << std::endl;
  std::cout << "Resolution: " << cam.getResolution() << std::endl;
  std::cout << "Frame rate: " << cam.getFrameRate() << std::endl;
  std::cout << std::endl;
  std::cout << "Press '1' for color stream." << std::endl;
  std::cout << "Press '2' for left IR stream." << std::endl;
  std::cout << "Press '3' for right IR stream." << std::endl;
  std::cout << "Press '4' for depth stream." << std::endl;
  std::cout << "Press 'l' to toggle laser." << std::endl;

  const std::string window_name{"frame"};
  cv::namedWindow(window_name, cv::WINDOW_NORMAL);

  bool laser_on = false;

  while (true)
  {
    cv::Mat frame;
    cam >> frame;

    cv::imshow(window_name, frame);

    auto key = cv::waitKey(1);
    if (key == static_cast<int>('l'))
    {
      laser_on = !laser_on;
      cam.setLaserMode(laser_on ? SingleStreamCamera::LaserMode::ON : SingleStreamCamera::LaserMode::OFF);
    }
    else if (key == static_cast<int>('1'))
    {
      cam.setActiveStream(SingleStreamCamera::CameraStream::COLOR);
    }
    else if (key == static_cast<int>('2'))
    {
      cam.setActiveStream(SingleStreamCamera::CameraStream::LEFT);
    }
    else if (key == static_cast<int>('3'))
    {
      cam.setActiveStream(SingleStreamCamera::CameraStream::RIGHT);
    }
    else if (key == static_cast<int>('4'))
    {
      cam.setActiveStream(SingleStreamCamera::CameraStream::DEPTH);
    }
    else if (key >= 0)
    { break; }
  }
  return EXIT_SUCCESS;
}
catch (const std::exception& e)
{
  std::cerr << e.what() << std::endl;
  return EXIT_FAILURE;
}
