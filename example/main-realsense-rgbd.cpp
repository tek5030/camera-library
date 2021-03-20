#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "tek5030/realsense_rgbd.h"
#include <iostream>

int main() try
{
  using namespace tek5030;
  using namespace tek5030::RealSense;

  RGBDCamera cam;

  std::cout << "Connected to RealSense camera: " << std::endl;
  std::cout << "Resolution: " << cam.getResolution() << std::endl;
  std::cout << "Frame rate: " << cam.getFrameRate() << std::endl;
  std::cout << std::endl << "Press 'l' to toggle laser." << std::endl;

  const std::string window_name{"frame"};
  cv::namedWindow(window_name, cv::WINDOW_NORMAL);

  bool laser_on = false;

  while (true)
  {
    RGBDImage frame;
    cam >> frame;

    // Convert depth to uint8 (0-5 meters).
    cv::Mat depth;
    frame.depth.convertTo(depth, CV_8UC1, 255.0/5.0);

    // Apply colormap to depth.
    cv::Mat depth_color;
    cv::applyColorMap(depth, depth_color, cv::COLORMAP_JET);

    // Visualize.
    cv::Mat vis;
    cv::hconcat(frame.color, depth_color, vis);
    cv::imshow(window_name, vis);

    auto key = cv::waitKey(1);
    if (key == static_cast<int>('l'))
    {
      laser_on = !laser_on;
      cam.setLaserMode(laser_on ? RGBDCamera::LaserMode::ON : RGBDCamera::LaserMode::OFF);
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
