## tek5030 camera library
This library contains wrappers for cameras used in the course tek5030.
The classes are designed in such a way that it should be as easy as possible to switch between cameras.

The following camera classes are available
- `tek5030::OpenCVCamera`, _super thin wrapper around cv::VideoCapture_
- `tek5030::DualCamera`, _capture images simultaneously-ish from two cv::VideoCapture objects so that we can use ordinary webcams as stereo cameras_
- `tek5030::KittiCamera`, _capture image pairs from a [Kitti Dataset](http://www.cvlibs.net/datasets/kitti/raw_data.php), downloaded by you._
- `tek5030::RealSense::SingleStreamCamera`, _single stream interface to Intel® RealSense™ D435_
- `tek5030::RealSense::StereoCamera`, _stereo interface to Intel® RealSense™ D435_
- `tek5030::RealSense::RGBDCamera`, _RGBD interface to Intel® RealSense™ D435_

### Dependencies
- OpenCV
- [librealsense](https://github.com/IntelRealSense/librealsense) (SDK for Intel® RealSense™ cameras)

##### Install librealsense:
```bash
sudo apt-key adv --keyserver keys.gnupg.net --recv-key C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C8B3A55A6F3EFCDE
sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo $(lsb_release -sc) main" -u

sudo apt install -y librealsense2-{dkms,utils,dev,dbg}
```
- After installing the realsense libraries, you should run `realsense-viewer` from the terminal.
  This will allow you to update the camera firmware (you will get a notification if a new version is available) and to test the camera.
  

### Install this library
```bash
cd <path to library>
mkdir build && cd $_
cmake .. [-DCMAKE_INSTALL_PREFIX=<custom prefix>] [-DBUILD_EXAMPLES=ON]
make
(sudo) make install
```
- `sudo make install` without any `CMAKE_INSTALL_PREFIX` will install the library under `/usr/local`.   
  This is convenient for the non-experienced user, as it makes the library easy to locate for other projects (labs).
- `-DBUILD_EXAMPLES=ON` will build the example project included with this library. You will then be able to test
  your cameras.

### Use this library in your project
In your _CMakeLists.txt_:
```cmake
find_package(tek5030 CONFIG OPTIONAL_COMPONENTS rs2)
add_executable(my_exe my_main.cpp)
target_link_libraries(my_exe PRIVATE tek5030::tek5030)

if(TARGET tek5030::rs2)  #  if(tek5030_rs2_FOUND) does also work
  target_link_libraries(my_exe PRIVATE tek5030::rs2)
endif()
```

See the `example` directory for complete, working examples for all camera types.

Stream single frames:
```c++
#include "tek5030/opencv_camera.h"     // Super thin wrapper around cv::VideoCapture
#include "tek5030/realsense_single.h"  // Use a RealSense camera

using namespace tek5030;
using namespace tek5030::RealSense;

// Initialize with device ID
OpenCVCamera cam(0);

// Initialize with a default camera feed (for use with the operator>>)
SingleStreamCamera cam(SingleStreamCamera::CameraStream::COLOR);

// Fetch a single frame
cv::Mat frame;
cam >> frame;
```

Fetch a StereoPair:
```c++
#include "tek5030/dual_camera.h"       // Stereo from two cv::VideoCapture cameras
#include "tek5030/kitti_camera.h"      // Stereo from image pairs from a Kitti dataset
#include "tek5030/realsense_stereo.h"  // Use a RealSense camera

using namespace tek5030;
using namespace tek5030::RealSense;

// Initialize with device IDs 
DualCamera cam(0,1);

// Initialize with path to dataset and calibration. Select color or monochrome camera pair.
bool color = false;
KittiCamera cam("path_to_data", "path_to_calib", color)

// Use a RealSense
RealSense::StereoCamera cam(CaptureMode::UNRECTIFIED);
cam.setLaserMode(LaserMode::OFF)

// Available from DualCamera, KittiCamera or RealSense::StereoCamera
const StereoPair stereo_pair = cam.getStereoPair();
// or
const auto [left, right] = cam.getStereoPair();
```
Access and configure the internal `cv::VideoCapture` object:
```c++
// Available from DualCamera or OpenCVCamera
auto& cap = cam.getVideoCapture();
cap.set(cv::CAP_PROP_xyz, value); 
```
Retreive the calibration data for a `KittiCamera`
```cpp
const auto calibration_data = cam.getCalibration(KittiCamera::Cam::GrayLeft);
```
Retreive the calibration data for a `RealSense` camera
```cpp
const cv::Matx33f K = cam.K(CameraStream::LEFT);
const cv::Vec5f distortion = cam.distortion(CameraStream::LEFT);
```

