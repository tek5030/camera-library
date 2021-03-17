## tek5030 camera library
This library contains wrappers for cameras used in the course tek5030.
The classes are designed in such a way that it should be as easy as possible to switch between cameras.

The following camera classes are available
- `tek5030::OpenCVCamera`, _super thin wrapper around cv::VideoCapture_
- `tek5030::DualCamera`, _capture images simultaneously-ish from two cv::VideoCapture objects so that we can use ordinary webcams as stereo cameras_
- `tek5030::RealSense::SingleStreamCamera`, _single stream interface to Intel® RealSense™ D435_
- `tek5030::RealSense::StereoCamera`, _stereo interface to Intel® RealSense™ D435_
- `tek5030::RealSense::RGBDCamera`, _RGBD interface to Intel® RealSense™ D435_

### Dependencies
- OpenCV
- [librealsense](https://github.com/IntelRealSense/librealsense) (SDK for Intel® RealSense™ cameras)

Install librealsense:
```bash
sudo apt-key adv --keyserver keys.gnupg.net --recv-key C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C8B3A55A6F3EFCDE
sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo $(lsb_release -sc) main" -u

sudo apt install -y librealsense2-{dkms,utils,dev,dbg}
```

### Install this library
```bash
cd <path to library>
mkdir build && cd $_
cmake .. [-DCMAKE_INSTALL_PREFIX=<custom prefix>]
make
(sudo) make install
```
### Use this library in your project
In your _CMakeLists.txt_:
```cmake
find_package(tek5030 CONFIG REQUIRED)
add_executable(my_exe my_main.cpp)
target_link_libraries(my_exe PRIVATE tek5030::tek5030)
```

Stream single frames:
```c++
#include "tek5030/opencv_camera.h"     // Super thin wrapper around cv::VideoCapture
#include "tek5030/dual_camera.h"       // Stereo from two cv::VideoCapture cameras
#include "tek5030/realsense_single.h"  // Use a RealSense camera

using namespace tek5030;
using namespace tek5030::RealSense;

// Initialize with device ID
OpenCVCamera cam(0);

// Initialize with device IDs 
DualCamera cam(0,1);

// Initialize with a default camera feed (for use with the operator>>)
SingleStreamCamera cam(SingleStreamCamera::CameraStream::COLOR);

cv::Mat frame;
cam >> frame;
```

Fetch a StereoPair:
```c++
// Available from RealSense::StereoCamera or DualCamera
tek5030::StereoPair stereo_pair = cam.getStereoPair();
```
Access and configure the internal `cv::VideoCapture` object:
```c++
// Available from DualCamera or OpenCVCamera
auto& cap = cam.getVideoCapture();
cap.set(cv::CAP_PROP_xyz, value); 
```

