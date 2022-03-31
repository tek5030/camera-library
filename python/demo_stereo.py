#!/usr/bin/env python
import cv2
import numpy as np
from realsense_stereo import (RealSenseStereoCamera, CaptureMode, LaserMode)


def demo_stereo():
    cam = RealSenseStereoCamera(CaptureMode.RECTIFIED)
    print("Connected to RealSense camera:")
    print(cam)
    print("Press 'l' to toggle laser.")
    print("Press 'u' to toggle rectified/unrectified.")
    print("Press 'q' to quit.")
    
    cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)

    laser_on = False
    rectified = False
    while True:
        frame_1, frame_2 = cam.get_stereo_pair()
        pair = np.hstack((frame_1, frame_2))
        
        pair = cv2.cvtColor(pair, cv2.COLOR_GRAY2BGR)

        # Draw lines along the image rows.
        # For rectified pair, these should coincide with the epipolar lines.
        for i in np.arange(50, pair.shape[0], 50):
            cv2.line(pair, (0, i), (pair.shape[1], i), (0,0,65535))
        
        cv2.imshow('RealSense', pair)
        key = cv2.waitKey(1)
        if key == ord('q'):
            print("Bye")
            break
        elif key == ord('l'):
            laser_on = not laser_on
            cam.laser_mode = LaserMode.ON if laser_on else LaserMode.OFF
            print(f"Laser: {laser_on}")
        elif key == ord('u'):
            rectified = not rectified
            if rectified:
                cam.capture_mode = CaptureMode.RECTIFIED
            else:
                cam.capture_mode = CaptureMode.UNRECTIFIED
            print(f"Rectified: {rectified}")


if __name__ == "__main__":
    demo_stereo()
