#!/usr/bin/env python
import cv2
import numpy as np
from realsense_common import (CameraStream, CaptureMode, LaserMode, Size)
from realsense_mono import (RealSenseSingleStreamCamera)


def demo_mono():
    cam = RealSenseSingleStreamCamera(CameraStream.LEFT)
    print("Connected to RealSense camera:")
    print("Press 'l' to toggle laser.")
    print("Press 'u' to toggle rectified/unrectified.")
    print("Press 'q' to quit.")
    
    cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)

    laser_on = False
    rectified = True
    while True:
        frame = cam.get_frame()
        
        cv2.imshow('RealSense', frame)
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
            cam.capture_mode = CaptureMode.RECTIFIED if rectified else CaptureMode.UNRECTIFIED
            print(f"Rectified: {rectified}")
        elif key == ord('a'):
            cam.active_stream = CameraStream.LEFT
        elif key == ord('s'):
            cam.active_stream = CameraStream.COLOR
        elif key == ord('d'):
            cam.active_stream = CameraStream.RIGHT
        elif key == ord('f'):
            cam.active_stream = CameraStream.DEPTH


if __name__ == "__main__":
    demo_mono()
