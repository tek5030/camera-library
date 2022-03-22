#!/usr/bin/env python
import pyrealsense2 as rs2
import numpy as np
import cv2

class Size:
    """Represents image size"""

    def __init__(self, width: float, height: float):
        self._width = width
        self._height = height

    @classmethod
    def from_numpy_shape(cls, shape):
        return cls(*shape[1::-1])

    @property
    def width(self):
        return self._width

    @property
    def height(self):
        return self._height

def get_video_stream_profile(pipeline, camera):
    if camera == "left":
        return pipeline.get_active_profile().get_stream(rs2.stream.infrared, 1).as_video_stream_profile()
    elif camera == "right":
        return pipeline.get_active_profile().get_stream(rs2.stream.infrared, 2).as_video_stream_profile()
    return None

class StereoCamera:
    def __init__(self, capture_mode):
        ir_size = Size(width=1280, height=800)

        self._pipe = rs2.pipeline()
        self.set_capture_mode(capture_mode)
    
    def __del__(self):
        self._pipe.stop()
    
    def get_info(self):
        device = self._pipe.get_active_profile().get_device()
        serial_number = device.get_info(rs2.camera_info.serial_number)
        device_product_line = str(device.get_info(rs2.camera_info.product_line))

        print(f" product line: {device_product_line}\n serial: {serial_number}")

    def get_stereo_pair(self):
        data = self._pipe.wait_for_frames()
        frame_1 = np.asanyarray(data.get_infrared_frame(1).get_data())
        frame_2 = np.asanyarray(data.get_infrared_frame(2).get_data())
        return frame_1, frame_2
    
    def get_stamped_stereo_pair(self):
        data = self._pipe.wait_for_frames()
        frame_1 = np.asanyarray(data.get_infrared_frame(1).get_data())
        frame_2 = np.asanyarray(data.get_infrared_frame(2).get_data())
        usec_timestamp = data.get_frame_metadata(rs2.frame_metadata_value.frame_timestamp)
        return frame_1, frame_2, usec_timestamp
    
    def get_framerate(self, camera):
        profile = get_video_stream_profile(self._pipe, camera)
        return profile.fps()
    
    def get_resolution(self, camera):
        profile = get_video_stream_profile(self._pipe, camera)
        return profile.width(), profile.height()
    
    def get_k_matrix(self, camera):
        """bare for rs2.format.y8"""
        i = get_video_stream_profile(self._pipe, camera).get_intrinsics()
        return np.array([
            [i.fx, 9, i.ppx],
            [0, i.fy, i.ppy],
            [0, 0, 1]
        ])
    
    def get_distortion(self, camera):
        """bare for rs2.format.y8"""
        d = get_video_stream_profile(self._pipe, camera).get_intrinsics().coeffs
        return np.array([d])
    
    def get_pose(self):
        selection = self._pipe.get_active_profile()
        left_stream = selection.get_stream(rs2.stream.infrared, 1)
        right_stream = selection.get_stream(rs2.stream.infrared, 2)
        e = left_stream.get_extrinsics_to(right_stream)
        R = np.asarray(e.rotation).reshape((3,3))
        t = np.asarray(e.translation)

        return R, t
    
    def set_capture_mode(self, mode):
        if mode == 'rectified':
            mode = rs2.format.y8
        elif mode == 'unrectified':
            mode = rs2.format.y16
        else:
            return

        ir_size = Size(width=1280, height=800)

        try:
            self._pipe.stop()
        except:
            pass
        cfg = rs2.config()
        cfg.disable_all_streams()
        cfg.enable_stream(rs2.stream.infrared, 1, ir_size.width, ir_size.height, mode)
        cfg.enable_stream(rs2.stream.infrared, 2, ir_size.width, ir_size.height, mode)
        self._pipe.start(cfg)

    def set_laser_mode(self, laser_on):
        """'on' or anyting"""
        if laser_on == True:
            laser_on = 1
        else:
            laser_on = 0
        depth_sensor = self._pipe.get_active_profile().get_device().first_depth_sensor()
        if depth_sensor.supports(rs2.option.emitter_enabled):
            depth_sensor.set_option(rs2.option.emitter_enabled, laser_on)

def demo_stereo():
    cam = StereoCamera('unrectified')
    print("Connected to RealSense camera:")
    cam.get_info()
    print(f" resolution: {cam.get_resolution('left')}")
    print(f" framerate: {cam.get_framerate('left')}")
    print("Press 'l' to toggle laser.")
    print("Press 'u' to toggle rectified/unrectified.")
    print("Press 'q' to quit.")
    
    cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)

    laser_on = False
    rectified = False
    while True:
        frame_1, frame_2, ts = cam.get_stamped_stereo_pair()
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
            cam.set_laser_mode(laser_on)
            print(f"Laser: {laser_on}")
        elif key == ord('u'):
            rectified = not rectified
            if rectified:
                cam.set_capture_mode('rectified')
            else:
                cam.set_capture_mode('unrectified')
            print(f"Rectified: {rectified}")


if __name__ == "__main__":
    demo_stereo()
