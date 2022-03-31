import pyrealsense2 as rs2
import numpy as np
from enum import Enum, IntEnum, unique, auto
from realsense_common import (CaptureMode, CameraIndex, CameraStream, LaserMode, Size, StereoPair)


class RealSenseStereoCamera:
    def __init__(self, capture_mode: CaptureMode = CaptureMode.RECTIFIED):
        self._pipe = rs2.pipeline()
        self.capture_mode = capture_mode
        self.laser_mode = LaserMode.OFF

    def __del__(self):
        self._pipe.stop()

    def __str__(self):
        device = self._pipe.get_active_profile().get_device()
        serial_number = device.get_info(rs2.camera_info.serial_number)
        device_product_line = str(device.get_info(rs2.camera_info.product_line))

        return ("RealSense:\n"
                f"  product line: {device_product_line}\n"
                f"  serial: {serial_number}\n"
                f"  resolution: {self.get_resolution(CameraIndex.LEFT)}\n"
                f"  framerate: {self.get_framerate(CameraIndex.LEFT)}"
                )

    @property
    def capture_mode(self):
        return self._capture_mode

    @capture_mode.setter
    def capture_mode(self, capture_mode: CaptureMode):
        self._capture_mode = capture_mode

        if capture_mode is CaptureMode.RECTIFIED:
            mode = rs2.format.y8
        elif capture_mode is CaptureMode.UNRECTIFIED:
            mode = rs2.format.y16

        ir_size = Size(width=640, height=480)

        try:
            self._pipe.stop()
        except RuntimeError:
            pass
        cfg = rs2.config()
        cfg.disable_all_streams()
        cfg.enable_stream(rs2.stream.infrared, int(CameraIndex.LEFT), width=ir_size.width, height=ir_size.height, format=mode)
        cfg.enable_stream(rs2.stream.infrared, int(CameraIndex.RIGHT), width=ir_size.width, height=ir_size.height, format=mode)
        self._pipe.start(cfg)

    @property
    def laser_mode(self):
        return

    @laser_mode.setter
    def laser_mode(self, laser_mode: LaserMode):
        depth_sensor = self._pipe.get_active_profile().get_device().first_depth_sensor()

        if not depth_sensor.supports(rs2.option.emitter_enabled):
            return

        if laser_mode is LaserMode.ON:
            depth_sensor.set_option(rs2.option.emitter_enabled, 1)
        else:
            depth_sensor.set_option(rs2.option.emitter_enabled, 0)

    def get_stereo_pair(self) -> StereoPair:
        data = self._pipe.wait_for_frames()
        frame_l = np.asanyarray(data.get_infrared_frame(CameraIndex.LEFT).get_data())
        frame_r = np.asanyarray(data.get_infrared_frame(CameraIndex.RIGHT).get_data())
        return StereoPair(frame_l, frame_r)

    def get_stamped_stereo_pair(self):
        data = self._pipe.wait_for_frames()
        frame_l = np.asanyarray(data.get_infrared_frame(int(CameraIndex.LEFT)).get_data())
        frame_r = np.asanyarray(data.get_infrared_frame(int(CameraIndex.RIGHT)).get_data())
        usec_timestamp = data.get_frame_metadata(rs2.frame_metadata_value.frame_timestamp)
        return StereoPair(frame_l, frame_r), usec_timestamp

    def get_framerate(self, camera: CameraIndex):
        profile = self._get_video_stream_profile(camera)
        return profile.fps()

    def get_resolution(self, camera: CameraIndex) -> Size:
        profile = self._get_video_stream_profile(camera)
        return Size(width=profile.width(), height=profile.height())

    def get_calibration_matrix(self, camera: CameraIndex):
        i = self._get_video_stream_profile(camera).get_intrinsics()
        return np.array([
            [i.fx, 0, i.ppx],
            [0, i.fy, i.ppy],
            [0, 0, 1]
        ])

    def get_distortion(self, camera: CameraIndex):
        d = self._get_video_stream_profile(camera).get_intrinsics().coeffs
        return np.array([d])

    def get_pose(self):
        selection = self._pipe.get_active_profile()
        left_stream = selection.get_stream(rs2.stream.infrared, CameraIndex.LEFT)
        right_stream = selection.get_stream(rs2.stream.infrared, CameraIndex.RIGHT)
        e = left_stream.get_extrinsics_to(right_stream)
        R = np.asarray(e.rotation).reshape((3, 3))
        t = np.asarray(e.translation)

        return R, t

    def _get_video_stream_profile(self, camera: CameraIndex):
        return self._pipe.get_active_profile().get_stream(rs2.stream.infrared, camera).as_video_stream_profile()
