#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# RealSense D435i Camera Device
################################################################

import numpy as np

# Try to import pyrealsense2
PYREALSENSE_AVAILABLE = False
rs = None
try:
    import pyrealsense2 as rs
    PYREALSENSE_AVAILABLE = True
except ImportError:
    # Will be caught when device is instantiated
    pass

from ..cam_base import HexCamBase
from ...zmq_base import HexSafeValue, hex_zmq_ts_now

CAMERA_CONFIG = {
    "serial_number": None,  # None for first available device
    "width": 640,
    "height": 480,
    "fps": 30,
    "enable_rgb": True,
    "enable_depth": True,
    "sens_ts": True,
    # Optional: separate depth resolution (defaults to RGB resolution if not specified)
    "depth_width": None,
    "depth_height": None,
}


class HexCamRealsense(HexCamBase):
    """RealSense D435i camera device."""

    def __init__(self, camera_config: dict = CAMERA_CONFIG):
        HexCamBase.__init__(self)

        if not PYREALSENSE_AVAILABLE:
            raise ImportError("pyrealsense2 is not installed. Install with: pip install pyrealsense2")

        try:
            self.__serial_number = camera_config.get("serial_number", None)
            self.__width = camera_config["width"]
            self.__height = camera_config["height"]
            self.__fps = camera_config["fps"]
            self.__enable_rgb = camera_config["enable_rgb"]
            self.__enable_depth = camera_config["enable_depth"]
            self.__sens_ts = camera_config["sens_ts"]
            # Optional separate depth resolution
            self.__depth_width = camera_config.get("depth_width", None) or self.__width
            self.__depth_height = camera_config.get("depth_height", None) or self.__height
        except KeyError as ke:
            missing_key = ke.args[0]
            raise ValueError(
                f"camera_config is not valid, missing key: {missing_key}")

        # RealSense variables
        self.__pipeline = None
        self.__config = None
        self.__profile = None
        self.__align = None

        # Camera intrinsics (fx, fy, cx, cy)
        self.__intri = np.zeros(4)

        # Open device
        if not self.__open_device():
            print("\033[91mFailed to open RealSense device\033[0m")
            return

        # Start work loop
        self._working.set()
        print(f"\033[92m✓ RealSense camera initialized\033[0m")

    def get_intri(self) -> np.ndarray:
        """Get camera intrinsics [fx, fy, cx, cy]."""
        self._wait_for_working()
        return self.__intri

    def get_serial_number(self) -> str:
        """Get camera serial number."""
        self._wait_for_working()
        return self.__serial_number

    def __open_device(self) -> bool:
        """Open RealSense camera and configure streams."""
        try:
            # Create pipeline
            self.__pipeline = rs.pipeline()
            self.__config = rs.config()

            # Enable specific device if serial number provided
            if self.__serial_number:
                self.__config.enable_device(self.__serial_number)

            # Configure streams
            if self.__enable_rgb:
                self.__config.enable_stream(
                    rs.stream.color,
                    self.__width,
                    self.__height,
                    rs.format.rgb8,
                    self.__fps
                )

            if self.__enable_depth:
                self.__config.enable_stream(
                    rs.stream.depth,
                    self.__depth_width,
                    self.__depth_height,
                    rs.format.z16,
                    self.__fps
                )

            # Start pipeline
            self.__profile = self.__pipeline.start(self.__config)

            # Get camera intrinsics
            if self.__enable_rgb:
                color_stream = self.__profile.get_stream(rs.stream.color)
                intrinsics = color_stream.as_video_stream_profile().get_intrinsics()

                self.__intri = np.array([
                    intrinsics.fx,
                    intrinsics.fy,
                    intrinsics.ppx,
                    intrinsics.ppy
                ])

                print(f"Camera intrinsics: fx={intrinsics.fx:.2f}, fy={intrinsics.fy:.2f}, "
                      f"cx={intrinsics.ppx:.2f}, cy={intrinsics.ppy:.2f}")

            # Create align object (align depth to color)
            if self.__enable_rgb and self.__enable_depth:
                self.__align = rs.align(rs.stream.color)

            # Get actual device serial number
            device = self.__profile.get_device()
            self.__serial_number = device.get_info(rs.camera_info.serial_number)
            print(f"RealSense device: {self.__serial_number}")

            return True

        except Exception as e:
            print(f"\033[91mFailed to open RealSense: {e}\033[0m")
            return False

    def work_loop(self, hex_values: list[HexSafeValue]):
        """Main work loop to continuously capture frames."""
        rgb_value = hex_values[0]
        depth_value = hex_values[1]

        rgb_count = 0
        depth_count = 0

        print("RealSense work loop started")

        while self._working.is_set():
            try:
                # Wait for frames
                frames = self.__pipeline.wait_for_frames(timeout_ms=1000)

                if frames:
                    # Align depth to color if both enabled
                    if self.__align:
                        frames = self.__align.process(frames)

                    # Get timestamp
                    if self.__sens_ts:
                        # Use hardware timestamp (milliseconds)
                        ts = frames.get_timestamp() / 1000.0  # Convert to seconds
                    else:
                        ts = hex_zmq_ts_now()

                    # Process RGB frame
                    if self.__enable_rgb:
                        color_frame = frames.get_color_frame()
                        if color_frame:
                            # Convert to numpy array (H, W, 3) uint8
                            rgb_image = np.asanyarray(color_frame.get_data())
                            rgb_value.set((ts, rgb_count, rgb_image))
                            rgb_count = (rgb_count + 1) % self._max_seq_num

                    # Process depth frame
                    if self.__enable_depth:
                        depth_frame = frames.get_depth_frame()
                        if depth_frame:
                            # Convert to numpy array (H, W) uint16 (millimeters)
                            depth_image = np.asanyarray(depth_frame.get_data())
                            depth_value.set((ts, depth_count, depth_image))
                            depth_count = (depth_count + 1) % self._max_seq_num

            except RuntimeError as e:
                # Timeout or no frames
                print(f"\033[93mWarning: {e}\033[0m")
                continue

            except Exception as e:
                print(f"\033[91mError in work_loop: {e}\033[0m")
                break

        print("RealSense work loop stopped")

    def close(self):
        """Stop camera and cleanup."""
        print("Closing RealSense camera...")
        if self.__pipeline:
            try:
                self.__pipeline.stop()
                print("\033[92m✓ RealSense camera closed\033[0m")
            except Exception as e:
                print(f"\033[91mError closing RealSense: {e}\033[0m")
