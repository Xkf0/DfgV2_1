# camera_handler.py
import cv2
import numpy as np
import pyrealsense2 as rs
import threading
import time


class CameraHandler:
    """相机处理类，支持RealSense和普通USB相机"""

    def __init__(self, is_real_sense=True, camera_no='211622062803', warmup_frames=30):
        """
        初始化相机

        Args:
            is_real_sense: bool, 是否使用RealSense相机
            camera_no: str/int, RealSense序列号或USB相机ID
            warmup_frames: int, 预热帧数，默认30帧
        """
        self.is_real_sense = is_real_sense
        self.camera_no = camera_no
        self.warmup_frames = warmup_frames

        self.pipeline = None
        self.cap = None
        self.latest_frame = None
        self.frame_lock = threading.Lock()
        self.is_running = False
        self.capture_thread = None
        self.frame_count = 0

        self._initialize_camera()
        self.detect_mode = 1

    def _initialize_camera(self):
        """初始化相机设备"""
        if self.is_real_sense:
            self._init_realsense()
        else:
            self._init_usb_camera()

    def _init_realsense(self):
        """初始化RealSense相机"""
        try:
            self.pipeline = rs.pipeline()
            config = rs.config()
            config.enable_stream(rs.stream.color, 1920, 1080, rs.format.bgr8, 30)
            config.enable_device(self.camera_no)
            self.pipeline.start(config)
            print(f"RealSense相机 {self.camera_no} 初始化成功")

            # 预热阶段：丢弃前N帧
            print(f"相机预热中，丢弃前 {self.warmup_frames} 帧...")
            for i in range(self.warmup_frames):
                frames = self.pipeline.wait_for_frames()
                if (i + 1) % 10 == 0:
                    print(f"预热进度: {i + 1}/{self.warmup_frames}")
            print("相机预热完成")

        except Exception as e:
            print(f"RealSense相机初始化失败: {e}")
            raise

    def set_detect_mode(self, mode):
        self.detect_mode = mode

    def _init_usb_camera(self):
        """初始化USB相机"""
        try:
            camera_id = int(self.camera_no)
            self.cap = cv2.VideoCapture(camera_id)

            if not self.cap.isOpened():
                raise RuntimeError(f"无法打开USB相机 {camera_id}")

            print(f"USB相机 {camera_id} 初始化成功")

            # 预热阶段：丢弃前N帧
            print(f"相机预热中，丢弃前 {self.warmup_frames} 帧...")
            for i in range(self.warmup_frames):
                ret, _ = self.cap.read()
                if not ret:
                    raise RuntimeError("预热阶段读取帧失败")
                if (i + 1) % 10 == 0:
                    print(f"预热进度: {i + 1}/{self.warmup_frames}")
            print("相机预热完成")

        except Exception as e:
            print(f"USB相机初始化失败: {e}")
            raise

    def _capture_loop(self):
        """后台持续捕获帧的线程函数"""
        while self.is_running:
            try:
                if self.is_real_sense:
                    frames = self.pipeline.wait_for_frames()
                    frame = np.asanyarray(frames.get_color_frame().get_data())
                else:
                    ret, frame = self.cap.read()
                    if not ret:
                        print("读取USB相机帧失败")
                        time.sleep(0.01)
                        continue

                with self.frame_lock:
                    self.latest_frame = frame.copy()
                    self.frame_count += 1

            except Exception as e:
                print(f"捕获帧异常: {e}")
                time.sleep(0.01)

    def start_capture(self):
        """启动后台捕获线程"""
        if not self.is_running:
            self.is_running = True
            self.capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
            self.capture_thread.start()
            print("后台捕获线程已启动")

    def get_frame(self):
        """
        获取最新的一帧

        Returns:
            numpy.ndarray: 最新的图像帧，如果没有可用帧则返回None
        """
        if self.is_real_sense:
            # RealSense直接读取
            try:
                frames = self.pipeline.wait_for_frames()
                frame = np.asanyarray(frames.get_color_frame().get_data())
                self.frame_count += 1

                if self.detect_mode == 2:
                    # 去除绿色通道
                    frame[:, :, 1] = frame[:, :, 2]
                elif self.detect_mode == 3:
                    frame[:, :, 1] = frame[:, :, 0]
                elif self.detect_mode == 3:
                    frame[:, :, 1] = 0.6 * (frame[:, :, 0] + frame[:, :, 2])

                return frame
            except Exception as e:
                print(f"RealSense读取帧失败: {e}")
                return None
        else:
            # USB相机直接读取
            ret, frame = self.cap.read()
            if ret:
                self.frame_count += 1
                return frame
            else:
                print("USB相机读取帧失败")
                return None

    def get_latest_frame_threaded(self):
        """
        从后台线程获取最新帧（需要先调用start_capture）

        Returns:
            numpy.ndarray: 最新的图像帧，如果没有可用帧则返回None
        """
        with self.frame_lock:
            if self.latest_frame is not None:
                return self.latest_frame.copy()
            return None

    def get_frame_count(self):
        """获取已捕获的帧数"""
        return self.frame_count

    def is_ready(self):
        """检查相机是否就绪"""
        if self.is_real_sense:
            return self.pipeline is not None
        else:
            return self.cap is not None and self.cap.isOpened()

    def stop(self):
        """停止相机并释放资源"""
        print("正在停止相机...")

        # 停止后台线程
        if self.is_running:
            self.is_running = False
            if self.capture_thread is not None:
                self.capture_thread.join(timeout=2.0)

        # 释放相机资源
        if self.is_real_sense and self.pipeline is not None:
            self.pipeline.stop()
            print("RealSense相机已停止")
        elif not self.is_real_sense and self.cap is not None:
            self.cap.release()
            print("USB相机已释放")

        self.latest_frame = None
        print("相机资源已清理")

    def __enter__(self):
        """支持with语句"""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """支持with语句，自动清理"""
        self.stop()

    def __del__(self):
        """析构函数，确保资源释放"""
        self.stop()
