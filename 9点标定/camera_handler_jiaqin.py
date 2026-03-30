import cv2
import numpy as np
import threading
import time
from typing import Optional, Tuple


class CameraHandler:
    """
    MJPG 格式 USB 相机处理类（Linux 专用）
    特性：
    1. 强制使用 MJPG 编码格式，提升 1080P 帧率
    2. 支持后台线程持续捕获帧，避免主线程阻塞
    3. 相机预热机制，丢弃初始不稳定帧
    4. 完善的资源管理和异常处理
    """

    def __init__(
        self,
        camera_id: int = 0,
        width: int = 1920,
        height: int = 1080,
        fps: int = 30,
        warmup_frames: int = 30
    ):
        """
        初始化 MJPG 格式 USB 相机

        Args:
            camera_id: USB 相机设备ID（默认 0，对应 /dev/video0）
            width: 帧宽度（默认 1920）
            height: 帧高度（默认 1080）
            fps: 目标帧率（默认 30，需相机硬件支持）
            warmup_frames: 预热帧数，丢弃前 N 帧（默认 30）
        """
        # 相机基础参数
        self.camera_id = camera_id
        self.width = width
        self.height = height
        self.fps = fps
        self.warmup_frames = warmup_frames

        # MJPG 编码格式（固定）
        self.fourcc = cv2.VideoWriter_fourcc(*'MJPG')

        # 相机资源与状态
        self.cap: Optional[cv2.VideoCapture] = None
        self.latest_frame: Optional[np.ndarray] = None
        self.frame_lock = threading.Lock()  # 帧数据线程锁
        self.is_running = False
        self.capture_thread: Optional[threading.Thread] = None
        self.frame_count = 0

        # 初始化相机
        self._initialize_camera()

    def _initialize_camera(self) -> None:
        """初始化 USB 相机（强制 MJPG 格式 + 1080P + 指定帧率）"""
        try:
            # Linux 专用 V4L2 后端，避免 GStreamer 问题
            self.cap = cv2.VideoCapture(self.camera_id, cv2.CAP_V4L2)
            if not self.cap.isOpened():
                raise RuntimeError(f"无法打开 USB 相机（ID: {self.camera_id}）")

            # 1. 设置 MJPG 编码格式（核心！节省带宽提升帧率）
            self.cap.set(cv2.CAP_PROP_FOURCC, self.fourcc)
            # 2. 设置分辨率
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            # 3. 设置帧率
            self.cap.set(cv2.CAP_PROP_FPS, self.fps)

            # 验证实际生效的参数
            self._verify_camera_params()

            # 相机预热：丢弃前 N 帧（初始帧可能模糊/不稳定）
            self._warmup_camera()

            print(f" USB 相机初始化成功（ID: {self.camera_id}）")

        except Exception as e:
            self.stop()  # 初始化失败时释放资源
            raise RuntimeError(f"相机初始化失败: {str(e)}") from e

    def _verify_camera_params(self) -> None:
        """验证相机实际生效的参数（硬件可能自动降级）"""
        actual_fourcc = int(self.cap.get(cv2.CAP_PROP_FOURCC))
        actual_fourcc_str = "".join([chr((actual_fourcc >> 8 * i) & 0xFF) for i in range(4)])
        actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = self.cap.get(cv2.CAP_PROP_FPS)

        print("\n 相机实际生效参数：")
        print(f"  编码格式: {actual_fourcc_str} (目标: MJPG)")
        print(f"  分辨率: {actual_width}x{actual_height} (目标: {self.width}x{self.height})")
        print(f"  帧率: {actual_fps:.1f} FPS (目标: {self.fps} FPS)")

        # 警告：如果编码格式不是 MJPG，帧率可能受限
        if actual_fourcc_str != "MJPG":
            print(f"  警告：相机不支持 MJPG 格式，当前格式为 {actual_fourcc_str}，帧率可能降低")

    def _warmup_camera(self) -> None:
        """相机预热，丢弃前 N 帧"""
        print(f"\n 相机预热中，丢弃前 {self.warmup_frames} 帧...")
        for i in range(self.warmup_frames):
            ret, _ = self.cap.read()
            if not ret:
                raise RuntimeError(f"预热第 {i+1} 帧时读取失败")
            if (i + 1) % 10 == 0:
                print(f"  预热进度: {i+1}/{self.warmup_frames}")
        print("相机预热完成")

    def _capture_loop(self) -> None:
        """后台线程：持续捕获帧并更新最新帧"""
        while self.is_running:
            try:
                ret, frame = self.cap.read()
                if not ret:
                    print("  后台线程读取帧失败，重试中...")
                    time.sleep(0.01)
                    continue

                # 线程安全更新最新帧
                with self.frame_lock:
                    self.latest_frame = frame.copy()
                    self.frame_count += 1

            except Exception as e:
                print(f" 后台捕获帧异常: {str(e)}")
                time.sleep(0.01)

    def start_capture(self) -> None:
        """启动后台捕获线程（非阻塞读取帧）"""
        if not self.is_running and self.cap.isOpened():
            self.is_running = True
            self.capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
            self.capture_thread.start()
            print(" 后台捕获线程已启动")

    def get_latest_frame(self) -> Optional[np.ndarray]:
        """
        获取后台线程捕获的最新帧（线程安全）
        Returns:
            最新帧数组（BGR 格式），无可用帧时返回 None
        """
        with self.frame_lock:
            if self.latest_frame is not None:
                return self.latest_frame.copy()
        return None

    def get_frame_directly(self) -> Optional[np.ndarray]:
        """
        直接读取一帧（适用于无需后台线程的场景）
        Returns:
            帧数组（BGR 格式），读取失败返回 None
        """
        if not self.cap.isOpened():
            print(" 相机未就绪，无法直接读取帧")
            return None

        ret, frame = self.cap.read()
        if ret:
            self.frame_count += 1
            if self.frame_count == 1:
                print(f"首次读取帧尺寸: {frame.shape}")
            return frame
        else:
            print("直接读取帧失败")
            return None

    def get_frame_count(self) -> int:
        """获取已捕获的总帧数"""
        return self.frame_count

    def is_ready(self) -> bool:
        """检查相机是否就绪"""
        return self.cap is not None and self.cap.isOpened()

    def stop(self) -> None:
        """停止相机并释放所有资源"""
        print("\n 正在停止相机...")

        # 停止后台线程
        self.is_running = False
        if self.capture_thread is not None:
            self.capture_thread.join(timeout=2.0)
            self.capture_thread = None

        # 释放 VideoCapture 资源
        if self.cap is not None:
            self.cap.release()
            self.cap = None

        # 清空帧数据
        self.latest_frame = None
        self.frame_count = 0

        print(" 相机资源已全部释放")

    def __enter__(self):
        """支持 with 语句"""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """with 语句结束时自动释放资源"""
        self.stop()

    def __del__(self):
        """析构函数，确保程序退出时释放资源"""
        self.stop()


# ------------------------------ 测试示例 ------------------------------
if __name__ == "__main__":
    # 使用示例1：直接读取帧
    try:
        camera = CameraHandler(
            camera_id=0,
            width=1920,
            height=1080,
            fps=30,
            warmup_frames=30
        )

        # 方式1：直接读取帧（适用于简单场景）
        print("\n 开始直接读取帧（按 'q' 退出）")
        prev_time = 0
        while True:
            frame = camera.get_frame_directly()
            if frame is None:
                break

            # 计算并绘制实时帧率
            curr_time = time.time()
            fps = 1 / (curr_time - prev_time) if (curr_time - prev_time) > 0 else 0
            prev_time = curr_time
            cv2.putText(
                frame,
                f"FPS: {fps:.1f} | Res: {frame.shape[1]}x{frame.shape[0]}",
                (10, 50),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (0, 255, 0),
                2
            )

            frame = cv2.resize(frame,(1920//2, 1080//2))

            cv2.imshow("MJPG 1080P USB Camera (Direct Read)", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except Exception as e:
        print(f" 测试失败: {str(e)}")
    finally:
        cv2.destroyAllWindows()

    # 使用示例2：后台线程读取帧（适用于高并发场景）
    # try:
    #     with MJPGUSBCameraHandler(camera_id=0) as camera:
    #         camera.start_capture()
    #         print("\n 后台线程读取帧（按 'q' 退出）")
    #         while True:
    #             frame = camera.get_latest_frame()
    #             if frame is None:
    #                 time.sleep(0.01)
    #                 continue
    #             cv2.imshow("MJPG 1080P USB Camera (Threaded)", frame)
    #             if cv2.waitKey(1) & 0xFF == ord('q'):
    #                 break
    # except Exception as e:
    #     print(f"测试失败: {str(e)}")
    # finally:
    #     cv2.destroyAllWindows()
