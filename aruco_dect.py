import cv2
import numpy as np
import pyrealsense2 as rs
import time
from camera_handler_jiaqin import CameraHandler


class ArUcoDetector:
    """
    用于检测 ArUco 码的类。
    """

    def __init__(self, dictionary_id=cv2.aruco.DICT_4X4_50):
        """
        初始化 ArUco 检测器。

        Args:
            dictionary_id (int): OpenCV 中预定义的 ArUco 字典 ID。
                                 例如: cv2.aruco.DICT_4X4_50, cv2.aruco.DICT_6X6_250
        """
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(dictionary_id)

        # 兼容新旧版本的 OpenCV API
        if hasattr(cv2.aruco, "DetectorParameters_create"):
            self.aruco_params = cv2.aruco.DetectorParameters_create()
        else:
            self.aruco_params = cv2.aruco.DetectorParameters()

        if hasattr(cv2.aruco, "ArucoDetector"):
            self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        else:
            self.detector = None

        # 存储上次检测的结果
        self._last_detected_ids = []
        self._last_detected_corners = []
        self._last_detected_corners_raw = []  # 新增：存储原始角点格式
        self._last_image_shape = None

    def detect(self, image):
        """
        在给定的图像上检测 ArUco 码，并更新内部状态。

        Args:
            image (numpy.ndarray): 输入的 BGR 图像。

        Returns:
            bool: 如果检测成功则返回 True，否则返回 False。
                  即使没有检测到任何标记也算作成功的操作，会返回 True，
                  但 len(self.get_ids()) 将为 0。
        """
        self._last_image_shape = image.shape
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        try:
            if self.detector is not None:
                # 使用新 API
                corners, ids, _ = self.detector.detectMarkers(gray)
            else:
                # 使用旧 API 作为备选
                corners, ids, _ = cv2.aruco.detectMarkers(
                    gray, self.aruco_dict, parameters=self.aruco_params
                )

            if ids is not None:
                # 存储原始角点格式（用于绘制）
                self._last_detected_corners_raw = corners

                # 将 IDs 和 Corners 转换为方便使用的格式
                self._last_detected_ids = ids.flatten().tolist()
                self._last_detected_corners = [corner.reshape(4, 2) for corner in corners]
            else:
                # 如果没有检测到任何标记
                self._last_detected_ids = []
                self._last_detected_corners = []
                self._last_detected_corners_raw = []

            return True
        except Exception as e:
            print(f"ArUco 检测过程中发生错误: {e}")
            self._last_detected_ids = []
            self._last_detected_corners = []
            self._last_detected_corners_raw = []
            return False

    def get_count(self):
        """
        获取检测到的 ArUco 码的数量。

        Returns:
            int: 检测到的 ArUco 码的数量。
        """
        return len(self._last_detected_ids)

    def get_ids(self):
        """
        获取检测到的所有 ArUco 码的 ID 列表。

        Returns:
            list: 包含检测到的 ArUco ID 的列表。
                  例如: [1, 5, 23]
        """
        return self._last_detected_ids.copy()  # 返回副本以防止外部修改

    def get_pixel_values(self):
        """
        获取检测到的所有 ArUco 码的像素坐标。

        这里返回的是每个 ArUco 码的四个角点的坐标。

        Returns:
            dict: 一个字典，其中 key 是 ArUco ID，value 是一个包含该 ID 的 ArUco
                  四个角点坐标的 numpy 数组。
                  角点顺序通常为 [左上, 右上, 右下, 左下]。

                  例如:
                  {
                      1: array([[x1, y1], [x2, y2], [x3, y3], [x4, y4]]),
                      5: array([[x5, y5], [x6, y6], [x7, y7], [x8, y8]])
                  }
        """
        if not self._last_detected_ids or not self._last_detected_corners:
            return {}

        pixel_coords = {}
        for id_val, corners in zip(self._last_detected_ids, self._last_detected_corners):
            pixel_coords[id_val] = corners
        return pixel_coords

    def get_centers(self):
        """
        获取检测到的所有 ArUco 码的中心点像素坐标。

        Returns:
            dict: 一个字典，其中 key 是 ArUco ID，value 是一个包含该 ID 的 ArUco
                  中心点坐标的元组 (x, y)。

                  例如:
                  {1: (x_center_1, y_center_1), 5: (x_center_5, y_center_5)}
        """
        centers = {}
        all_corners = self.get_pixel_values()
        for id_val, corners in all_corners.items():
            # 计算四个角点的平均值作为中心
            center_x = int(np.mean(corners[:, 0]))
            center_y = int(np.mean(corners[:, 1]))
            centers[id_val] = (center_x, center_y)
        return centers

    def draw_markers(self, image, ids_to_highlight=None, highlight_color=(0, 255, 0),
                     border_thickness=3, corner_thickness=2):
        """
        将检测到的 ArUco 码绘制在图像上。

        Args:
            image (numpy.ndarray): 要在其上绘制的 BGR 图像。
            ids_to_highlight (list, optional): 一个要特别高亮的 ID 列表。
            highlight_color (tuple, optional): 高亮颜色 (B, G, R)。
            border_thickness (int): 边框粗细（像素）
            corner_thickness (int): 角点标记粗细（像素）

        Returns:
            numpy.ndarray: 绘制后的图像。
        """
        if self._last_detected_ids and self._last_detected_corners_raw:
            # 自定义绘制每个标记
            for i, (corners, id_val) in enumerate(zip(self._last_detected_corners_raw,
                                                      self._last_detected_ids)):
                # corners 形状为 (1, 4, 2)
                corner_points = corners[0].astype(int)

                # 确定颜色（如果是高亮标记，使用高亮颜色，否则使用默认绿色）
                if ids_to_highlight and id_val in ids_to_highlight:
                    color = highlight_color
                else:
                    color = (0, 255, 0)  # 默认绿色

                # 绘制边框（四边形）
                cv2.polylines(image, [corner_points], True, color, border_thickness)

                # 绘制四个角点的小圆，使角点更明显
                for point in corner_points:
                    cv2.circle(image, tuple(point), corner_thickness * 2, color, -1)

                # 在标记中心绘制 ID
                center_x = int(np.mean(corner_points[:, 0]))
                center_y = int(np.mean(corner_points[:, 1]))
                cv2.putText(image, str(id_val), (center_x - 10, center_y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        return image


def draw_info_on_image(image, fps, resolution):
    """
    在图像左上角绘制分辨率和FPS信息

    Args:
        image (numpy.ndarray): 输入的BGR图像
        fps (float): 当前帧率
        resolution (tuple): 图像分辨率 (宽度, 高度)

    Returns:
        numpy.ndarray: 绘制了信息的图像
    """
    # 设置文本样式
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.8
    font_thickness = 2
    text_color = (255, 255, 255)  # 白色
    bg_color = (0, 0, 0)  # 黑色背景
    padding = 5  # 文本背景填充

    # 准备文本内容
    res_text = f"Resolution: {resolution[0]}x{resolution[1]}"
    fps_text = f"FPS: {fps:.2f}"

    # 获取文本尺寸
    res_text_size = cv2.getTextSize(res_text, font, font_scale, font_thickness)[0]
    fps_text_size = cv2.getTextSize(fps_text, font, font_scale, font_thickness)[0]

    # 计算文本背景框位置 (分辨率文本在上，FPS文本在下)
    # 分辨率文本背景框
    res_bg_x1 = 10
    res_bg_y1 = 10
    res_bg_x2 = res_bg_x1 + res_text_size[0] + 2 * padding
    res_bg_y2 = res_bg_y1 + res_text_size[1] + 2 * padding

    # FPS文本背景框
    fps_bg_x1 = 10
    fps_bg_y1 = res_bg_y2 + 5  # 与上一个文本间隔5像素
    fps_bg_x2 = fps_bg_x1 + fps_text_size[0] + 2 * padding
    fps_bg_y2 = fps_bg_y1 + fps_text_size[1] + 2 * padding

    # 绘制背景框
    cv2.rectangle(image, (res_bg_x1, res_bg_y1), (res_bg_x2, res_bg_y2), bg_color, -1)
    cv2.rectangle(image, (fps_bg_x1, fps_bg_y1), (fps_bg_x2, fps_bg_y2), bg_color, -1)

    # 绘制文本
    cv2.putText(image, res_text, (res_bg_x1 + padding, res_bg_y2 - padding),
                font, font_scale, text_color, font_thickness)
    cv2.putText(image, fps_text, (fps_bg_x1 + padding, fps_bg_y2 - padding),
                font, font_scale, text_color, font_thickness)

    return image


# --- 使用示例 ---
if __name__ == "__main__":

    # 初始化摄像头
    IS_REAL_SENSE = False
    CAMERA_NO = 0  # TODO: 替换为你的设备序列号

    if IS_REAL_SENSE:
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 1920, 1080, rs.format.bgr8, 30)
        config.enable_device(CAMERA_NO)
        pipeline.start(config)
    else:
        camera = CameraHandler(
            camera_id=CAMERA_NO,
            width=1920,
            height=1080,
            fps=30,
            warmup_frames=30
        )

    # 创建 ArUco 检测器实例
    # 注意：如果你的标记是 DICT_4X4_100，请在此处指定
    detector = ArUcoDetector(dictionary_id=cv2.aruco.DICT_4X4_100)

    # 初始化FPS计算变量
    fps_counter = 0
    fps_start_time = time.time()
    current_fps = 0.0

    print("启动摄像头，按 'q' 退出...")

    try:
        while True:
            # 读取帧
            if IS_REAL_SENSE:
                frames = pipeline.wait_for_frames()
                frame = frames.get_color_frame()
                if not frame:
                    continue
                frame = np.asanyarray(frame.get_data())
            else:
                frame = camera.get_frame_directly()

            # 执行检测
            success = detector.detect(frame)

            if success:
                count = detector.get_count()
                ids = detector.get_ids()
                centers = detector.get_centers()  # 或者使用 get_pixel_values() 获取四个角点

                # 打印检测信息
                if count > 0:
                    print(f"\n--- 检测到 {count} 个 ArUco 码 ---")
                    for aruco_id in ids:
                        center_point = centers.get(aruco_id, (None, None))
                        print(f"ID: {aruco_id}, 中心坐标: {center_point}")

                # 绘制检测结果
                frame = detector.draw_markers(frame)

            # 计算FPS
            fps_counter += 1
            elapsed_time = time.time() - fps_start_time
            if elapsed_time >= 1.0:  # 每秒更新一次FPS
                current_fps = fps_counter / elapsed_time
                fps_counter = 0
                fps_start_time = time.time()

            # 获取图像分辨率
            height, width = frame.shape[:2]
            resolution = (width, height)

            # 在图像上绘制分辨率和FPS信息
            frame = draw_info_on_image(frame, current_fps, resolution)

            # 显示图像
            frame = cv2.resize(frame, (1920*2//3, 1080*2//3))

            # cv2.namedWindow('ArUco Detection', cv2.WINDOW_NORMAL)
            cv2.imshow('ArUco Detection', frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        if IS_REAL_SENSE:
            pipeline.stop()
        else:
            camera.stop()
        cv2.destroyAllWindows()
        print("程序已退出。")