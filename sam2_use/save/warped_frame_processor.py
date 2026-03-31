import cv2
import numpy as np


class WarpedFrameProcessor:
    def __init__(
        self,
        aruco_dict_type=cv2.aruco.DICT_4X4_50,
        aruco_ids_corner=None,
        real_w=64,
        real_h=92.05,
        ratio_wh=20
    ):
        """
        与原代码参数完全一致的 warped_frame 处理类
        """

        # ---------- ArUco ----------
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict_type)

        if hasattr(cv2.aruco, "DetectorParameters_create"):
            self.aruco_params = cv2.aruco.DetectorParameters_create()
        else:
            self.aruco_params = cv2.aruco.DetectorParameters()

        if hasattr(cv2.aruco, "ArucoDetector"):
            self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        else:
            self.detector = None

        # ---------- ArUco ID ----------
        self.ARUCO_IDS_CORNER = aruco_ids_corner or {
            'top_left': 0,
            'top_right': 8,
            'bottom_right': 24,
            'bottom_left': 16
        }

        # ---------- 输出尺寸 ----------
        self.REAL_W = real_w
        self.REAL_H = real_h
        self.RATIO_WH = ratio_wh

        self.OUTPUT_W = int(self.REAL_W * self.RATIO_WH)
        self.OUTPUT_H = int(self.REAL_H * self.RATIO_WH)

        self.dst_points = np.float32([
            [0, 0],
            [self.OUTPUT_W - 1, 0],
            [self.OUTPUT_W - 1, self.OUTPUT_H - 1],
            [0, self.OUTPUT_H - 1]
        ])

        # ---------- 状态 ----------
        self.perspective_matrix = None
        self.is_initialized = False

    # =========================================================
    # ArUco 检测（与你原 detect_aruco_corners 完全一致）
    # =========================================================
    def _detect_aruco_corners(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        if self.detector is not None:
            corners_list, ids, _ = self.detector.detectMarkers(gray)
        else:
            corners_list, ids, _ = cv2.aruco.detectMarkers(
                gray, self.aruco_dict, parameters=self.aruco_params
            )

        if ids is None:
            return None

        ids = ids.flatten()
        id_to_corners = {
            id_val: corner.reshape(4, 2)
            for corner, id_val in zip(corners_list, ids)
        }

        try:
            tl = id_to_corners[self.ARUCO_IDS_CORNER['top_left']]
            tr = id_to_corners[self.ARUCO_IDS_CORNER['top_right']]
            br = id_to_corners[self.ARUCO_IDS_CORNER['bottom_right']]
            bl = id_to_corners[self.ARUCO_IDS_CORNER['bottom_left']]

            src_pts = np.float32([
                tl[2],  # 左上 ArUco 的右下角
                tr[3],  # 右上 ArUco 的左下角
                br[0],  # 右下 ArUco 的左上角
                bl[1]   # 左下 ArUco 的右上角
            ])
            return src_pts

        except KeyError:
            return None

    # =========================================================
    # 对外接口：输入 frame → 输出 warped_frame
    # =========================================================
    def get_warped_frame(self, frame):
        """
        输入一帧 BGR frame
        返回 warped_frame（或 None）
        """

        # 尚未初始化：尝试检测 ArUco
        if not self.is_initialized:
            src_pts = self._detect_aruco_corners(frame)
            if src_pts is None:
                return None

            self.perspective_matrix = cv2.getPerspectiveTransform(
                src_pts, self.dst_points
            )
            self.is_initialized = True
            print("[WarpedFrameProcessor] 透视矩阵初始化完成")

        # 已初始化：直接 warp
        warped_frame = cv2.warpPerspective(
            frame,
            self.perspective_matrix,
            (self.OUTPUT_W, self.OUTPUT_H)
        )

        h, w, _ = warped_frame.shape

        # 计算上下10%高度
        top_height = int(0.05 * h)
        bottom_height = int(0.05 * h)

        # 用黑色覆盖上下区域
        warped_frame[0:top_height, :] = 0  # 上10%
        warped_frame[h - bottom_height:h, :] = 0  # 下10%

        return warped_frame
