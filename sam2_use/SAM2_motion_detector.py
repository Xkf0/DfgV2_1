import cv2
import numpy as np
import torch

from sam2_use.sam2.build_sam import build_sam2
from sam2_use.sam2.sam2_image_predictor import SAM2ImagePredictor
from config_loader import Configer
from fairino2_8 import (CONFIG)
cfg_1 = Configer(**CONFIG["CONFIG_PARAMS_1"])
from logger import LOG_DEBUG, LOG_INFO, LOG_WARN, LOG_ERROR, LOG_CRITICAL


class SAM2MotionDetector:
    def __init__(self, model_cfg, checkpoint, device="cuda"):
        self.device = torch.device(device)

        # 根据显卡能力选择 dtype
        if torch.cuda.is_available():
            if torch.cuda.is_bf16_supported():
                self.dtype = torch.bfloat16
            else:
                self.dtype = torch.float16
        else:
            self.dtype = torch.float32

        print("Loading SAM2 Model...")
        self.sam2_model = build_sam2(
            model_cfg,
            checkpoint,
            device=self.device
        )
        self.predictor = SAM2ImagePredictor(self.sam2_model)

        # ---- 修改 1: 优化背景减除参数 ----
        # history: 历史帧数，越大会适应得越慢（有助于忽略瞬间光影），但也可能残留拖影
        # varThreshold: 阈值，设得越高，对光照变化越不敏感（推荐 100-200）
        # detectShadows: 开启阴影检测，会把阴影过滤掉
        self.back_sub = cv2.createBackgroundSubtractorMOG2(
            history=150,
            varThreshold=150,  # 调高以抵抗噪点
            detectShadows=False
        )

        self.kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (5, 5)
        )

    def process_frame(self, frame):
        # ---- 修改 2: 预处理 - 高斯模糊 ----
        # 在检测运动之前先模糊，可以平滑掉图像中的高频噪点和微小光线抖动
        # (21, 21) 是模糊核大小，必须是奇数。越大越模糊，抗噪越强。
        blurred_frame = cv2.GaussianBlur(frame, (11, 11), 0)

        # 1. 背景减除
        fg_mask = self.back_sub.apply(blurred_frame)

        # ---- 修改 3: 过滤阴影 ----
        # MOG2 中，前景是 255，阴影是 127，背景是 0。
        # 我们使用 threshold 将 127 (阴影) 归零，只保留 255 (强移动)。
        _, fg_mask = cv2.threshold(fg_mask, 50, 255, cv2.THRESH_BINARY)


        # ---- 修改 4: 加强形态学去噪 ----
        # iterations=2 表示做两次开运算，能更彻底地去除孤立的小白点
        fg_mask = cv2.morphologyEx(fg_mask, cv2.MORPH_OPEN, self.kernel, iterations=2)
        # 膨胀，把断开的物体连起来
        fg_mask = cv2.morphologyEx(fg_mask, cv2.MORPH_DILATE, self.kernel, iterations=2)


        contours, _ = cv2.findContours(
            fg_mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        prompts = []

        # 获取图像尺寸，用于防止边缘误检
        h_img, w_img = frame.shape[:2]

        # print("00000000000000000000")

        for cnt in contours:
            # ---- 修改 5: 提高面积阈值 ----
            # 500 可能太小，容易受噪点影响。根据实际分辨率调整，例如 1000 或 1500
            area = cv2.contourArea(cnt)
            if area > 500:
                x, y, w, h = cv2.boundingRect(cnt)

                # 可选：排除紧贴屏幕边缘的检测（通常边缘会有伪影）
                if x < 5 or y < 5 or (x + w) > w_img - 5 or (y + h) > h_img - 5:
                    continue

                cx = x + w / 2
                cy = y + h / 2
                prompts.append([cx, cy])

        if CONFIG["testStaticFabricLength"]:
            prompts.append([20 * CONFIG["test_x_cm"], 20 * CONFIG["test_y_cm"]])
            prompts.append([20 * (CONFIG["test_x_cm"] + 5), 20 * (CONFIG["test_y_cm"] + 5)])
            prompts.append([20 * (CONFIG["test_x_cm"] + 5), 20 * (CONFIG["test_y_cm"] - 5)])
            prompts.append([20 * (CONFIG["test_x_cm"] - 5), 20 * (CONFIG["test_y_cm"] - 5)])
            prompts.append([20 * (CONFIG["test_x_cm"] - 5), 20 * (CONFIG["test_y_cm"] + 5)])

        # 初始化为 全黑单通道 mask，形状同当前帧
        h, w = frame.shape[:2]
        new_objects_masks = np.zeros((h, w), dtype=np.uint8)   # (1,1,H,W) 全 0

        # print('1111111111111111111111')

        # 2. 有运动目标才调用 SAM2
        if len(prompts) > 0:
        # if True:
            # SAM2 使用原始清晰图像，而不是模糊后的图像
            self.predictor.set_image(frame)

            input_points = np.array(prompts)
            input_labels = np.ones(len(prompts), dtype=np.int32)
            # input_points = [(85*10, 41.1*10)]
            # input_labels = [1]

            masks, scores, _ = self.predictor.predict_2(
                point_coords=input_points,
                point_labels=input_labels,
                multimask_output=False
            )

            # masks: (N, 1, H, W)
            new_objects_masks = masks
        # print('22222222222222222222')
        # print(f"masks:\n{new_objects_masks}")

        # return frame, prompts, new_objects_masks
        return new_objects_masks