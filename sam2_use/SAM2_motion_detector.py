import cv2
import numpy as np
import torch

from sam2_use.sam2.build_sam import build_sam2
from sam2_use.sam2.sam2_image_predictor import SAM2ImagePredictor
from config_loader import Configer
from fairino2_8 import (CONFIG)
cfg_1 = Configer(**CONFIG["CONFIG_PARAMS_1"])
from logger import LOG_DEBUG, LOG_INFO, LOG_WARN, LOG_ERROR, LOG_CRITICAL
import os


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

    def process_frame_jiaqin(self, frame, img_filename, is_static):
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

        foldname = f"/home/xf/imgs/{img_filename}/fg_mask"
        os.makedirs(foldname, exist_ok=True)
        from datetime import datetime
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S.%f")[:-3]
        filename = f"{foldname}/{timestamp}.png"
        cv2.imwrite(filename, fg_mask)


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

        movingPointExist = False
        # 2. 有运动目标才调用 SAM2
        if is_static:
            self.predictor.set_image(frame)

            input_points = [(85*10, 41.1*10)]
            input_labels = [1]

            masks, scores, _ = self.predictor.predict_2(
                point_coords=input_points,
                point_labels=input_labels,
                multimask_output=False
            )

            # masks: (N, 1, H, W)
            new_objects_masks = masks
        else:
            if len(prompts) > 0:
                # SAM2 使用原始清晰图像，而不是模糊后的图像
                self.predictor.set_image(frame)

                input_points = np.array(prompts)
                input_labels = np.ones(len(prompts), dtype=np.int32)

                masks, scores, _ = self.predictor.predict_2(
                    point_coords=input_points,
                    point_labels=input_labels,
                    multimask_output=False
                )

                # masks: (N, 1, H, W)
                new_objects_masks = masks
                movingPointExist = True
                
        # print('22222222222222222222')
        # print(f"masks:\n{new_objects_masks}")

        # return frame, prompts, new_objects_masks
        return new_objects_masks, movingPointExist
    
    def process_frame_wanqi(self, frame, img_filename):
        """
        基于背景建模粗检前景，再生成更稳的 SAM 提示点，最后用 SAM 输出单通道二值 mask。
        重点优化：前景点 prompts 的生成过程。
        返回:
            final_mask: np.ndarray, shape=(H, W), dtype=uint8, 前景=255, 背景=0
        """
        import math
        import numpy as np
        import cv2

        def _filter_small_components(binary_mask, min_area):
            """去除小连通域"""
            num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(binary_mask, connectivity=8)
            filtered = np.zeros_like(binary_mask)
            for i in range(1, num_labels):  # 0 是背景
                area = stats[i, cv2.CC_STAT_AREA]
                if area >= min_area:
                    filtered[labels == i] = 255
            return filtered

        def _contour_centroid(cnt):
            """轮廓质心，失败时退化为 bbox 中心"""
            m = cv2.moments(cnt)
            if abs(m["m00"]) > 1e-6:
                return [m["m10"] / m["m00"], m["m01"] / m["m00"]]
            x, y, w, h = cv2.boundingRect(cnt)
            return [x + w / 2.0, y + h / 2.0]

        def _clip_point(pt, w, h, margin=2):
            x = min(max(float(pt[0]), margin), w - margin - 1)
            y = min(max(float(pt[1]), margin), h - margin - 1)
            return [x, y]

        def _deduplicate_points(points, min_dist=20.0):
            """对距离过近的提示点去重"""
            if not points:
                return []

            kept = []
            min_dist2 = min_dist * min_dist
            for p in points:
                keep = True
                for q in kept:
                    dx = p[0] - q[0]
                    dy = p[1] - q[1]
                    if dx * dx + dy * dy < min_dist2:
                        keep = False
                        break
                if keep:
                    kept.append(p)
            return kept

        def _extract_prompt_points_from_contour(cnt, h_img, w_img):
            """
            从单个轮廓生成更稳的 prompt 点。
            返回若干点:
            - 普通目标: 1 个质心点
            - 细长/长条目标: 3 个点（中心 + 主轴两侧）
            """
            points = []

            area = cv2.contourArea(cnt)
            if area < 500:
                return points

            x, y, w, h = cv2.boundingRect(cnt)

            # 1) 边界过滤：紧贴边界的通常是伪影
            if x < 5 or y < 5 or (x + w) > w_img - 5 or (y + h) > h_img - 5:
                return points

            # 2) 形状过滤
            rect_area = float(w * h)
            if rect_area <= 1:
                return points

            fill_ratio = area / rect_area
            aspect_ratio = max(w, h) / max(1.0, min(w, h))

            # 过于稀疏/过薄的伪轮廓不要
            if fill_ratio < 0.12:
                return points
            if min(w, h) < 8:
                return points

            # 基础中心点：轮廓质心
            c = _contour_centroid(cnt)
            c = _clip_point(c, w_img, h_img)
            points.append(c)

            # 3) 对细长目标，增加主轴方向辅助点
            # 适合布料、条状、倾斜目标，避免只打一中心点导致 SAM 只吃一部分
            if aspect_ratio >= 2.2 and area >= 900:
                rect = cv2.minAreaRect(cnt)
                (cx, cy), (rw, rh), angle = rect

                long_len = max(rw, rh)
                short_len = min(rw, rh)

                # 主轴方向角
                # OpenCV 的 minAreaRect 角度定义略特殊，这里统一转换
                if rw >= rh:
                    theta = math.radians(angle)
                else:
                    theta = math.radians(angle + 90.0)

                ux = math.cos(theta)
                uy = math.sin(theta)

                # 沿主轴两侧偏移，距离不要太大，避免跑出目标
                offset = max(8.0, min(long_len * 0.22, 28.0))

                p1 = [cx + ux * offset, cy + uy * offset]
                p2 = [cx - ux * offset, cy - uy * offset]

                p1 = _clip_point(p1, w_img, h_img)
                p2 = _clip_point(p2, w_img, h_img)

                points.append(p1)
                points.append(p2)

            return points

        # -------------------------
        # 0. 基础信息
        # -------------------------
        h_img, w_img = frame.shape[:2]

        # -------------------------
        # 1. 预处理
        # -------------------------
        # 不要模糊太重，否则小目标/细边会被抹掉
        blurred_frame = cv2.GaussianBlur(frame, (5, 5), 0)

        # -------------------------
        # 2. MOG2 粗前景
        # -------------------------
        fg_mask = self.back_sub.apply(blurred_frame)

        # MOG2: 背景=0, 阴影=127, 前景=255
        # 这里必须真正滤掉阴影，所以阈值应高于127
        _, fg_mask = cv2.threshold(fg_mask, 200, 255, cv2.THRESH_BINARY)

        # -------------------------
        # 3. 稳定粗前景区域
        # -------------------------
        fg_mask = cv2.morphologyEx(fg_mask, cv2.MORPH_CLOSE, self.kernel, iterations=1)
        fg_mask = cv2.morphologyEx(fg_mask, cv2.MORPH_OPEN, self.kernel, iterations=1)
        fg_mask = cv2.dilate(fg_mask, self.kernel, iterations=1)

        # 连通域过滤，去掉碎块
        fg_mask = _filter_small_components(fg_mask, min_area=400)

        foldname = f"/home/xf/imgs/{img_filename}/fg_mask"
        os.makedirs(foldname, exist_ok=True)
        from datetime import datetime
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S.%f")[:-3]
        filename = f"{foldname}/{timestamp}.png"
        cv2.imwrite(filename, fg_mask)

        # -------------------------
        # 4. 从粗前景轮廓中提取 prompts（重点优化部分）
        # -------------------------
        contours, _ = cv2.findContours(
            fg_mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        prompts = []

        for cnt in contours:
            pts = _extract_prompt_points_from_contour(cnt, h_img, w_img)
            if pts:
                prompts.extend(pts)

        # 去重，避免一个目标给出过多相近点
        prompts = _deduplicate_points(prompts, min_dist=18.0)

        # 测试点保留
        if CONFIG["testStaticFabricLength"]:
            prompts.append([20 * CONFIG["test_x_cm"], 20 * CONFIG["test_y_cm"]])
            prompts.append([20 * (CONFIG["test_x_cm"] + 5), 20 * (CONFIG["test_y_cm"] + 5)])
            prompts.append([20 * (CONFIG["test_x_cm"] + 5), 20 * (CONFIG["test_y_cm"] - 5)])
            prompts.append([20 * (CONFIG["test_x_cm"] - 5), 20 * (CONFIG["test_y_cm"] - 5)])
            prompts.append([20 * (CONFIG["test_x_cm"] - 5), 20 * (CONFIG["test_y_cm"] + 5)])

        # -------------------------
        # 5. 默认输出
        # -------------------------
        final_mask = np.zeros((h_img, w_img), dtype=np.uint8)

        # -------------------------
        # 6. SAM 必须使用：只要有 prompts 就调用
        # -------------------------
        if len(prompts) > 0:
            self.predictor.set_image(frame)

            input_points = np.array(prompts, dtype=np.float32)
            input_labels = np.ones(len(prompts), dtype=np.int32)

            masks, scores, _ = self.predictor.predict_2(
                point_coords=input_points,
                point_labels=input_labels,
                multimask_output=False
            )

            if masks is not None:
                masks_np = np.asarray(masks)

                # 兼容不同输出维度
                if masks_np.ndim == 4:
                    # (N,1,H,W)
                    masks_np = masks_np[:, 0, :, :]
                elif masks_np.ndim == 2:
                    # (H,W)
                    masks_np = masks_np[None, :, :]

                # union 成一张图
                for i in range(masks_np.shape[0]):
                    one_mask = (masks_np[i] > 0).astype(np.uint8) * 255
                    final_mask = cv2.bitwise_or(final_mask, one_mask)

        # -------------------------
        # 7. 对 SAM 输出做轻微清理
        # -------------------------
        final_mask = cv2.morphologyEx(final_mask, cv2.MORPH_CLOSE, self.kernel, iterations=1)
        final_mask = cv2.morphologyEx(final_mask, cv2.MORPH_OPEN, self.kernel, iterations=1)
        final_mask = _filter_small_components(final_mask, min_area=300)

        return final_mask
    
    def process_frame_xiefan(self, frame, img_filename, centroids):
        # ---- 修改 2: 预处理 - 高斯模糊 ----
        # 在检测运动之前先模糊，可以平滑掉图像中的高频噪点和微小光线抖动
        # (21, 21) 是模糊核大小，必须是奇数。越大越模糊，抗噪越强。
        prompts = []

        for (x, y, _) in centroids:
            prompts.append([x, y])

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