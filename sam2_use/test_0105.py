import time
import cv2
import numpy as np
import torch

from sam2.build_sam import build_sam2
from sam2.sam2_image_predictor import SAM2ImagePredictor
from camera_handler import CameraHandler
from warped_frame_processor import WarpedFrameProcessor


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

        # OpenCV 背景减除（速度快）
        self.back_sub = cv2.createBackgroundSubtractorMOG2(
            history=500,
            varThreshold=50,
            detectShadows=False
        )

        self.kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (5, 5)
        )

    def process_frame(self, frame):
        # 1. 背景减除，快速找运动区域
        fg_mask = self.back_sub.apply(frame)

        fg_mask = cv2.morphologyEx(fg_mask, cv2.MORPH_OPEN, self.kernel)
        fg_mask = cv2.morphologyEx(fg_mask, cv2.MORPH_DILATE, self.kernel)

        contours, _ = cv2.findContours(
            fg_mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        prompts = []
        for cnt in contours:
            if cv2.contourArea(cnt) > 500:
                x, y, w, h = cv2.boundingRect(cnt)
                cx = x + w / 2
                cy = y + h / 2
                prompts.append([cx, cy])

        new_objects_masks = []

        # 2. 有运动目标才调用 SAM2
        if len(prompts) > 0:
            self.predictor.set_image(frame)

            input_points = np.array(prompts)
            input_labels = np.ones(len(prompts), dtype=np.int32)

            masks, scores, _ = self.predictor.predict(
                point_coords=input_points,
                point_labels=input_labels,
                multimask_output=False
            )

            # masks: (N, 1, H, W)
            new_objects_masks = masks

        return frame, prompts, new_objects_masks


def main():
    detector = SAM2MotionDetector(
        model_cfg="configs/sam2.1/sam2.1_hiera_t.yaml",
        checkpoint="models/sam2.1_hiera_tiny.pt"
    )

    camera = CameraHandler(
        is_real_sense=True,
        camera_no="204222061636"
    )

    # 透视变换器
    warp_processor = WarpedFrameProcessor()

    RED = np.array([0, 0, 255], dtype=np.uint8)
    alpha = 0.4  # 红色透明度

    while True:
        frame = camera.get_frame()
        if frame is None:
            break

        t_start = time.time()

        frame = warp_processor.get_warped_frame(frame)

        processed_frame, prompts, masks = detector.process_frame(frame)

        display_frame = processed_frame.copy()

        # ---- 画统一红色 Mask ----
        if len(masks) > 0:
            for i, mask in enumerate(masks):
                m = mask.squeeze().astype(bool)

                display_frame[m] = (
                    display_frame[m].astype(np.float32) * (1 - alpha)
                    + RED.astype(np.float32) * alpha
                ).astype(np.uint8)

                # 提示点（红色实心圆）
                pt = prompts[i]
                cv2.circle(
                    display_frame,
                    (int(pt[0]), int(pt[1])),
                    30,
                    (0, 0, 255),
                    -1
                )

        t_end = time.time()
        fps = 1.0 / max(t_end - t_start, 1e-6)

        cv2.putText(
            display_frame,
            f"FPS: {fps:.2f}",
            (10, 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            2,
            (0, 255, 0),
            2
        )

        h, w = display_frame.shape[:2]
        display_frame = cv2.resize(display_frame, (w // 4, h // 4))

        cv2.imshow("Hybrid Detection (Red Mask)", display_frame)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    camera.stop()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
