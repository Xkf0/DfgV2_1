import time
import cv2
import numpy as np
import torch

from camera_handler import CameraHandler
from warped_frame_processor import WarpedFrameProcessor
from SAM2_motion_detector import SAM2MotionDetector



def main():

    # 检测器
    detector = SAM2MotionDetector(
        model_cfg="configs/sam2.1/sam2.1_hiera_t.yaml",
        checkpoint="models/sam2.1_hiera_tiny.pt"
    )

    # 读帧器
    camera = CameraHandler(
        is_real_sense=True,
        camera_no="211622062803"
    )

    # 透视变换器
    warp_processor = WarpedFrameProcessor()

    RED = np.array([0, 0, 255], dtype=np.uint8)
    alpha = 0.6  # 红色透明度

    frame_idx = 0

    while True:
        frame = camera.get_frame()

        frame_idx += 1

        if frame is None:
            break

        t_start = time.time()

        frame = warp_processor.get_warped_frame(frame)

        if frame_idx == 1:
            cv2.imwrite('imgs/warped_frame.jpg', frame)

        processed_frame, prompts, masks = detector.process_frame(frame)


        #--------------形态学处理开始--------------
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2, 2))

        processed_masks = []

        for mask in masks:
            if isinstance(mask, torch.Tensor):
                m = mask.cpu().numpy().squeeze()
            else:
                m = mask.squeeze()

            # 确保是 uint8 的 0/255 二值图
            if m.dtype != np.uint8:
                m = (m > 0).astype(np.uint8) * 255

            m = cv2.morphologyEx(m, cv2.MORPH_OPEN, kernel, iterations=1)
            m = cv2.morphologyEx(m, cv2.MORPH_DILATE, kernel, iterations=1)

            processed_masks.append(m)

        masks = processed_masks
        # --------------形态学处理结束--------------

        # print(f'prompts:{len(prompts)}, masks:{len(masks)}')

        display_frame = processed_frame.copy()

        # ---- 画统一红色 Mask ----
        if len(masks) > 0:
            for i, mask in enumerate(masks):
                # 确保 mask 维度正确
                if isinstance(mask, torch.Tensor):
                    m = mask.cpu().numpy().squeeze().astype(bool)
                else:
                    m = mask.squeeze().astype(bool)

                # 防止 mask 尺寸不匹配（极端情况）
                if m.shape[:2] == display_frame.shape[:2]:
                    display_frame[m] = (
                            display_frame[m].astype(np.float32) * (1 - alpha)
                            + RED.astype(np.float32) * alpha
                    ).astype(np.uint8)

        # 提示点（红色实心圆）
        for i in range(len(prompts)):
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
        # 显示缩放，减少屏幕占用
        display_frame = cv2.resize(display_frame, (w // 4, h // 4))

        cv2.imwrite('imgs/combined1.jpg', display_frame)
        # cv2.imshow("Hybrid Detection (Red Mask)", display_frame)

        # if cv2.waitKey(1) & 0xFF == ord("q"):
        #     break

    camera.stop()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()