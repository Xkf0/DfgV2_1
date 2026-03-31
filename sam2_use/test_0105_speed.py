import time
import cv2
import numpy as np
import torch

from camera_handler import CameraHandler
from warped_frame_processor import WarpedFrameProcessor
from SAM2_motion_detector import SAM2MotionDetector


def main():
    # 初始化记录变量
    cross_1_3_recorded = False  # 是否已记录过1/3线
    cross_2_3_recorded = False  # 是否已记录过2/3线
    cross_1_3_info = {"timestamp": None, "x": None, "y": None}  # 1/3线穿越信息
    cross_2_3_info = {"timestamp": None, "x": None, "y": None}  # 2/3线穿越信息
    
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

    aruco_dict_type=cv2.aruco.DICT_4X4_50,
    aruco_ids_corner=None,
    real_w=64,
    real_h=92.05,
    ratio_wh=20

    # 透视变换器
    warp_processor = WarpedFrameProcessor(aruco_dict_type, aruco_ids_corner, real_w, real_h, ratio_wh)

    RED = np.array([0, 0, 255], dtype=np.uint8)
    alpha = 0.6  # 红色透明度
    
    # 线条颜色（绿色）和厚度
    LINE_COLOR = (0, 255, 0)
    LINE_THICKNESS = 2

    frame_idx = 0

    while True:
        # 记录读帧时间戳（关键：使用读帧时刻的时间）
        frame_read_timestamp = time.time()
        frame = camera.get_frame()

        frame_idx += 1

        if frame is None:
            break

        t_start = time.time()

        frame = warp_processor.get_warped_frame(frame)
        
        # 获取画面尺寸，计算1/3和2/3横坐标
        frame_h, frame_w = frame.shape[:2]
        line_1_3_x = int(frame_w * 1/3)
        line_2_3_x = int(frame_w * 2/3)

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
        
        # 绘制1/3和2/3分割线
        cv2.line(display_frame, (line_1_3_x, 0), (line_1_3_x, frame_h), LINE_COLOR, LINE_THICKNESS)
        cv2.line(display_frame, (line_2_3_x, 0), (line_2_3_x, frame_h), LINE_COLOR, LINE_THICKNESS)
        # 添加线条标注
        cv2.putText(display_frame, "1/3", (line_1_3_x + 10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, LINE_COLOR, 2)
        cv2.putText(display_frame, "2/3", (line_2_3_x + 10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, LINE_COLOR, 2)

        # 计算目标形心并记录穿越信息
        centroid_x, centroid_y = None, None
        if len(masks) > 0:
            # 合并所有mask（处理多目标情况）
            combined_mask = np.zeros((frame_h, frame_w), dtype=np.uint8)
            for mask in masks:
                if mask.shape[:2] == (frame_h, frame_w):
                    combined_mask = np.maximum(combined_mask, mask)
            
            # 计算形心
            moments = cv2.moments(combined_mask)
            if moments["m00"] > 0:  # 确保mask有有效区域
                centroid_x = int(moments["m10"] / moments["m00"])
                centroid_y = int(moments["m01"] / moments["m00"])
                
                # 绘制形心（蓝色实心圆）
                cv2.circle(display_frame, (centroid_x, centroid_y), 10, (255, 0, 0), -1)
                # 标注形心坐标
                cv2.putText(display_frame, f"({centroid_x},{centroid_y})", 
                           (centroid_x + 15, centroid_y), cv2.FONT_HERSHEY_SIMPLEX, 
                           0.8, (255, 0, 0), 2)
                
                # 记录首次穿越1/3线的信息
                if not cross_1_3_recorded and centroid_x >= line_1_3_x:
                    cross_1_3_info["timestamp"] = frame_read_timestamp
                    cross_1_3_info["x"] = centroid_x
                    cross_1_3_info["y"] = centroid_y
                    cross_1_3_recorded = True
                    print(f"首次穿越1/3线 - 时间戳: {cross_1_3_info['timestamp']:.6f}, 坐标: ({centroid_x}, {centroid_y})")
                
                # 记录首次穿越2/3线的信息
                if not cross_2_3_recorded and centroid_x >= line_2_3_x:
                    cross_2_3_info["timestamp"] = frame_read_timestamp
                    cross_2_3_info["x"] = centroid_x
                    cross_2_3_info["y"] = centroid_y
                    cross_2_3_recorded = True
                    print(f"首次穿越2/3线 - 时间戳: {cross_2_3_info['timestamp']:.6f}, 坐标: ({centroid_x}, {centroid_y})")

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

    # 打印最终记录的穿越信息
    print("\n===== 最终穿越记录 =====")
    print(f"1/3线: 时间戳={cross_1_3_info['timestamp']}, 坐标=({cross_1_3_info['x']}, {cross_1_3_info['y']})")
    print(f"2/3线: 时间戳={cross_2_3_info['timestamp']}, 坐标=({cross_2_3_info['x']}, {cross_2_3_info['y']})")
    speed = (cross_2_3_info['x'] - cross_1_3_info['x']) / ratio_wh / (cross_2_3_info['timestamp'] - cross_1_3_info['timestamp'])
    print(f"speed:{speed} cm/s")

    camera.stop()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
