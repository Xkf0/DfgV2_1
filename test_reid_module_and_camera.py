# main.py
#!/usr/bin/env python3
import time
import cv2
import threading
import datetime  # 导入时间模块，用于获取当前时间

# 引入 Configer 和新的 ObjectDetector
from config_loader import Configer
from object_detector import ObjectDetector
from camera_handler1 import CameraHandler  # 新增导入

# 自定义模块导入
import vision_utils as vu
from fairino2_8 import (CONFIG)
from get_points import get_points

import reid_module
import numpy as np

# 实例化Configer
cfg_1 = Configer(**CONFIG["CONFIG_PARAMS_1"])
cfg_2 = Configer(**CONFIG["CONFIG_PARAMS_2"])

# --- 主函数 main 修改 ---
def anomaly_detection():
    # 初始化两个机械臂（假设init_robot支持指定IP，若同IP则传相同参数）
    # robot_mode=3

    # 2. 初始化相机（使用新的CameraHandler类）
    try:
        camera = CameraHandler(
            is_real_sense=cfg_1.is_real_sense,
            camera_id=cfg_1.camera_no,
            warmup_frames=30  # 前30帧预热
        )
        print("相机初始化完成")
    except Exception as e:
        print(f"相机初始化失败: {e}")
        return

    featureExtractor = reid_module.FeatureExtractor()
    left = 1000 # 距离左侧边缘距离
    bottom = 200 # 距离底部边缘距离
    right = 450 # 距离右侧边缘距离
    top = 600 # 距离顶部边缘距离
    # left = 0 # 距离左侧边缘距离
    # bottom = 0 # 距离底部边缘距离
    # right = 0 # 距离右侧边缘距离
    # top = 0 # 距离顶部边缘距离
    contour = np.array([
        [top, right],   # 右上角
        [1080 - bottom, right],   # 右下角
        [1080 - bottom, 1920 - left],  # 左下角
        [top, 1920 - left]    # 左上角
    ], dtype=np.float32)  # 必须为 float32 类型

    print("开始主循环...")

    first_frame = True
    feature_first_frame = None

    while True:
        time_start = time.perf_counter()
        
        # 获取最新帧（使用新的相机类）
        current_t = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]  # 格式：年-月-日 时:分:秒.毫秒
        content = f"{current_t} 准备获得图像帧\n"  # 加换行符，让每条记录单独一行
        with open("output.txt", mode='a', encoding='utf-8') as f:
            f.write(content)  # 将内容写入文件

        frame = camera.get_frame_directly()
        if frame is None:
            print("未能获取有效帧，跳过本次循环")
            continue

        # 获取最新帧（使用新的相机类）
        current_t = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]  # 格式：年-月-日 时:分:秒.毫秒
        content = f"{current_t} 已获得图像帧\n"  # 加换行符，让每条记录单独一行
        with open("output.txt", mode='a', encoding='utf-8') as f:
            f.write(content)  # 将内容写入文件
        frame = np.transpose(frame, (1, 0, 2))
        frame = cv2.flip(frame, 0)

        if first_frame:
            crop, feature_first_frame = reid_module.extract_mask_crop_with_features(frame, contour, featureExtractor)
            first_frame = False
        else:
            crop, feature = reid_module.extract_mask_crop_with_features(frame, contour, featureExtractor)
            similarity = featureExtractor.cosine_similarity(feature_first_frame, feature)
            print(similarity)

        display_img = crop

        # 绘制FPS
        fps = 1 / (time.perf_counter() - time_start)
        cv2.putText(display_img, f"FPS:{fps:.2f}", (cfg_1.output_w - 500, cfg_1.output_h - 20), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 8)

        # 显示或保存
        display_img = cv2.resize(display_img, (display_img.shape[1]//3, display_img.shape[0]//3))
        
        cv2.imshow('combined1', display_img)
        cv2.waitKey(1)
        
        current_t = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]  # 格式：年-月-日 时:分:秒.毫秒
        content = f"{current_t} 结束当前图像帧\n"  # 加换行符，让每条记录单独一行
        # print(f'{content} ----------------------- ')
        with open("output.txt", mode='a', encoding='utf-8') as f:
            f.write(content)  # 将内容写入文件

if __name__ == "__main__":
    anomaly_detection()



