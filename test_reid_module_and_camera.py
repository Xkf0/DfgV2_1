# main.py
#!/usr/bin/env python3
import time
import cv2

# 引入 Configer 和新的 ObjectDetector
from camera_handler1 import CameraHandler  # 新增导入

# 自定义模块导入
import vision_utils as vu
from get_points import get_points

import reid_module
import numpy as np
import time
from logger import LOG_DEBUG, LOG_INFO, LOG_WARN, LOG_ERROR, LOG_CRITICAL
from global_state import AppState
import threading
from global_state import AppState

lastNoFabric      = time.time() # 上一次图中没有布料的时间

lastThereIsFabric = [] # 上一帧是否有布料，如果有则为True
timeEnter         = []
timeLeave         = []
timesEnter        = []
timesLeave        = [] # 离开次数，只有大于timesContinuous才是真的离开
enterFabric       = []
count             = []
similarityMin     = []
similarityMax     = []
draw_picture      = False

num               = 4

for index in range(num * 2):
    lastThereIsFabric.append(False) # 上一帧是否有布料，如果有则为True
    timeEnter.append(time.time())
    timeLeave.append(time.time())
    timesEnter.append(0)
    timesLeave.append(0) # 离开次数，只有大于timesContinuous才是真的离开
    enterFabric.append(False)
    count.append(0)
    similarityMin.append(10000)
    similarityMax.append(-10000)

threshold = [0.92, 0.92, 0.92, 0.92, 0.95, 0.92, 0.92, 0.92]
timesContinuous = 10


def detect(lastThereIsFabric, nowThereIsFabric, timeEnter, timeLeave, timesEnter, timesLeave, enterFabric, current_time, index, count):
    if lastThereIsFabric == False and nowThereIsFabric == True:
        if enterFabric:
            timesLeave = 0
        else:
            timesEnter = 1

    if lastThereIsFabric == True and nowThereIsFabric == False:
        if enterFabric:
            timesLeave = 1
        else:
            timesEnter = 0

    if lastThereIsFabric == False and nowThereIsFabric == False:
        if enterFabric:
            timesLeave += 1
            if timesLeave == timesContinuous:
                enterFabric = False
                timeLeave = current_time
                timesLeave = 0
                LOG_INFO("第%d个窗口的第%d个布料离开, 经过时间为%fs", index, count, timeLeave - timeEnter)
                print(f"第{index}个窗口的第{count}个布料离开, 经过时间为{timeLeave - timeEnter}s")

    if lastThereIsFabric == True and nowThereIsFabric == True:
        if enterFabric == False:
            timesEnter += 1
            if timesEnter > timesContinuous:
                enterFabric = True
                timeEnter = current_time
                timesEnter = 0
                count += 1
                LOG_INFO("第%d个窗口的第%d个布料进入", index, count)
                print(f"第{index}个窗口的第{count}个布料进入")
        if enterFabric == True:
            if current_time - timeEnter > 10:
                LOG_INFO("——————————————————————————堵料——————————————————————————")
                print("——————————————————————————堵料——————————————————————————")
                return lastThereIsFabric, nowThereIsFabric, timeEnter, timeLeave, timesEnter, timesLeave, enterFabric, count, True
    return lastThereIsFabric, nowThereIsFabric, timeEnter, timeLeave, timesEnter, timesLeave, enterFabric, count, False

def anomaly_detection():
    global lastNoFabric, lastThereIsFabric, timeEnter, timeLeave, timesLeave, timesEnter, enterFabric, threshold, timesContinuous, similarityMin, similarityMax, count, num, draw_picture
    featureExtractor = reid_module.FeatureExtractor()
    leftLarge = 770 # 距离左侧边缘距离
    bottom = 435 # 距离底部边缘距离
    rightLarge = 590 # 距离右侧边缘距离                                         
    top = 570 # 距离顶部边缘距离
    left = 0 # 距离左侧边缘距离
    bottom = 0 # 距离底部边缘距离
    right = 0 # 距离右侧边缘距离
    top = 0 # 距离顶部边缘距离
    contourLarge = np.array([
        [bottom, leftLarge],   # 右上角
        [1080 - top, leftLarge],   # 右下角
        [1080 - top, 1920 - rightLarge],  # 左下角
        [bottom, 1920 - rightLarge]    # 左上角
    ], dtype=np.float32)  # 必须为 float32 类型
    deltax = (1920 - leftLarge - rightLarge) / num
    left = []
    right = []
    for index in range(num):
        left.append(int(leftLarge + index * deltax))
    for index in range(num):
        right.append(int(rightLarge + (3 - index) * deltax))
    left.append(leftLarge)
    right.append(rightLarge)
    for index in range(num - 1):
        left.append(int(leftLarge + (index + 0.5) * deltax))
    for index in range(num - 1):
        right.append(int(rightLarge + (2.5 - index) * deltax))
    
    contour = []
    for index in range(num * 2):
        contour.append(np.array([
        [bottom, left[index]],   # 右上角
        [1080 - top, left[index]],   # 右下角
        [1080 - top, 1920 - right[index]],  # 左下角
        [bottom, 1920 - right[index]]    # 左上角
    ], dtype=np.float32))  # 必须为 float32 类型
        
    print(f"contour: {contour}")

    try:
        camera = CameraHandler(
            is_real_sense=AppState.cfg_1.is_real_sense,
            camera_id=AppState.cfg_1.camera_no,
            warmup_frames=30  # 前30帧预热
        )
        print("相机初始化完成")
    except Exception as e:
        print(f"相机初始化失败: {e}")
        return

    print("开始主循环...")

    first_frame = True
    feature_first_frame = None

    while True:
        if AppState.wait_detect is False:
            time_start = time.perf_counter()

            frame = camera.get_frame_directly()
            if frame is None:
                print("未能获取有效帧，跳过本次循环")
                continue

            frame = np.transpose(frame, (1, 0, 2))
            frame = cv2.flip(frame, 1)

            if first_frame:
                feature_first_frame = []
                for index in range(num * 2):
                    crop, feature_first_frame_temp = reid_module.extract_mask_crop_with_features(frame, contour[index], featureExtractor)
                    feature_first_frame.append(feature_first_frame_temp)
                first_frame = False
            else:
                crop, feature_large = reid_module.extract_mask_crop_with_features(frame, contourLarge, featureExtractor)
                similarity = []
                nowThereIsFabric = []
                for index in range(num * 2):
                    _, feature_temp = reid_module.extract_mask_crop_with_features(frame, contour[index], featureExtractor)
                    similarity.append(featureExtractor.cosine_similarity(feature_first_frame[index], feature_temp))
                    nowThereIsFabric.append(similarity[index] <= threshold[index])
                    # print(similarity)
                    similarityMin[index] = min(similarity[index], similarityMin[index])
                    similarityMax[index] = max(similarity[index], similarityMax[index])
                    # print(f"similarityMin: {similarityMin[index]}, index: {index}")
                    # print(f"similarityMax: {similarityMax[index]}, index: {index}")

                current_time = time.time()
                thereIs = False
                
                for index in range(num * 2):
                    LOG_INFO("index: %d, astThereIsFabric: %d, nowThereIsFabric: %d, timeEnter: %f, timeLeave: %f, timesEnter: %d, timesLeave: %d, enterFabric: %d, similarity: %.5f", index, lastThereIsFabric[index], nowThereIsFabric[index], timeEnter[index], timeLeave[index], timesEnter[index], timesLeave[index], enterFabric[index], similarity[index])
                    lastThereIsFabric[index], nowThereIsFabric[index], timeEnter[index], timeLeave[index], timesEnter[index], timesLeave[index], enterFabric[index], count[index], thereIs = detect(lastThereIsFabric[index], nowThereIsFabric[index], timeEnter[index], timeLeave[index], timesEnter[index], timesLeave[index], enterFabric[index], current_time, index, count[index])
                    if thereIs:
                        break
                if thereIs:
                    with AppState.state_lock:
                        AppState.blocked_state = True
                        AppState.wait_detect = True
                    continue
                # if similarity > threshold:
                #     lastNoFabric = current_time
                lastThereIsFabric = nowThereIsFabric
                # if current_time - lastNoFabric > 10:
                #     print("——————————————————————————堵料——————————————————————————")

            display_img = crop

            # 绘制FPS
            fps = 1 / (time.perf_counter() - time_start)
            cv2.putText(display_img, f"FPS:{fps:.2f}", (AppState.cfg_2.output_w - 500, AppState.cfg_2.output_h - 20), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 8)

            # 显示或保存
            display_img = cv2.resize(display_img, (display_img.shape[1]//3, display_img.shape[0]//3))
            
            cv2.imshow('combined1', display_img)
            cv2.waitKey(1)
        else:
            time.sleep(0.1)

if __name__ == "__main__":
    draw_picture = True
    thread = threading.Thread(target=anomaly_detection, args=(), daemon=True)
    thread.start()
    while True:
        time.sleep(0.1)
        x = 1
    
    # anomaly_detection()