# main.py
#!/usr/bin/env python3
import time
import cv2
import numpy as np
import threading

# 引入 Configer 和新的 ObjectDetector
from config_loader import Configer
from object_detector import ObjectDetector
from camera_handler1 import initCamera  # 新增导入

# 自定义模块导入
import vision_utils as vu
from fairino2_8 import (
    initRobot_And_move2SafePosition_And_clothLength2Zero,
    process_tasks_1,
    process_tasks_2,
    CONFIG
)

from modules.module_speeddect import SpeedMonitor, process_tasks_speed
from logger import LOG_DEBUG, LOG_INFO, LOG_WARN, LOG_ERROR, LOG_CRITICAL
from global_state import AppState

# --- 主函数 main 修改 ---
def main():
    # 1. 初始化两个机械臂（假设init_robot支持指定IP，若同IP则传相同参数）以及初始化变量
    robot_ip1             = CONFIG["robot_ip1"]
    robot_ip2             = CONFIG["robot_ip2"]
    length_lead_screw_cm  = CONFIG["length_lead_screw_cm"]
    openCollisionDetect   = CONFIG["openCollisionDetect"]
    speed_mode            = CONFIG["speed_mode"]
    speed_port            = CONFIG["speed_port"]
    robot_mode            = CONFIG["robot_mode"]
    edge_params           = CONFIG["edge_params"]
    cfg_1                 = Configer(**CONFIG["CONFIG_PARAMS_1"])
    cfg_2                 = Configer(**CONFIG["CONFIG_PARAMS_2"])
    edge_2_in             = int(edge_params["edge_2_in_ratio"] * cfg_1.ratio_wh)
    edge_2_out            = int(edge_params["edge_2_out_ratio"] * cfg_1.ratio_wh)
    edge_1_in             = int(edge_params["edge_1_in_ratio"] * cfg_1.ratio_wh)
    edge_1_out            = int(edge_params["edge_1_out_ratio"] * cfg_1.ratio_wh) 
    DIRECTION_1           = np.array(CONFIG["DIRECTION_1"])
    DIRECTION_2           = np.array(CONFIG["DIRECTION_2"])
    AppState.speed_now    = cfg_1.speed
    AppState.time_pre_now = cfg_1.time_pre
    sum_detect            = [0]
    IS_USE_SAM            = True

    rpc_1, rpc_2 = initRobot_And_move2SafePosition_And_clothLength2Zero(robot_mode, cfg_1, cfg_2, AppState.task_lock_1, AppState.task_lock_2, robot_ip1, robot_ip2)

    # 2. 初始化相机与对象检测器
    camera = initCamera(cfg_1)
    detector = ObjectDetector(cfg_1, IS_USE_SAM)

    # 3. 加载坐标标定并计算仿射矩阵
    AFFINE_MATRIX_1 = vu.compute_affine_transform(cfg_1)
    AFFINE_MATRIX_2 = vu.compute_affine_transform(cfg_2)
    print("仿射矩阵计算完成")

    # 4. 启动任务线程
    if robot_mode==1:
        threading.Thread(target=process_tasks_1, args=(rpc_1,), daemon=True).start()
    elif robot_mode==2:
        threading.Thread(target=process_tasks_2, args=(rpc_2,), daemon=True).start()
    elif robot_mode==3:
        threading.Thread(target=process_tasks_1, args=(rpc_1,), daemon=True).start()
        threading.Thread(target=process_tasks_2, args=(rpc_2,), daemon=True).start()
    else:
        print("请选择正确的模式")
    if speed_mode == 1:
        monitor = SpeedMonitor(serial_port = speed_port, enable_plot=False)  # 根据需要配置
        monitor.start()
        threading.Thread(target=process_tasks_speed, args=(monitor, AppState.speed_lock, cfg_1), daemon=True).start()

    print("开始主循环...")

    # try:
    while True:
        time_start = time.perf_counter()

        frame = camera.get_frame_directly()
        # frame2 = camera.get_frame_directly()
        if frame is None:
            print("未能获取有效帧，跳过本次循环")
            continue
        # test_reid_module_and_camera.anomaly_detection(frame2)
                
        current_time = time.perf_counter()

        with AppState.speed_lock:
            get_speed_now=AppState.speed_now
            get_time_pre_now=AppState.time_pre_now
        
        motion_dict, tracked_objects, display_img = detector.get_motionDict_trackedObjects_DisplayImg(frame, AFFINE_MATRIX_1, current_time, get_speed_now)
        
        to_del = []

        vu.cycleTaskHandle(tracked_objects, edge_1_in, edge_1_out, edge_2_in, edge_2_out, sum_detect, cfg_1, cfg_2, motion_dict, openCollisionDetect, length_lead_screw_cm, to_del, AFFINE_MATRIX_1, AFFINE_MATRIX_2, DIRECTION_1, DIRECTION_2, get_speed_now, get_time_pre_now, AppState.task_lock_1, AppState.task_lock_2, AppState.task_queue_1, AppState.task_queue_2)

        # 绘制FPS
        fps = 1 / (time.perf_counter() - time_start)
        cv2.putText(display_img, f"FPS:{fps:.2f}", (cfg_1.output_w - 500, cfg_1.output_h - 20), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 8)
        # 显示或保存
        display_img = cv2.resize(display_img, (display_img.shape[1]//2, display_img.shape[0]//2))
        cv2.imshow('combined', display_img)
        cv2.waitKey(1)
                     
    # finally:
    #     # 清理资源
    #     print("正在清理资源...")
    #     AppState.task_queue_1.put((None, None, None, None, None))
    #     AppState.task_queue_1.join()
    #     AppState.task_queue_2.put((None, None, None, None, None))
    #     AppState.task_queue_2.join()
        
    #     # 停止相机
    #     camera.stop()
        
    #     cv2.destroyAllWindows()
    #     if robot_mode==1:
    #         rpc_1.RobotEnable(0)
    #     elif robot_mode==2:
    #         rpc_2.RobotEnable(0)
    #     elif robot_mode==3:
    #         rpc_1.RobotEnable(0)
    #         rpc_2.RobotEnable(0)
    #     print("程序退出")

if __name__ == "__main__":
    main()



