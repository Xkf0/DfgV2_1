# main.py
#!/usr/bin/env python3
import json
import time
import cv2
import numpy as np
import threading
import queue
from loguru import logger
import datetime  # 导入时间模块，用于获取当前时间

# 引入 Configer 和新的 ObjectDetector
from config_loader import Configer
from object_detector import ObjectDetector
from camera_handler1 import initCamera  # 新增导入

# 自定义模块导入
import vision_utils as vu
from fairino2_8 import (
    move_to_safe_position_add_cloth_lenth,
    place_and_move_to_safe,
    place_and_move_to_safe_arm2,
    update_centroid_time, 
    update_count_and_next_placement_height,
    update_count_and_next_placement_height_arm2,
    update_centroid_time_arm2,
    follow_and_grasp_dynamic_smooth_with_detect,
    follow_and_grasp_dynamic_smooth_with_detect_arm2,
    get_count_arm1,get_count_arm2,
    initRobot_And_move2SafePosition_And_clothLength2Zero,
    CONFIG
)

from modules.module_speeddect import SpeedMonitor
from logger import LOG_DEBUG, LOG_INFO, LOG_WARN, LOG_ERROR, LOG_CRITICAL
import math
from control_air_close_open import grip_clamp, grip_open, grip_release, start_suction, stop_suction
import test_reid_module_and_camera
# 全局变量
robot_mode=1
# 1号机械臂队列与锁
task_queue_1 = queue.Queue()
task_lock_1 = threading.Lock()
# 2号机械臂新增队列与锁
task_queue_2 = queue.Queue()
task_lock_2 = threading.Lock()

speed_now=0
time_pre_now=0
speed_lock = threading.Lock()  # 新增：速度变量的锁

# 成功抓取布料后放置位置
place_pos_arm_1 = [535.021, 387.565, 170.813, 178.894, -1.774,-47.716]
place_pos_arm_2 = [535.021, 387.565, 141.813, 178.894, -1.774,-47.716] # G  ( 178.894, -0.425, -149.585)末端位姿需要结合末端改

# 其他原有变量保留
secondary_task_queue = queue.Queue()
secondary_lock = threading.Lock()

# AFFINE_MATRIX = None
AFFINE_MATRIX_1 = None
AFFINE_MATRIX_2 = None

IS_USE_SAM = True
# IS_USE_SAM = False

be_ignored = 0

# 从配置中读取参数
robot_mode = CONFIG["robot_mode"]
DIRECTION_1 = np.array(CONFIG["DIRECTION_1"])
DIRECTION_2 = np.array(CONFIG["DIRECTION_2"])
place_pos_arm_1 = CONFIG["place_pos_arm_1"]
place_pos_arm_2 = CONFIG["place_pos_arm_2"]
TASK_EXPIRATION_THRESHOLD = CONFIG["TASK_EXPIRATION_THRESHOLD"]

# 实例化Configer
cfg_1 = Configer(**CONFIG["CONFIG_PARAMS_1"])
cfg_2 = Configer(**CONFIG["CONFIG_PARAMS_2"])

# 计算边缘参数（原硬编码的edge_1/2相关）
edge_params = CONFIG["edge_params"]
edge_2_in = int(edge_params["edge_2_in_ratio"] * cfg_1.ratio_wh)
edge_2_out = int(edge_params["edge_2_out_ratio"] * cfg_1.ratio_wh)
edge_1_in = int(edge_params["edge_1_in_ratio"] * cfg_1.ratio_wh)
edge_1_out = int(edge_params["edge_1_out_ratio"] * cfg_1.ratio_wh)

def process_tasks_speed(monitor):
    """
    处理实时获取速度模式
    """
    global time_pre_now
    global speed_now
    time.sleep(1)  # 初始等待数据
    count=0
    while True:
        count+=1
        speed= -monitor.speed_now*100
        # LOG_INFO("speed: %f", speed)
        if speed > 0:
            with speed_lock:
                speed_now=speed
                time_pre_now = cfg_1.WAIT_DISTANCE / speed_now 
                if(count%100==1):
                    print(f"当前实时平均速度: {speed_now:.4f} cm/s")
                    print(f"当前实时预抓取时间: {time_pre_now:.4f} s")
        else:
            print(f"速度获取异常,沿用上一次速度")
        time.sleep(0.1)  # 刷新间隔，调整根据需要   

def process_tasks_1(rpc):
    """
    处理抓取任务的线程函数（智能过滤版）
    """
    global DIRECTION_1
    
    place_pos = place_pos_arm_1
    sum_detectnum=0
    sum_abort=0
    sum_to_do=0
    print(">>> 1号机械臂处理线程已启动，等待任务...")

    while True:
        # 1. 获取任务（阻塞等待，直到至少有一个任务）
        item = task_queue_1.get()
        
        # 退出信号检查
        if item[0] is None:
            secondary_task_queue.put((None, None))
            # task_queue.task_done()
            task_queue_1.task_done()
            break

        # 2. 【核心修改】智能任务清洗逻辑
        # 机械臂刚忙完，现在看看队列里是不是堆了一堆旧任务？
        # 我们把它们全取出来，只挑最好的一个。
        latest_valid_item = item # 默认当前取到的这个
        dropped_count = 0
        sum_detectnum += 1
        with speed_lock:
            get_speed_now=speed_now
            get_time_pre_now=time_pre_now
        # 锁定队列，把里面积压的任务全拿出来检查
        # with task_lock:
        with task_lock_1:
            while not task_queue_1.empty():
                try:
                    next_item = task_queue_1.get_nowait()
                    
                    # 检查 next_item 是否是退出信号
                    if next_item[0] is None:
                        # 如果队列里夹杂着退出信号，优先处理退出
                        task_queue_1.task_done()
                        latest_valid_item = next_item
                        break 
                    
                    # 比较时间戳，判断 next_item 是否比 latest_valid_item 更“值得”抓
                    # 这里的逻辑是：既然我们已经落后了，直接丢弃旧的，只看最新的
                    # (注意：task_queue.task_done() 必须调用，否则 join 会阻塞)
                    task_queue_1.task_done() 
                    
                    # 将旧的 latest_valid_item 丢弃，更新为 next_item
                    # 这里假设队列后面的肯定是更新的检测结果
                    latest_valid_item = next_item
                    dropped_count += 1
                    sum_detectnum += 1
                    sum_abort += 1
                    
                except queue.Empty:
                    break
        
        if dropped_count > 0:
            print(f"[1号臂-智能过滤] 丢弃了 {dropped_count} 个积压的旧任务，直接处理最新任务。")

        # 重新赋值 item 为清洗后的最新任务
        item = latest_valid_item
        if item[0] is None: break #再一次检查退出

        motion_dict, mid, pos_data, safe_pos = item
        logger.info(motion_dict[mid]['motion_center'])
        try:
            # 3. 【核心修改】过期时间检查 (Time Check)
            # 在移动机械臂之前，先算一下是否来得及
            # time_pre 是配置的预估到达时间，line2_time 是检测线触发时间
            arrival_time = motion_dict[mid]['line2_time'] + get_time_pre_now

            current_time = time.perf_counter()
            wait_time = arrival_time - current_time

            print(f"1号臂-id-{mid} 预判检查: wait_time={wait_time:.3f}s")

            # 如果 wait_time 小于阈值（例如 -1.0s），说明物体已经过了很久了
            # logger.error(f"+++++++++++++++TASK_EXPIRATION_THRESHOLD:{TASK_EXPIRATION_THRESHOLD}")
            if wait_time < TASK_EXPIRATION_THRESHOLD:
                print(f"[1号臂-过期跳过] id-{mid} 已过期 {abs(wait_time):.2f}s，放弃本次抓取，立即寻找下一个。")
                sum_abort += 1
                continue # 直接跳过，进入下一次循环
            #机械臂运动开关
            if False:
            # if True:
               continue # 直接跳过，进入下一次循环


            # --- 任务有效，开始执行 ---
            sum_to_do += 1
            cloth_lenth = motion_dict[mid]['long_side_length']
            print(f"1号臂-id-{mid} 开始执行 | 布料长度: {cloth_lenth:.2f}mm")
            # cloth_lenth=360
            
            time11 = time.perf_counter()
            # if True:
                # start_suction(rpc)
            success, err = move_to_safe_position_add_cloth_lenth(rpc, safe_pos, motion_dict, mid, cloth_lenth,robot_lock=task_lock_1,arm_id=1)
            time22 = time.perf_counter()
            # print("Move Safe Time:", time22 - time11)
            
            if not success:
                print(f"1号臂-id-{mid} 移动到安全点失败: {err}")
                continue

            # 6. 二次等待时间校准
            # 重新计算 wait_time，因为上面执行了一些动作
            # 注意：这里的逻辑原代码可能有点问题，通常 move_to_safe 之后就要准备抓了
            # 如果 wait_time 还是正数，说明我们动作太快了，需要等物体过来
            
            wait_time_final = get_time_pre_now - (time.perf_counter() - motion_dict[mid]['line2_time']) - 0.5
            # wait_time_final = cfg_1.time_pre - (time.perf_counter() - motion_dict[mid]['line2_time'])
            
            if wait_time_final > 0:
                # print(f"动作超前，等待物体到位: {wait_time_final:.3f}s")
                time.sleep(wait_time_final)
            # elif wait_time_final < TASK_EXPIRATION_THRESHOLD:
            else:
                 # 极端情况：移动到一半，物体跑了
                 print(f"[1号臂-严重过期] 准备抓取时发现物体已跑远: {wait_time_final:.3f}s")
                 # 这里可以选择继续尝试(赌一把)或者放弃，建议继续尝试，因为已经到这步了
            
            print("1号臂-抓取点预测坐标%%%%%%%%%%%%%%%%%%",pos_data)
            
            # 7. 动态跟随抓取,带异常检测 (Phase 2)
            res, pos, err = follow_and_grasp_dynamic_smooth_with_detect(
                rpc, pos_data, speed_cm_s=48.0, descend_duration=0.30,
                descend_height_mm=cfg_1.down_height, clamp_delay_s=0.1,
                follow_after_clamp_s=0.30, movej_vel=40.0, control_freq_hz=200,
                place_pos=place_pos, safe_pos1=cfg_1.safe_pos_1, safe_pos2=cfg_1.safe_pos_2, 
                cloth_length=cloth_lenth,safe_pos_1_5=cfg_1.safe_pos_1_5,
                DIRECTION=DIRECTION_1,robot_lock=task_lock_1,
                detect_pose=cfg_1.detect_pose, detect_none_pose=cfg_1.detect_none_pose,
                if_grip=False
            )

            
            result='失败'
            if res==1:
                result='成功'
                tl_cur_pos_array = rpc.robot_state_pkg.tl_cur_pos
                tl_cur_pos_list = [tl_cur_pos_array[i] for i in range(6)]
                print("tl_cur_pos_list@@@@@@@@@@@@@@@",tl_cur_pos_list)
                place_and_move_to_safe(rpc, cfg_1.safe_pos_1, tl_cur_pos_list, cfg_1.safe_pos_2)
                update_count_and_next_placement_height()
                update_centroid_time()
            elif res==2:
                print(f"1号臂-id-{mid} 抓取动作执行结果: 未检测到布料，放弃本次抓取。\n")
            else:
                print(f"1号臂-id-{mid} 抓取动作执行失败: {err}")
            
            sum_real_done =get_count_arm1()
            logger.info(f"====================1号臂: "
                        f"当前处理结果: {result} ; "
                        f"总检测量： {sum_detectnum}; "
                        f"实际成功总量:{sum_real_done}; "
                        f"积压抛弃总量{sum_abort}; "
                        f"机械臂处理数量: {sum_to_do} ; "
                        f"机械臂处理失败总量: {sum_to_do-sum_real_done} ; "
                        f"====================")

        except Exception as e:
            print(f"1号臂-任务执行异常: {e}")
            import traceback
            traceback.print_exc()
        finally:
            # 标记当前单个任务完成（对应最开始 get 的那个）
            task_queue_1.task_done()

def process_tasks_2(rpc):
    """
    处理2号抓取任务的线程函数（智能过滤版）
    """
    global DIRECTION_2
    place_pos = place_pos_arm_2
    sum_detectnum=0
    sum_abort=0
    sum_to_do=0
    print(">>> 2号机械臂处理线程已启动，等待任务...")

    while True:
        # 1. 获取任务（阻塞等待，直到至少有一个任务）
        item = task_queue_2.get()
        
        # 退出信号检查
        if item[0] is None:
            secondary_task_queue.put((None, None))
            task_queue_2.task_done()
            break

        # 2. 智能任务清洗逻辑（对齐1号臂）
        latest_valid_item = item
        dropped_count = 0
        sum_detectnum += 1
        with speed_lock:
            get_speed_now=speed_now
            get_time_pre_now=time_pre_now
        
        # 锁定2号臂队列，清理积压任务
        with task_lock_2:
            while not task_queue_2.empty():
                try:
                    next_item = task_queue_2.get_nowait()
                    
                    # 检查 next_item 是否是退出信号
                    if next_item[0] is None:
                        # 如果队列里夹杂着退出信号，优先处理退出
                        task_queue_2.task_done()
                        latest_valid_item = next_item
                        break 
                    
                    # 丢弃旧任务，保留最新的                    
                    # 比较时间戳，判断 next_item 是否比 latest_valid_item 更“值得”抓
                    # 这里的逻辑是：既然我们已经落后了，直接丢弃旧的，只看最新的
                    # (注意：task_queue.task_done() 必须调用，否则 join 会阻塞)
                    task_queue_2.task_done() 
                    
                    # 将旧的 latest_valid_item 丢弃，更新为 next_item
                    # 这里假设队列后面的肯定是更新的检测结果
                    latest_valid_item = next_item
                    dropped_count += 1
                    sum_detectnum += 1
                    sum_abort += 1
                    
                except queue.Empty:
                    break
        
        if dropped_count > 0:
            print(f"[2号臂-智能过滤] 丢弃了 {dropped_count} 个积压的旧任务，直接处理最新任务。")

        # 重新赋值 item 为清洗后的最新任务
        item = latest_valid_item
        #再一次检查退出
        if item[0] is None: 
            break

        motion_dict, mid, pos_data, safe_pos = item
        logger.info(f"2号臂-{mid} 运动中心: {motion_dict[mid]['motion_center']}")
        try:
            # 3. 【核心修改】过期时间检查 (Time Check)
            # 在移动机械臂之前，先算一下是否来得及
            # time_pre 是配置的预估到达时间，line2_time 是检测线触发时间
            # arrival_time = line2_time + time_pre
            arrival_time = motion_dict[mid]['line2_time'] + get_time_pre_now
            current_time = time.perf_counter()
            wait_time = arrival_time - current_time

            print(f"2号臂-id-{mid} 预判检查: wait_time={wait_time:.3f}s")

            # 过期任务直接跳过
            if wait_time < TASK_EXPIRATION_THRESHOLD:
                print(f"[2号臂-过期跳过] id-{mid} 已过期 {abs(wait_time):.2f}s，放弃本次抓取。")
                sum_abort += 1
                continue
            
            # 调试开关
            if False:
                continue

            # --- 任务有效，开始执行 ---
            sum_to_do += 1
            cloth_lenth = motion_dict[mid]['long_side_length']
            print(f"2号臂-id-{mid} 开始执行 | 布料长度: {cloth_lenth:.2f}mm")
            
            # 移动到安全位置（使用2号臂锁）
            success, err = move_to_safe_position_add_cloth_lenth(rpc, safe_pos, motion_dict, mid, cloth_lenth,robot_lock=task_lock_2,arm_id=2)
            
            if not success:
                print(f"2号臂-id-{mid} 移动到安全点失败: {err}")
                continue

            # 二次等待时间校准
            wait_time_final = get_time_pre_now - (time.perf_counter() - motion_dict[mid]['line2_time'])
            if wait_time_final > 0:
                time.sleep(wait_time_final)
            else:
                 # 极端情况：移动到一半，物体跑了
                print(f"[2号臂-严重过期] 准备抓取时发现物体已跑远: {wait_time_final:.3f}s")
            
            print("2号臂-抓取点预测坐标%%%%%%%%%%%%%%%%%%",pos_data)
            
            # 7. 动态跟随抓取（使用2号臂专属函数）
            res, pos, err = follow_and_grasp_dynamic_smooth_with_detect_arm2(
                rpc, pos_data, speed_cm_s=get_speed_now, descend_duration=0.30,
                descend_height_mm=cfg_2.down_height, clamp_delay_s=0.1,
                follow_after_clamp_s=0.30, movej_vel=40.0, control_freq_hz=200,
                place_pos=place_pos, safe_pos1=cfg_2.safe_pos_1, cloth_length=cloth_lenth,
                DIRECTION=DIRECTION_2,robot_lock=task_lock_2,
                detect_pose=cfg_2.detect_pose, detect_none_pose=cfg_2.detect_none_pose,
                if_grip=False
            )

            # 抓取结果处理
            result='失败'
            if res==1:
                result='成功'
                place_and_move_to_safe_arm2(rpc, cfg_2.safe_pos_1, place_pos)
                update_count_and_next_placement_height_arm2()
                update_centroid_time_arm2()
            elif res==2:
                print(f"2号臂-id-{mid} 抓取动作执行结果: 未检测到布料，放弃本次抓取。\n")
            else:
                print(f"2号臂-id-{mid} 抓取动作执行失败: {err}")
            
            # 统计信息输出
            sum_real_done = get_count_arm2()
            logger.info(f"====================2号臂: "
                        f"当前处理结果: {result} ; "
                        f"总检测量： {sum_detectnum}; "
                        f"实际成功总量:{sum_real_done}; "
                        f"积压抛弃总量{sum_abort}; "
                        f"机械臂处理数量: {sum_to_do} ; "
                        f"机械臂处理失败总量: {sum_to_do-sum_real_done} ; "
                        f"====================")

        except Exception as e:
            print(f"2号臂-任务执行异常: {e}")
            import traceback
            traceback.print_exc()
        finally:
            # 标记任务完成，更新忙碌状态
            task_queue_2.task_done()

# --- 主函数 main 修改 ---
def main():
    global AFFINE_MATRIX_1, AFFINE_MATRIX_2, speed_now, time_pre_now, be_ignored
    # 1. 初始化两个机械臂（假设init_robot支持指定IP，若同IP则传相同参数）以及初始化变量
    robot_ip1            = CONFIG["robot_ip1"]
    robot_ip2            = CONFIG["robot_ip2"]
    length_lead_screw_cm = CONFIG["length_lead_screw_cm"]
    openCollisionDetect  = CONFIG["openCollisionDetect"]
    speed_mode           = CONFIG["speed_mode"]
    speed_port           = CONFIG["speed_port"]
    speed_now            = cfg_1.speed
    time_pre_now         = cfg_1.time_pre
    sum_detect           = [0]

    rpc_1, rpc_2 = initRobot_And_move2SafePosition_And_clothLength2Zero(robot_mode, cfg_1, cfg_2, task_lock_1, task_lock_2, robot_ip1, robot_ip2)

    # 2. 初始化相机与对象检测器
    camera = initCamera(cfg_1)
    camera2 = initCamera(cfg_2)
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
        threading.Thread(target=process_tasks_speed, args=(monitor,), daemon=True).start()

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

        with speed_lock:
            get_speed_now=speed_now
            get_time_pre_now=time_pre_now
        
        motion_dict, tracked_objects, display_img = detector.get_motionDict_trackedObjects_DisplayImg(frame, AFFINE_MATRIX_1, current_time, get_speed_now)
        
        to_del = []

        vu.cycleTaskHandle(tracked_objects, edge_1_in, edge_1_out, edge_2_in, edge_2_out, sum_detect, cfg_1, cfg_2, motion_dict, openCollisionDetect, length_lead_screw_cm, to_del, AFFINE_MATRIX_1, AFFINE_MATRIX_2, DIRECTION_1, DIRECTION_2, get_speed_now, get_time_pre_now, task_lock_1, task_lock_2, task_queue_1, task_queue_2)

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
    #     task_queue_1.put((None, None, None, None, None))
    #     task_queue_1.join()
    #     task_queue_2.put((None, None, None, None, None))
    #     task_queue_2.join()
        
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



