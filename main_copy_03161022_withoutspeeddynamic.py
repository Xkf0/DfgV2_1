# main.py
#!/usr/bin/env python3
import json
import time
import cv2
import numpy as np
import threading
import queue
from loguru import logger

# 引入 Configer 和新的 ObjectDetector
from config_loader import Configer
from object_detector import ObjectDetector
from camera_handler import CameraHandler  # 新增导入

# 自定义模块导入
import vision_utils as vu
from control_air_close_open import grip_open, start_suction, stop_suction
from fairino2_8 import (
    init_robot, move_to_safe_position_add_cloth_lenth,
    move_to_safe_position_add_cloth_lenth3, place_and_move_to_safe,place_and_move_to_safe_arm2, real_phase1_down,
    update_centroid_time, update_count_and_next_placement_height,
    update_count_and_next_placement_height_arm2,
    update_centroid_time_arm2,
    follow_and_grasp_dynamic_smooth_with_detect,
    follow_and_grasp_dynamic_smooth_with_detect_arm2,
    get_count_arm1,get_count_arm2,
    
)
from get_points import get_points
from logger import LOG_DEBUG, LOG_INFO, LOG_WARN, LOG_ERROR, LOG_CRITICAL

# 全局变量
# task_queue = queue.Queue()
# task_lock = threading.Lock()
# secondary_task_queue = queue.Queue()
# secondary_lock = threading.Lock()
# is_processing = False
# 全局变量（双机械臂扩展）
# robot_mode=2
robot_mode=1
# 1号机械臂队列与锁
task_queue_1 = queue.Queue()
task_lock_1 = threading.Lock()
# 2号机械臂新增队列与锁
task_queue_2 = queue.Queue()
task_lock_2 = threading.Lock()
# DIRECTION_1= np.array([0.9999489503, 0.0101047507]) 
# DIRECTION_2= np.array([0.70006452,-0.71407959])

# 成功抓取布料后放置位置
place_pos_arm_1 = [535.021, 387.565, 170.813, 178.894, -1.774,-47.716]
place_pos_arm_2 = [535.021, 387.565, 141.813, 178.894, -1.774,-47.716] # G  ( 178.894, -0.425, -149.585)末端位姿需要结合末端改

# 其他原有变量保留
secondary_task_queue = queue.Queue()
secondary_lock = threading.Lock()
# 分机械臂标记忙碌状态
is_processing_1 = False
is_processing_2 = False

# AFFINE_MATRIX = None
AFFINE_MATRIX_1 = None
AFFINE_MATRIX_2 = None

IS_USE_SAM = True

speed_lock = threading.Lock()  # 新增：速度变量的锁

be_ignored = 0

speed_now=0
# 定义配置参数
# CONFIG_PARAMS_1 = {
#     "real_w": 64,
#     "real_h": 92.5,
#     "ratio_wh": 20,
#     "aruco_ids_corner": {'top_left': 0, 'top_right': 8, 'bottom_right': 24, 'bottom_left': 16},
#     "speed_smooth_frames": 5,
#     "similarity_threshold": 0.80,
#     # "diff_threshold": 30,
#     # "diff_threshold": 50,
#     "diff_threshold": 70,
#     "morph_kernel_size": (2, 2),
#     "morph_open_iterations": 1,
#     "morph_dilate_iterations": 1,
#     "min_contour_area": 4000,
#     "max_distance": 5,
#     "max_speed_history": 30,
#     "speed_update_interval": 0.1,
#     "area_update_interval": 1.0,
    
#     # "speed": 6.85,
#     # "speed": 8.85,
#     # "speed": 8.25,
#     "speed": 9.83,
#     # "speed": 10,
#     "WAIT_DISTANCE":75, # G 默认90,等待抓取点至1/2线的距离，用于计算time_pre：可改变拦截点等待时间
#     "safe_pos_init": [-178.672, 562.176, None,  179.688, -3.836, -47.715],
#     "stand_z": 225, # G 丝杆
#     "stand_rx": 179.688,#抬头
#     "stand_ry": -3.836,
#     "stand_rz": -47.715,
#     "blend_alpha": 1.0,
#     "is_real_sense": True,
#     "camera_no": '211622062803',
#     "points_file_path": 'cv_aruco_1.csv',
#     "down_height": 16, #传送带上方抓取时下降高度，单位mm
#     "safe_pos_2": [290.774, 487.228, 304.005, 178.893, -1.774, -47.721], # 桌子上方安全等待位
#     "detect_pose":[195.243, 526.027, 309.835, 178.894,-1.774,-47.716],
#     "detect_none_pose": [195.243, 526.027, 309.835, 178.894,-1.774,-47.716],
# }

# # 定义配置参数
# CONFIG_PARAMS_2 = {
#     "real_w": 60.1,
#     "real_h": 140.7,
#     "ratio_wh": 20,
#     "aruco_ids_corner": {'top_left': 0, 'top_right': 8, 'bottom_right': 24, 'bottom_left': 16},
#     "speed_smooth_frames": 5,
#     "similarity_threshold": 0.80,
#     # "diff_threshold": 30,
#     "diff_threshold": 70,
#     "morph_kernel_size": (2, 2),
#     "morph_open_iterations": 1,
#     "morph_dilate_iterations": 1,
#     "min_contour_area": 4000,
#     "max_distance": 5,
#     "max_speed_history": 30,
#     "speed_update_interval": 0.1,
#     "area_update_interval": 1.0,
#     "stand_rz": -85.458,
#     # "speed": 6.85,
#     # "speed": 8.85,
#     "speed": 9.5,
#     "WAIT_DISTANCE":90, # G 等待抓取点至1/2线的距离，用于计算time_pre
#     # "safe_pos_init": [-122.704, -623.732, None, 178.894, -0.425, 168.588], # 传送带上空安全等待位（拦截位置），这里的z值即高度，config中配置等于stand_z
#     "safe_pos_init": [-178.672, 562.176, None,  178.894, -1.773, -47.715], # 传送带上空安全等待位（拦截位置），这里的z值即高度，config中配置等于stand_z
#     "stand_z": 304.017,
#     "stand_rx": 178.894,
#     "stand_ry": -1.773,
#     "blend_alpha": 1.0,
#     "is_real_sense": True,
#     "camera_no": '211622062803',
#     "points_file_path": 'cv_aruco_2.csv',
#     "down_height": 16,
#     # "safe_pos_2": [429.31, -468.68, 342, 178.894, -0.425, -149.585],
#     "safe_pos_2": [355.82, -474.63, 342, -180, -0.2, -85.458], # G  #G  ( 178.894, -0.425, -149.585)末端位姿需要结合末端改
#     "detect_pose":[355.82, -474.63, 342, -180, -0.2, -85.458],
#     "detect_none_pose": [355.82, -474.63, 342, -180, -0.2, -85.458],
# }



def load_config():
    with open("CONFIG.json", "r", encoding="utf-8") as f:
        config = json.load(f)
    return config





# 全局配置对象
CONFIG = load_config()

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



# # 实例化 Configer
# # cfg = Configer(**CONFIG_PARAMS)
# cfg_1 = Configer(**CONFIG_PARAMS_1)
# cfg_2 = Configer(**CONFIG_PARAMS_2)


# # G 质心任务分配所需坐标（源于纠正后图像）
# output_w_g = int(CONFIG_PARAMS_1["real_w"]* CONFIG_PARAMS_1["ratio_wh"]) # G x坐标
# output_h_g = int(CONFIG_PARAMS_1["real_h"] * CONFIG_PARAMS_1["ratio_wh"]) # G y坐标


# edge_2_in = int(5* CONFIG_PARAMS_1["ratio_wh"]) 
# edge_2_out = int(40 * CONFIG_PARAMS_1["ratio_wh"]) 
# edge_1_in = int((CONFIG_PARAMS_1["real_h"]-40)* CONFIG_PARAMS_1["ratio_wh"]) 
# edge_1_out = int((CONFIG_PARAMS_1["real_h"]-5) * CONFIG_PARAMS_1["ratio_wh"]) 




# --- 任务处理函数保持不变 ---
# ================= 配置区域 =================
# 如果 wait_time 小于此值（秒），说明物体已经流过机械臂太远了，放弃抓取
TASK_EXPIRATION_THRESHOLD = -0.8
# ===========================================

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
        # print
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
    # global is_processing
    global is_processing_1,is_processing_2,DIRECTION_1
    
    
    # place_pos = [400.455, 460.545, 280.548, -180, -0.2, -85.458]
    place_pos = place_pos_arm_1
    # place_pos[2] = CONFIG["MOVE_PARAMS_1"]["INIT_PLACE_HEIGHT"]
    sum_detectnum=0
    sum_abort=0
    sum_to_do=0
    LOG_INFO(">>> 机械臂处理线程已启动，等待任务...")

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
            LOG_INFO(f"[智能过滤] 丢弃了 {dropped_count} 个积压的旧任务，直接处理最新任务。")

        # 重新赋值 item 为清洗后的最新任务
        item = latest_valid_item
        if item[0] is None: break #再一次检查退出

        motion_dict, mid, pos_data, safe_pos, mode,robot_coord_test = item
        logger.info(motion_dict[mid]['motion_center'])
        try:
            # 3. 【核心修改】过期时间检查 (Time Check)
            # 在移动机械臂之前，先算一下是否来得及
            # time_pre 是配置的预估到达时间，line2_time 是检测线触发时间
            # arrival_time = line2_time + time_pre
            arrival_time = motion_dict[mid]['line2_time'] + cfg_1.time_pre
            current_time = time.perf_counter()
            wait_time = arrival_time - current_time

            LOG_INFO(f"id-{mid} 预判检查: wait_time={wait_time:.3f}s")

            # 如果 wait_time 小于阈值（例如 -1.0s），说明物体已经过了很久了
            if wait_time < TASK_EXPIRATION_THRESHOLD:
                print(f"[过期跳过] id-{mid} 已过期 {abs(wait_time):.2f}s，放弃本次抓取，立即寻找下一个。")
                is_processing_1 = False # 标记为空闲
                sum_abort += 1
                continue # 直接跳过，进入下一次循环
            
            # if True:
            if False:
                print("抓取点真实坐标##################",robot_coord_test)
                print("抓取点预测坐标%%%%%%%%%%%%%%%%%%",pos_data)
                continue # 直接跳过，进入下一次循环


            # --- 任务有效，开始执行 ---
            sum_to_do += 1
            cloth_lenth = motion_dict[mid]['long_side_length']
            print(f"id-{mid} 开始执行 | 布料长度: {cloth_lenth:.2f}mm")
            
            time11 = time.perf_counter()
            success, err = move_to_safe_position_add_cloth_lenth(rpc, safe_pos, motion_dict, mid, cloth_lenth,robot_lock=task_lock_1)
            time22 = time.perf_counter()
            # print("Move Safe Time:", time22 - time11)
            
            if not success:
                print(f"id-{mid} a移动到安全点失败: {err}")
                continue

            # # 4. 精确下降 (Phase 1)
            # # 注意：real_phase1_down 内部使用了 time.sleep 来做时间同步
            # # 我们需要确保传入的参数能匹配当前的实际物理位置
            # time_down = real_phase1_down(
            #     rpc, pos_data, speed_cm_s=42.0, descend_duration=0.40,
            #     descend_height_mm=cfg_1.down_height, clamp_delay_s=0.55,
            #     follow_after_clamp_s=0.30, movej_vel=40.0, control_freq_hz=200,
            #     DIRECTION=DIRECTION_1,robot_lock=task_lock_1
            # )
            # time33 = time.perf_counter()
            
            # # 5. 再次移动到安全点或中间点
            # move_to_safe_position_add_cloth_lenth3(rpc, safe_pos,robot_lock=task_lock_1)

            # 6. 二次等待时间校准
            # 重新计算 wait_time，因为上面执行了一些动作
            # 注意：这里的逻辑原代码可能有点问题，通常 move_to_safe 之后就要准备抓了
            # 如果 wait_time 还是正数，说明我们动作太快了，需要等物体过来
            wait_time_final = cfg_1.time_pre - (time.perf_counter() - motion_dict[mid]['line2_time'])
            
            if wait_time_final > 0:
                # print(f"动作超前，等待物体到位: {wait_time_final:.3f}s")
                time.sleep(wait_time_final)
            elif wait_time_final < TASK_EXPIRATION_THRESHOLD:
                 # 极端情况：移动到一半，物体跑了
                 print(f"[严重过期] 准备抓取时发现物体已跑远: {wait_time_final:.3f}s")
                 # 这里可以选择继续尝试(赌一把)或者放弃，建议继续尝试，因为已经到这步了
            
            print("抓取点真实坐标##################",robot_coord_test)
            print("抓取点预测坐标%%%%%%%%%%%%%%%%%%",pos_data)
            
            # 7. 动态跟随抓取,带异常检测 (Phase 2)
            res, pos, err = follow_and_grasp_dynamic_smooth_with_detect(
                rpc, pos_data, speed_cm_s=48.0, descend_duration=0.30,
                # descend_height_mm=cfg_1.down_height, clamp_delay_s=0.55,
                descend_height_mm=cfg_1.down_height, clamp_delay_s=0.1,
                follow_after_clamp_s=0.30, movej_vel=40.0, control_freq_hz=200,
                place_pos=place_pos, safe_pos2=cfg_1.safe_pos_2, cloth_length=cloth_lenth,safe_pos_1_5=cfg_1.safe_pos_1_5,
                DIRECTION=DIRECTION_1,robot_lock=task_lock_1,
                detect_pose=cfg_1.detect_pose, detect_none_pose=cfg_1.detect_none_pose,
                if_grip=False
                # if_grip=True
            )

            
            result='失败'
            if res==1:
                result='成功'
                tl_cur_pos_array = rpc.robot_state_pkg.tl_cur_pos
                
                tl_cur_pos_list = [tl_cur_pos_array[i] for i in range(6)]
                print("tl_cur_pos_list@@@@@@@@@@@@@@@",tl_cur_pos_list)
                place_and_move_to_safe(rpc, cfg_1.safe_pos_2, tl_cur_pos_list)
                update_count_and_next_placement_height()
                update_centroid_time()
            elif res==2:
                print(f"id-{mid} 抓取动作执行结果: 未检测到布料，放弃本次抓取。\n")
            else:
                print(f"id-{mid} 抓取动作执行失败: {err}")
            
            stop_suction(rpc)
            
            sum_real_done =get_count_arm1()
            LOG_INFO(f"====================1号臂: "
                        f"当前处理结果: {result} ; "
                        f"总检测量： {sum_detectnum}; "
                        f"实际成功总量:{sum_real_done}; "
                        f"积压抛弃总量{sum_abort}; "
                        f"机械臂处理数量: {sum_to_do} ; "
                        f"机械臂处理失败总量: {sum_to_do-sum_real_done} ; "
                        f"====================")

        except Exception as e:
            print(f"任务执行异常: {e}")
            import traceback
            traceback.print_exc()
        finally:
            # 标记当前单个任务完成（对应最开始 get 的那个）
            task_queue_1.task_done()
            with task_lock_1:
                # 再次检查队列是否真的空了
                if task_queue_1.empty():
                    is_processing_1 = False
                    
def process_tasks_2(rpc):
    """
    处理抓取任务的线程函数（智能过滤版）
    """
    global is_processing_2, DIRECTION_2
    # place_pos = [400.455, 460.545, 280.548, -180, -0.2, -85.458]
    # place_pos = [429.31, -468.68, 280, 178.894, -0.425, -149.585]
    place_pos = place_pos_arm_2
    sum_detectnum=0
    sum_abort=0
    sum_to_do=0
    print(">>> 机械臂处理线程已启动，等待任务...")

    while True:
        # 1. 获取任务（阻塞等待，直到至少有一个任务）
        item = task_queue_2.get()
        
        # 退出信号检查
        if item[0] is None:
            secondary_task_queue.put((None, None))
            task_queue_2.task_done()
            break

        # 2. 【核心修改】智能任务清洗逻辑
        # 机械臂刚忙完，现在看看队列里是不是堆了一堆旧任务？
        # 我们把它们全取出来，只挑最好的一个。
        latest_valid_item = item # 默认当前取到的这个
        dropped_count = 0
        sum_detectnum += 1
        # 锁定队列，把里面积压的任务全拿出来检查
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
                    
                    # 比较时间戳，判断 next_item 是否比 latest_valid_item 更“值得”抓
                    # 这里的逻辑是：既然我们已经落后了，直接丢弃旧的，只看最新的
                    # (注意：task_queue.task_done() 必须调用，否则 join 会阻塞)
                    task_queue_2.task_done() 
                    
                    # 将旧的 latest_valid_item 丢弃，更新为 next_item
                    # 这里假设队列后面的肯定是更新的检测结果
                    latest_valid_item = next_item
                    dropped_count += 1
                    sum_detectnum += 1
                    sum_abort+=1
                    
                except queue.Empty:
                    break
        
        if dropped_count > 0:
            LOG_INFO(f"[智能过滤] 丢弃了 {dropped_count} 个积压的旧任务，直接处理最新任务。")

        # 重新赋值 item 为清洗后的最新任务
        item = latest_valid_item
        if item[0] is None: break #再一次检查退出

        motion_dict, mid, pos_data, safe_pos, mode = item
        
        try:
            # 3. 【核心修改】过期时间检查 (Time Check)
            # 在移动机械臂之前，先算一下是否来得及
            # time_pre 是配置的预估到达时间，line2_time 是检测线触发时间
            # arrival_time = line2_time + time_pre
            arrival_time = motion_dict[mid]['line2_time'] + cfg_2.time_pre
            current_time = time.perf_counter()
            wait_time = arrival_time - current_time

            print(f"id-{mid} 预判检查: wait_time={wait_time:.3f}s")

            # 如果 wait_time 小于阈值（例如 -1.0s），说明物体已经过了很久了
            if wait_time < TASK_EXPIRATION_THRESHOLD:
                print(f"[过期跳过] id-{mid} 已过期 {abs(wait_time):.2f}s，放弃本次抓取，立即寻找下一个。")
                is_processing_2 = False # 标记为空闲
                sum_abort += 1
                continue # 直接跳过，进入下一次循环
            
            # --- 任务有效，开始执行 ---
            sum_to_do += 1
            cloth_lenth = motion_dict[mid]['long_side_length']
            print(f"id-{mid} 开始执行 | 布料长度: {cloth_lenth:.2f}mm")
            
            time11 = time.perf_counter()
            success, err = move_to_safe_position_add_cloth_lenth(rpc, safe_pos, motion_dict, mid, cloth_lenth,robot_lock=task_lock_2)
            time22 = time.perf_counter()
            # print("Move Safe Time:", time22 - time11)
            
            if not success:
                print(f"id-{mid} b移动到安全点失败: {err}")
                continue

            # # 4. 精确下降 (Phase 1)
            # # 注意：real_phase1_down 内部使用了 time.sleep 来做时间同步
            # # 我们需要确保传入的参数能匹配当前的实际物理位置
            # time_down = real_phase1_down(
            #     rpc, pos_data, speed_cm_s=42.0, descend_duration=0.40,
            #     descend_height_mm=cfg_2.down_height, clamp_delay_s=0.55,
            #     follow_after_clamp_s=0.30, movej_vel=40.0, control_freq_hz=200,
            #     DIRECTION=DIRECTION_2
            # )
            # time33 = time.perf_counter()
            
            # # 5. 再次移动到安全点或中间点
            # move_to_safe_position_add_cloth_lenth3(rpc, safe_pos)

            # 6. 二次等待时间校准
            # 重新计算 wait_time，因为上面执行了一些动作
            # 注意：这里的逻辑原代码可能有点问题，通常 move_to_safe 之后就要准备抓了
            # 如果 wait_time 还是正数，说明我们动作太快了，需要等物体过来
            # wait_time_final = cfg_2.time_pre - (time.perf_counter() - motion_dict[mid]['line2_time'])-4 # G 为了两臂同时
            wait_time_final = cfg_2.time_pre - (time.perf_counter() - motion_dict[mid]['line2_time'])
            
            if wait_time_final > 0:
                # print(f"动作超前，等待物体到位: {wait_time_final:.3f}s")
                time.sleep(wait_time_final)
            elif wait_time_final < TASK_EXPIRATION_THRESHOLD:
                 # 极端情况：移动到一半，物体跑了
                 print(f"[严重过期] 准备抓取时发现物体已跑远: {wait_time_final:.3f}s")
                 # 这里可以选择继续尝试(赌一把)或者放弃，建议继续尝试，因为已经到这步了

            # 7. 动态跟随抓取,带异常检测 (Phase 2)
            res, pos, err = follow_and_grasp_dynamic_smooth_with_detect_arm2(
                # rpc, pos_data, speed_cm_s=42.0, descend_duration=0.40,
                rpc, pos_data, speed_cm_s=42.0, descend_duration=0.30,
                descend_height_mm=cfg_2.down_height, clamp_delay_s=0.55,
                follow_after_clamp_s=0.30, movej_vel=40.0, control_freq_hz=200,
                place_pos=place_pos, safe_pos2=cfg_2.safe_pos_2, cloth_length=cloth_lenth,
                DIRECTION=DIRECTION_2,robot_lock=task_lock_2,
                detect_pose=cfg_2.detect_pose, detect_none_pose=cfg_2.detect_none_pose,
                if_grip=False
            )
            result='失败'
            if res==1:
                result='成功'
                tl_cur_pos_array = rpc.robot_state_pkg.tl_cur_pos
                tl_cur_pos_list = [tl_cur_pos_array[i] for i in range(6)]
                place_and_move_to_safe_arm2(rpc, cfg_2.safe_pos_2, tl_cur_pos_list)
                update_count_and_next_placement_height_arm2()
                update_centroid_time_arm2()
            elif res==2:
                print(f"id-{mid} 抓取动作执行结果: 未检测到布料，放弃本次抓取。\n")
            else:
                print(f"id-{mid} 抓取动作执行失败: {err}")

            sum_real_done =get_count_arm2()
            LOG_INFO(f"====================2号臂: "
                        f"当前处理结果: {result} ; "
                        f"总检测量： {sum_detectnum}; "
                        f"实际成功总量:{sum_real_done}; "
                        f"积压抛弃总量{sum_abort}; "
                        f"机械臂处理数量: {sum_to_do} ; "
                        f"机械臂处理失败总量: {sum_to_do-sum_real_done} ; "
                        f"====================")
        except Exception as e:
            print(f"任务执行异常: {e}")
            import traceback
            traceback.print_exc()
        finally:
            # 标记当前单个任务完成（对应最开始 get 的那个）
            task_queue_2.task_done()
            with task_lock_2:
                # 再次检查队列是否真的空了
                if task_queue_2.empty():
                    is_processing_2 = False
                    

# --- 主函数 main 修改 ---
def main():
    global AFFINE_MATRIX_1,AFFINE_MATRIX_2,is_processing_1,is_processing_2, be_ignored

    # 初始化两个机械臂（假设init_robot支持指定IP，若同IP则传相同参数）
    robot_ip1="192.168.57.4"
    robot_ip2="192.168.57.3"
    # robot_mode=3
    
    rpc_1=[]
    rpc_2=[]
    if robot_mode==1:
        rpc_1 = init_robot(ip=robot_ip1,arm=1)  # 1号机械臂IP
        if not (rpc_1):
            print("机器人初始化失败")
            return
        move_to_safe_position_add_cloth_lenth3(rpc_1, cfg_1.safe_pos_2,robot_lock=task_lock_1)
    elif robot_mode==2:
        rpc_2 = init_robot(ip=robot_ip2,arm=2)  # 2号机械臂IP
        if not (rpc_2):
            print("机器人初始化失败")
            return
        move_to_safe_position_add_cloth_lenth3(rpc_2, cfg_2.safe_pos_2,robot_lock=task_lock_2)
    elif robot_mode==3:
        rpc_1 = init_robot(ip=robot_ip1,arm=1)  # 1号机械臂IP
        rpc_2 = init_robot(ip=robot_ip2,arm=2)  # 2号机械臂IP（根据实际调整）#G 待 右臂需要跟丝杆做绑定
        # rpc_2 = 1 #G 临时测试用
        if not (rpc_1 and rpc_2):
            print("机器人初始化失败")
            return
        move_to_safe_position_add_cloth_lenth3(rpc_1, cfg_1.safe_pos_2,robot_lock=task_lock_1)
        move_to_safe_position_add_cloth_lenth3(rpc_2, cfg_2.safe_pos_2,robot_lock=task_lock_2)  

    # 2. 初始化相机（使用新的CameraHandler类）
    try:
        camera = CameraHandler(
            is_real_sense=cfg_1.is_real_sense,
            camera_no=cfg_1.camera_no,
            warmup_frames=30  # 前30帧预热
        )
        print("相机初始化完成")
    except Exception as e:
        print(f"相机初始化失败: {e}")
        return

    # 3. 加载坐标标定并计算仿射矩阵
    # ROBOT_POINTS, REAL_PTS = get_points(cfg.points_file_path)
    # AFFINE_MATRIX = vu.compute_affine_transform(REAL_PTS, ROBOT_POINTS)
    ROBOT_POINTS_1, REAL_PTS_1 = get_points(cfg_1.points_file_path)
    ROBOT_POINTS_2, REAL_PTS_2 = get_points(cfg_2.points_file_path)
    AFFINE_MATRIX_1 = vu.compute_affine_transform(REAL_PTS_1, ROBOT_POINTS_1)
    AFFINE_MATRIX_2 = vu.compute_affine_transform(REAL_PTS_2, ROBOT_POINTS_2) # G 需要根据右侧机械臂调整
     
    
    print("仿射矩阵计算完成")

    # 4. 初始化对象检测器
    detector = ObjectDetector(cfg_1, IS_USE_SAM)

    # 启动任务线程
    # threading.Thread(target=process_tasks, args=(rpc,), daemon=True).start()
    if robot_mode==1:
        threading.Thread(target=process_tasks_1, args=(rpc_1,), daemon=True).start()
    elif robot_mode==2:
        threading.Thread(target=process_tasks_2, args=(rpc_2,), daemon=True).start()
    elif robot_mode==3:
        threading.Thread(target=process_tasks_1, args=(rpc_1,), daemon=True).start()
        threading.Thread(target=process_tasks_2, args=(rpc_2,), daemon=True).start()
    else:
        print("请选择正确的模式")
    print("开始主循环...")
    
    try:
        while True:
            time_start = time.perf_counter()
            
            # 获取最新帧（使用新的相机类）
            frame = camera.get_frame()
            if frame is None:
                print("未能获取有效帧，跳过本次循环")
                continue
                    
            current_time = time.perf_counter()
            
            # 让检测器递增帧计数器
            detector.increment_frame_counter()

            # 尝试初始化背景
            detector.initialize_background(frame)
            warped_frame, mask, display_img = detector.detect_and_track(frame, AFFINE_MATRIX_1, current_time)# G 这里AFFINE_MATRIX是用来计算机械臂的移动方向（当前并未使用，使用的是写死的值）
            
            
            # 获取检测结果
            motion_dict = detector.get_motion_dict()
            tracked_objects = detector.get_tracked_objects()
            # print("111111111111111")
            # 6. 生成抓取任务
            to_del = []
            for mid in motion_dict:
                # print("22222222222222222")
                status = motion_dict[mid]['status']
                # if status == 2 and mid not in [t[1] for t in list(task_queue.queue) if t[1] is not None]:
                if (status == 2 
                and mid not in [t[1] for t in list(task_queue_1.queue) if t[1] is not None] 
                and mid not in [t[1] for t in list(task_queue_2.queue) if t[1] is not None]): # G
                    center = motion_dict[mid]['motion_center']
                    
                    # center = motion_dict[mid]['motion_center']
                    # rz_use = cfg.stand_rz + motion_dict[mid]['angle']
                    # if rz_use > 180: rz_use -= 360
                    # if rz_use < -180: rz_use += 360
                    # center = motion_dict[mid]['motion_center']
                    # mapped_robot_x, mapped_robot_y = vu.transform_point(AFFINE_MATRIX, (center[0] + cfg.speed * cfg.time_pre * cfg.ratio_wh, center[1]))
                    # target_safe_pos = cfg.safe_pos_init.copy()
                    # target_safe_pos[0] = mapped_robot_x
                    # target_safe_pos[1] = mapped_robot_y
                    # target_safe_pos[5] = rz_use
                    # robot_coord = [mapped_robot_x, mapped_robot_y, cfg.stand_z, cfg.stand_rx, cfg.stand_ry, rz_use]
                    # with task_lock:
                    #     task_queue.put((motion_dict, mid, robot_coord, target_safe_pos, "follow"))
                    #     is_processing = True
                    # motion_dict[mid]['status'] = 3
                    # print(f"id-{mid} 加入抓取队列")
                    print(f"============{mid}=============")
                    print("+++++++++++center=========:",center)
                    print("edge_1_in:",edge_1_in)
                    print("edge_1_out",edge_1_out)
                    print("edge_2_in",edge_2_in)
                    print("edge_2_out",edge_2_out)
                    # G根据双臂进行修改
                    if edge_1_in <= center[1]<=edge_1_out: # G 当前（左侧机械臂）
                    # if edge_2_in<= center[1]<edge_2_out : # G 新增（右侧机械臂）

                        print("33333333333333333")

                        rz_use = cfg_1.stand_rz + motion_dict[mid]['angle']
                        if rz_use > 180: rz_use -= 360
                        if rz_use < -180: rz_use += 360
                        # mapped_robot_x, mapped_robot_y = vu.transform_point(AFFINE_MATRIX_1, (center[0] + cfg_1.speed * cfg_1.time_pre * cfg_1.ratio_wh, center[1]))
                        mapped_robot_x_test, mapped_robot_y_test=vu.transform_point(AFFINE_MATRIX_1, (center[0] + 75*cfg_1.ratio_wh, center[1]))

                        mapped_robot_x, mapped_robot_y = vu.transform_point(AFFINE_MATRIX_1, (center[0] , center[1]))
                        print("original: center[0]: ", center[0])
                        print("original: center[1]: ", center[1])
                        print("original: mapped_robot_x: ", mapped_robot_x)
                        print("original: mapped_robot_y: ", mapped_robot_y)
                        mapped_robot_x= mapped_robot_x + cfg_1.WAIT_DISTANCE * DIRECTION_1[0] * 10
                        mapped_robot_y= mapped_robot_y + cfg_1.WAIT_DISTANCE * DIRECTION_1[1] * 10

                        target_safe_pos = cfg_1.safe_pos_init.copy() # G
                        target_safe_pos[0] = mapped_robot_x
                        target_safe_pos[1] = mapped_robot_y
                        target_safe_pos[5] = rz_use
                        print("target_safe_pos: ", target_safe_pos)

                        robot_coord = [mapped_robot_x, mapped_robot_y, cfg_1.stand_z, cfg_1.stand_rx, cfg_1.stand_ry, rz_use]
                        robot_coord_test = [mapped_robot_x_test, mapped_robot_y_test, cfg_1.stand_z, cfg_1.stand_rx, cfg_1.stand_ry, rz_use]
                        print("robot_coord: ", robot_coord)


                        # ===============================
                        # mapped_robot_x, mapped_robot_y = vu.transform_point(AFFINE_MATRIX_1, (center[0], center[1]))

                        # print(f"center[0], center[1]:{center[0], center[1]}")

                        # print(f"mapped_robot_x, mapped_robot_y:{mapped_robot_x, mapped_robot_y}")
                        # time.sleep(1000)
                        # ===============================

                        # 分配给1号机械臂，使用1号的初始安全位
                        with task_lock_1:
                            task_queue_1.put((motion_dict, mid, robot_coord, target_safe_pos, "follow",robot_coord_test)) # robot_coord（跟随运动出发位）和target_safe_pos（传送带静止拦截位）只有rx、ry不同
                            is_processing_1 = True

                        print(f"任务id-{mid} 分配至1号机械臂(左臂：面向相机时左侧) ")

                        motion_dict[mid]['status'] = 3
                        print(f"id-{mid} 加入抓取队列")
                    elif edge_2_in<= center[1]<edge_2_out : # G 新增（右侧机械臂）
                    # elif edge_1_in <= center[1]<=edge_1_out: # G 当前（左侧机械臂）

                        print("4444444444444444444444")
                         # G需要根据右侧臂进行调整！！！！！！！！！！！！！！！！！
                        rz_use = cfg_2.stand_rz + motion_dict[mid]['angle']
                        # rz_use = cfg.stand_rz2  #G temp
                        if rz_use > 180: rz_use -= 360
                        if rz_use < -180: rz_use += 360
                        # mapped_robot_x, mapped_robot_y = vu.transform_point(AFFINE_MATRIX2, (center[0] + cfg.speed * cfg.time_pre * cfg.ratio_wh, center[1]))
                        # print("cfg.speed * cfg.time_pre * cfg.ratio_wh",cfg_2.speed * cfg_2.time_pre * cfg_2.ratio_wh)  
                        # print("cfg.time_pre",cfg_2.time_pre)
                        # mapped_robot_x, mapped_robot_y = vu.transform_point(AFFINE_MATRIX_2, (center[0] + cfg_2.speed * 6 * cfg_2.ratio_wh, center[1])) # G 1230测试改cfg.time_pre
                        mapped_robot_x, mapped_robot_y = vu.transform_point(AFFINE_MATRIX_2, (center[0] + cfg_2.speed * cfg_2.time_pre * cfg_2.ratio_wh, center[1])) # G 1230测试改cfg.time_pre
                        target_safe_pos = cfg_2.safe_pos_init.copy()
                        target_safe_pos[0] = mapped_robot_x-50
                        # target_safe_pos[1] = mapped_robot_y
                        target_safe_pos[1] = mapped_robot_y+290
                        # target_safe_pos[1] = mapped_robot_y+120
                        target_safe_pos[5] = rz_use
                        # robot_coord = [mapped_robot_x, mapped_robot_y, cfg_2.stand_z, cfg_2.stand_rx, cfg_2.stand_ry, rz_use]
                        robot_coord = [mapped_robot_x-50, mapped_robot_y+290, cfg_2.stand_z, cfg_2.stand_rx, cfg_2.stand_ry, rz_use]
                        # robot_coord = [mapped_robot_x, mapped_robot_y+120, cfg_2.stand_z, cfg_2.stand_rx, cfg_2.stand_ry, rz_use]
                        print("2222222222222222222222 robot_coord:",robot_coord)

                        # 分配给2号机械臂，使用2号的初始安全位
                        with task_lock_2:
                            task_queue_2.put((motion_dict, mid, robot_coord, target_safe_pos, "follow"))
                            # task_queue2.put((motion_dict, mid, robot_coord,  cfg.safe_pos_init2, "follow"))  #GG temp
                            is_processing_2 = True

                        print(f"任务id-{mid} 分配至2号机械臂(右臂：面向相机时右侧) (y={ center[1]:.1f} )")

                        motion_dict[mid]['status'] = 3
                        print(f"id-{mid} 加入抓取队列")
                    else:
                        motion_dict[mid]['status'] = 3
                        be_ignored+=1
                        LOG_INFO(f"超出传送带范围{center[1]},总计超出检测区域数量{be_ignored}")


                if mid not in tracked_objects:
                    if time.perf_counter() - motion_dict[mid].get('last_seen', time.perf_counter()) > 60:
                        to_del.append(mid)
                    motion_dict[mid]['last_seen'] = time.perf_counter()
            
            
            for mid in to_del:
                motion_dict.pop(mid, None)

            # 绘制FPS
            fps = 1 / (time.perf_counter() - time_start)
            cv2.putText(display_img, f"FPS:{fps:.2f}", (cfg_1.output_w - 500, cfg_1.output_h - 20), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 8)

            # 显示或保存
            # combined_crop = display_img[:2532, :, :] if display_img.shape[0] > 2500 else display_img
            # combined_crop = cv2.resize(combined_crop, (combined_crop.shape[0]//2, combined_crop.shape[1]//2))
            display_img = cv2.resize(display_img, (display_img.shape[1]//3, display_img.shape[0]//3))
           

            # cv2.imwrite('lerobot/iros/DfgV1_2_dual_arm/imgs/combined1.jpg', combined_crop)
            # cv2.imwrite('/home/jack/fao/lerobot-main/src/lerobot/iros/DfgV1_2_dual_arm/imgs/combined1.jpg', combined_crop)
            # cv2.imwrite('imgs/combined1.jpg', display_img)
            # print("断点检测！！！！！！！！！！！！！！！")
            
            cv2.namedWindow('combined1', cv2.WINDOW_NORMAL)
            cv2.imshow('combined1', display_img)
            cv2.waitKey(1)

    finally:
        # 清理资源
        print("正在清理资源...")
        # task_queue.put((None, None, None, None, None))
        # task_queue.join()
        task_queue_1.put((None, None, None, None, None))
        task_queue_1.join()
        task_queue_2.put((None, None, None, None, None))
        task_queue_2.join()
        
        # 停止相机
        camera.stop()
        
        cv2.destroyAllWindows()
        if robot_mode==1:
            rpc_1.RobotEnable(0)
        elif robot_mode==2:
            rpc_2.RobotEnable(0)
        elif robot_mode==3:
            rpc_1.RobotEnable(0)
            rpc_2.RobotEnable(0)
        print("程序退出")

if __name__ == "__main__":
    main()
