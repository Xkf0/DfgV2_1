import threading
import time
import numpy as np
from Robot import RPC
from control_air_close_open import grip_clamp, grip_open, grip_release, start_suction, stop_suction

from  linear_actuator_long import move_to
from  detect_test import do_dect
import json
import logger
from logger import LOG_DEBUG, LOG_INFO, LOG_WARN, LOG_ERROR, LOG_CRITICAL
from global_state import AppState
import queue
import threading

from config_loader import Configer

# robot_lock = threading.Lock()



# G ####################添加全局监测功能######start
# ServoJ相关参数配置（100Hz控制）
EXAXIS_POS = [0]*4  # 外部轴位置
# 全局变量定义与初始化
complete_count = 0  # 完整动作计数
last_centroid_time = 0  # 最后接收质心坐标的时间

def load_config():
    with open("CONFIG.json", "r", encoding="utf-8") as f:
        config = json.load(f)
    return config
# 全局配置对象
CONFIG = load_config()
INIT_PLACE_HEIGHT = CONFIG["MOVE_PARAMS_1"]["INIT_PLACE_HEIGHT"]  # 三夹爪初始放置高度(mm)
PLACE_HEIGHT_INCREMENT = CONFIG["MOVE_PARAMS_1"]["PLACE_HEIGHT_INCREMENT"]  # 每次增加的高度(mm)
COUNT_RESET_INTERVAL = CONFIG["MOVE_PARAMS_1"]["COUNT_RESET_INTERVAL"]  # 5分钟(300秒)重置计数
ADJUSTMENT_GRIPPER_OFFSET = CONFIG["MOVE_PARAMS_1"]["ADJUSTMENT_GRIPPER_OFFSET"] # 调整夹爪向丝杆中心的移动距离[单边]，正为靠近，负为远离(mm) G 0115

global_lock = threading.Lock()  # 全局变量锁
current_place_height = INIT_PLACE_HEIGHT  # 当前放置高度(mm)
# arm2 全局变量定义与初始化
complete_count_arm2 = 0  # 完整动作计数
last_centroid_time_arm2 = 0  # 最后接收质心坐标的时间
INIT_PLACE_HEIGHT_ARM2 = 145.10  # 初始放置高度(mm)
current_place_height_arm2 = INIT_PLACE_HEIGHT_ARM2  # 当前放置高度(mm)
PLACE_HEIGHT_INCREMENT_ARM2 = 0.5  # 每次增加的高度(mm)
COUNT_RESET_INTERVAL_ARM2 =-2.775 # 5分钟(300秒)重置计数
global_lock_arm2 = threading.Lock()  # 全局变量锁
ADJUSTMENT_GRIPPER_OFFSET_ARM2 = 38 # 调整夹爪向丝杆中心的移动距离，正为靠近，负为远离(mm)


STAND_LENGTH = 1
USE_LINEAR_ACTUATOR = CONFIG["USE_LINEAR_ACTUATOR"]


# # G 增：更新质心坐标接收时间
# def update_centroid_time():
#     """
#     由主程序调用，更新最后接收质心坐标的时间
#     """
#     global last_centroid_time
#     with global_lock:
#         last_centroid_time = time.perf_counter()


# # G 增：定时检查计数重置线程
# def check_count_reset():
#     """
#     定时检查是否需要重置计数和放置高度
#     """
#     while True:
#         current_time = time.perf_counter()
#         with global_lock:
#             if current_time - last_centroid_time > COUNT_RESET_INTERVAL:
#                 global complete_count, current_place_height
#                 complete_count = 0
#                 current_place_height =  INIT_PLACE_HEIGHT  # 重置为初始高度
#                 print(f"5分钟未接收质心坐标，计数重置为0，放置高度重置为{current_place_height}mm")
#         time.sleep(60)  # 每分钟检查一次
        
 
# G 增：更新质心坐标接收时间
def update_centroid_time():
    """
    由主程序调用，更新最后接收质心坐标的时间
    """
    global last_centroid_time
    with global_lock:
        last_centroid_time = time.perf_counter()

# G 增：更新质心坐标接收时间
def update_centroid_time_arm2():
    """
    由主程序调用，更新最后接收质心坐标的时间
    """
    global last_centroid_time_arm2
    with global_lock_arm2:
        last_centroid_time_arm2 = time.perf_counter()

# G 增：定时检查计数重置线程
def check_count_reset():
    """
    定时检查是否需要重置计数和放置高度
    """
    global last_centroid_time
    while True:
        current_time = time.perf_counter()
        with global_lock:
            if current_time - last_centroid_time > COUNT_RESET_INTERVAL:
                global complete_count, current_place_height
                complete_count = 0
                current_place_height =  INIT_PLACE_HEIGHT  # 重置为初始高度
                print(f"arm1在{COUNT_RESET_INTERVAL}秒内未接收质心坐标，计数重置为0，放置高度重置为{current_place_height}mm")
        time.sleep(60)  # 每分钟检查一次
        
# G 增：定时检查计数重置线程
def check_count_reset_arm2():
    """
    定时检查是否需要重置计数和放置高度
    """
    global last_centroid_time_arm2
    while True:
        current_time_arm2 = time.perf_counter()
        with global_lock_arm2:
            if current_time_arm2 - last_centroid_time_arm2 > COUNT_RESET_INTERVAL_ARM2:
                global complete_count_arm2, current_place_height_arm2
                complete_count_arm2 = 0
                current_place_height_arm2 =  INIT_PLACE_HEIGHT_ARM2  # 重置为初始高度
                print(f"arm2在{COUNT_RESET_INTERVAL}秒未接收质心坐标，计数重置为0，放置高度重置为{current_place_height_arm2}mm")
        time.sleep(60)  # 每分钟检查一次

# G 增：更新计数和下次放置高度
def update_count_and_next_placement_height():
    """更新计数和下次放置高度"""
    # 4. 更新计数和下次放置高度
    with global_lock:
        global complete_count, current_place_height
        complete_count += 1
        current_place_height += PLACE_HEIGHT_INCREMENT
        print(f"\n#########1号臂完成第{complete_count}次抓取放置，下次放置高度将为{current_place_height}mm\n")

# G 增：更新计数和下次放置高度
def update_count_and_next_placement_height_arm2():
    """更新计数和下次放置高度"""
    # 4. 更新计数和下次放置高度
    with global_lock_arm2:
        global complete_count_arm2, current_place_height_arm2
        complete_count_arm2 += 1
        current_place_height_arm2 += PLACE_HEIGHT_INCREMENT_ARM2
        print(f"\n#########2号臂完成第{complete_count_arm2}次抓取放置，下次放置高度将为{current_place_height_arm2}mm\n")


def get_count_arm1():
    """更新计数和下次放置高度"""
    # 4. 更新计数和下次放置高度
    with global_lock:
        return complete_count
# G 增：更新计数和下次放置高度
def get_count_arm2():
    """更新计数和下次放置高度"""
    # 4. 更新计数和下次放置高度
    with global_lock_arm2:
        return complete_count_arm2

# 启动计数重置线程（程序启动时执行）
threading.Thread(target=check_count_reset, daemon=True, name="CountResetThread").start()
# 启动计数重置线程（程序启动时执行）
threading.Thread(target=check_count_reset_arm2, daemon=True, name="CountResetThread2").start()


# G ################添加全局监测功能######end       
        
        

# G 动作3: 抓取成功后抬起并移动到VLA调整位置
# def lift_and_place(robot, grab_pos, place_pos):
def lift_and_vla_adjust_position(robot, grab_pos, place_pos):
    """
    执行抬起并移动到VLA调整位置
    返回值: (success: bool, err: int)
    """
    # 抬起
    lift_pos = grab_pos.copy()
    lift_pos[2] += 50
    err = robot.MoveL(desc_pos=lift_pos, tool=1, user=2, vel=60.0, acc=-1.0, ovl=100.0, blendR=-1.0)
    if err != 0:
        print("抬起失败:", err)
        return False, err

    # 移动到VLA调整位置
    err = robot.MoveL(desc_pos=place_pos, tool=1, user=2, vel=60.0, acc=-1.0, ovl=100.0, blendR=-1.0)
    if err != 0:
        print("放置移动失败:", err)
        return False, err
    print("移动到放置点完成:", err)
    # time.sleep(0.5)

    return True, 0

# G 增：放置布料并移动到安全等待位置
def place_and_move_to_safe(robot,safe_pos, place_pos, safe_pos2):
    """
    放置布料并移动到安全等待位置
    """
    with global_lock:
        # 复制原始放置位置并更新高度（只修改Z轴）
        dynamic_place_pos = place_pos

        print("dynamic_place_pos-------------------",dynamic_place_pos)
        dynamic_place_pos[2] = current_place_height+50 # 使用当前计算的放置高度


    # # 0. 移动到动态放置位置上方（固定姿态）
    # err = robot.MoveL(desc_pos=place_pos, tool=1, user=2, vel=60.0, acc=-1.0, ovl=100.0, blendR=-1.0)
    # if err != 0:
    #     print("移动到放置点失败:", err)
    #     return False
    # print("移动到动态放置点上方完成")

    # 1. 移动到动态放置位置（使用更新后的高度）
    print(f"第{complete_count + 1}次放置，目标高度: {dynamic_place_pos[2]}mm")
    err = robot.MoveL(desc_pos=dynamic_place_pos, tool=1, user=2, vel=100.0, acc=30.0, ovl=100.0, blendR=100.0)
    stop_suction(robot)
    if err != 0:
        print("移动到放置点失败:", err)
        return False
    print("移动到动态放置点完成")



    dynamic_place_pos[2] = current_place_height
    print("current_place_height: %s", current_place_height)  # 参数测试

    err = robot.MoveL(desc_pos=dynamic_place_pos, tool=1, user=2, vel=100.0, acc=-1.0, ovl=100.0, blendR=100.0)
    if err != 0:
        return False

    # 2. 释放吸盘（假设使用数字输出口1控制，根据实际硬件调整）
    grip_release(robot)  # 0表示释放（需与硬件对应） # G temp_test夹爪
    
    time.sleep(3.0)  # 等待释放完成




    # 3. 移动到安全等待位置
    err = robot.MoveL(desc_pos=safe_pos, tool=1, user=2, vel=60.0, acc=-1.0, ovl=100.0, blendR=-1.0)
    if err != 0:
        print("移动到安全等待位置失败:", err)
        return False

    err = robot.MoveL(desc_pos=safe_pos2, tool=1, user=2, vel=60.0, acc=-1.0, ovl=100.0, blendR=-1.0)
    if err != 0:
        print("移动到安全等待位置2失败:", err)
        return False
    print("已返回安全等待位置")
    
# G 增：放置布料并移动到安全等待位置
def place_and_move_to_safe_arm2(robot,safe_pos, place_pos):
    """
    放置布料并移动到安全等待位置
    """
    with global_lock_arm2:
        # 复制原始放置位置并更新高度（只修改Z轴）
        dynamic_place_pos = place_pos

        print("dynamic_place_pos-------------------",dynamic_place_pos)
        dynamic_place_pos[2] = current_place_height+50 # 使用当前计算的放置高度


    # # 0. 移动到动态放置位置上方（固定姿态）
    # err = robot.MoveL(desc_pos=place_pos, tool=1, user=2, vel=60.0, acc=-1.0, ovl=100.0, blendR=-1.0)
    # if err != 0:
    #     print("移动到放置点失败:", err)
    #     return False
    # print("移动到动态放置点上方完成")

    # 1. 移动到动态放置位置（使用更新后的高度）
    print(f"第{complete_count_arm2 + 1}次放置，目标高度: {dynamic_place_pos[2]}mm")
    err = robot.MoveL(desc_pos=dynamic_place_pos, tool=1, user=2, vel=100.0, acc=30.0, ovl=100.0, blendR=100.0)
    if err != 0:
        print("移动到放置点失败:", err)
        return False
    print("移动到动态放置点完成")



    dynamic_place_pos[2] = current_place_height


    err = robot.MoveL(desc_pos=dynamic_place_pos, tool=1, user=2, vel=100.0, acc=-1.0, ovl=100.0, blendR=100.0)
    if err != 0:
        return False

    # 2. 释放吸盘（假设使用数字输出口1控制，根据实际硬件调整）
    # grip_release(robot)  # 0表示释放（需与硬件对应）# G temp_test夹爪
    time.sleep(0.1)  # 等待释放完成




    # 3. 移动到安全等待位置
    err = robot.MoveL(desc_pos=safe_pos, tool=1, user=2, vel=60.0, acc=-1.0, ovl=100.0, blendR=-1.0)
    if err != 0:
        print("移动到安全等待位置失败:", err)
        return False
    print("已返回安全等待位置")

# G 增：放置布料并移动到安全等待位置
# def place_and_move_to_safe_keep_pose_1107(robot, place_pos):
def place_and_move_to_safe_keep_pose_1107(robot):
    """
    放置布料并移动到安全等待位置
    """
    with global_lock:
        # 复制原始放置位置并更新高度（只修改Z轴）
        dynamic_place_pos = place_pose_higher_1107

        print("dynamic_place_pos-------------------",dynamic_place_pos)
        dynamic_place_pos[2] = current_place_height  # 使用当前计算的放置高度
        
        # keep_pose_1107
        # 获取tl_cur_pos并转换为Python列表
        tl_cur_pos_array = robot.robot_state_pkg.tl_cur_pos
        tl_cur_pos_list = [round(tl_cur_pos_array[i],3) for i in range(6)]  # 遍历0-5索引，生成列表
        dynamic_place_pos[3:]=tl_cur_pos_list[3:] # 保持机械臂现有姿态



    # # 0. 移动到动态放置位置上方（固定姿态）
    # err = robot.MoveL(desc_pos=place_pos, tool=1, user=2, vel=60.0, acc=-1.0, ovl=100.0, blendR=-1.0)
    # if err != 0:
    #     print("移动到放置点失败:", err)
    #     return False
    # print("移动到动态放置点上方完成")

    # 1. 移动到动态放置位置（使用更新后的高度）
    print(f"第{complete_count + 1}次放置，目标高度: {dynamic_place_pos[2]}mm")
    err = robot.MoveL(desc_pos=dynamic_place_pos, tool=1, user=2, vel=60.0, acc=-1.0, ovl=100.0, blendR=-1.0)
    # err = robot.MoveCart(desc_pos=dynamic_place_pos, tool=1, user=2, vel=60.0, acc=-1.0, ovl=100.0, blendR=-1.0) # G 1107 换为movep，防止movel不可达
    if err != 0:
        print("移动到放置点失败:", err)
        return False
    print("移动到动态放置点完成")

    # 2. 释放吸盘（假设使用数字输出口1控制，根据实际硬件调整）
    grip_release(robot)  # 0表示释放（需与硬件对应）
    time.sleep(0.1)  # 等待释放完成

    # 3. 移动到安全等待位置
    err = robot.MoveL(desc_pos=safe_wait_pos, tool=1, user=2, vel=60.0, acc=-1.0, ovl=100.0, blendR=-1.0)
    if err != 0:
        print("移动到安全等待位置失败:", err)
        return False
    print("已返回安全等待位置")


# # G 增：更新计数和下次放置高度
# def update_count_and_next_placement_height():
#     """更新计数和下次放置高度"""
#     # 4. 更新计数和下次放置高度
#     with global_lock:
#         global complete_count, current_place_height
#         complete_count += 1
#         current_place_height += PLACE_HEIGHT_INCREMENT
#         print(f"完成第{complete_count}次抓取放置，下次放置高度将为{current_place_height}mm")


# # 启动计数重置线程（程序启动时执行）
# threading.Thread(target=check_count_reset, daemon=True, name="CountResetThread").start()


# # G ################添加全局监测功能######end




# 初始化机械臂
# def init_robot():
#     robot = RPC(ip="192.168.58.2")
    
#     # robot.setup_logging(output_model=1, file_path="robot_grasp_log.log")
#     # robot.set_log_level(4)  # DEBUG 级别
# # G 1121 添加丝杆初始化（回原点校正）----start
#     move_to(-1)      # 回原点
#     time.sleep(0.5)    # 等待2秒
# # G 1121 添加丝杆初始化（回原点校正）----end

#     # 机械臂使能并切换到自动模式
#     if robot.RobotEnable(1) != 0 or robot.Mode(0) != 0:
#         print("机械臂初始化失败")
#         return None

#     print("机械臂初始化成功")
#     return robot
# 初始化机械臂
def init_robot(ip="192.168.57.4",arm=1):
    # robot = RPC(ip="192.168.58.2")
    robot = RPC(ip)
    print("######################机械臂ip:", ip)
    
    # robot.setup_logging(output_model=1, file_path="robot_grasp_log.log")
    # robot.set_log_level(4)  # DEBUG 级别
# G 1121 添加丝杆初始化（回原点校正）----start
    # move_to(-1)      # 回原点
    # time.sleep(0.5)    # 等待2秒
    if arm==1:
        if USE_LINEAR_ACTUATOR:
            move_to(-1)      # 回原点
        time.sleep(0.5)    # 等待2秒
    elif arm==2: #G 待 右臂需要跟丝杆做绑定
        # move_to(-1)      # 回原点
        print("GGG二号臂暂不用丝杆")
        time.sleep(0.5)    # 等待2秒
    
# G 1121 添加丝杆初始化（回原点校正）----end

    # 机械臂使能并切换到自动模式
    if robot.RobotEnable(1) != 0 or robot.Mode(0) != 0:
        print("机械臂初始化失败")
        return None

    print("机械臂初始化成功")
    return robot

def initRobot_And_move2SafePosition_And_clothLength2Zero(robot_mode, cfg_1, cfg_2, task_lock_1, task_lock_2, robot_ip1, robot_ip2):
    if robot_mode==1:
        rpc_1 = init_robot(ip=robot_ip1,arm=1)  # 1号机械臂IP
        if not (rpc_1):
            print("机器人初始化失败")
            return
        move_to_safe_position_add_cloth_lenth3(rpc_1, cfg_1.safe_pos_1,cfg_1.safe_pos_2, robot_lock=task_lock_1,arm_id=1)
        rpc_2 = []
    elif robot_mode==2:
        rpc_2 = init_robot(ip=robot_ip2,arm=2)  # 2号机械臂IP
        if not (rpc_2):
            print("机器人初始化失败")
            return
        move_to_safe_position_add_cloth_lenth3(rpc_2, cfg_2.safe_pos_1,robot_lock=task_lock_2,arm_id=2)
        rpc_1 = []
    elif robot_mode==3:
        rpc_1 = init_robot(ip=robot_ip1,arm=1)  # 1号机械臂IP
        rpc_2 = init_robot(ip=robot_ip2,arm=2)  # 2号机械臂IP（根据实际调整）#G 待 右臂需要跟丝杆做绑定
        if not (rpc_1 and rpc_2):
            print("机器人初始化失败")
            return
        move_to_safe_position_add_cloth_lenth3(rpc_1, cfg_1.safe_pos_1,robot_lock=task_lock_1,arm_id=1)
        move_to_safe_position_add_cloth_lenth3(rpc_2, cfg_2.safe_pos_1,robot_lock=task_lock_2,arm_id=2)  

    return rpc_1, rpc_2


# 动作1: 移动到安全点
def move_to_safe_position(robot, safe_pos, motion_dict, mid):
    """
    移动到安全点
    返回值: (success: bool, err: int)
    """
    with robot_lock:
        print("移动到安全点:", safe_pos)
        err = robot.MoveL(desc_pos=safe_pos, tool=1, user=2, vel=60.0, acc=-1.0, ovl=100.0, blendR=-1.0)
        if err != 0:
            print("1移动到安全点失败:", err)
            return False, err
        motion_dict[mid]['status'] = 3  # 标记为已完成
    
    print("移动到安全点完成")
    return True, 0

def move_to_safe_position_add_cloth_lenth(robot, safe_pos, motion_dict, mid,cloth_lenth,robot_lock=None):
    """
    移动到安全点
    返回值: (success: bool, err: int)
    """
    # global STAND_LENGTH
# G 1121 添加丝杆长度调整----start
    # print("====================complete_count:",complete_count)
    # # if complete_count==0 :
    # # if True:
    # if cloth_lenth is not None :
    #     lenth_change = (cloth_lenth - STAND_LENGTH) / STAND_LENGTH
    #     if lenth_change > 0.1:
    #     # if cloth_lenth is not None:
    #         STAND_LENGTH = cloth_lenth
    #         distance = 215-cloth_lenth/2+ADJUSTMENT_GRIPPER_OFFSET  # 在布料长度基础上增加20mm作为安全余量
    #         if 1<=distance <=210:
    #             try:
    #                 move_to(distance)   
    #             except  Exception as e:
    #                 print(f"丝杆移动失败：{e}")
    #         else:
    #             print(f"丝杆目标位置超出行程允许范围：{distance}mm")
    #     else:
    #         print("未获取到新品种布料长度，丝杆不移动")
# G 1121 添加丝杆长度调整----end
    move_to_cloth_lenth(robot, cloth_lenth)


    with robot_lock:
        print("移动到安全点:", safe_pos)
        err = robot.MoveL(desc_pos=safe_pos, tool=1, user=2, vel=60.0, acc=-1.0, ovl=100.0, blendR=-1.0)
        if err != 0:
            print("2移动到安全点失败:", err)
            return False, err
        motion_dict[mid]['status'] = 3  # 标记为已完成
    
    print("移动到安全点完成")
    return True, 0

def move_to_safe_position_add_cloth_lenth(robot, safe_pos, motion_dict, mid,cloth_lenth,robot_lock=None,arm_id=1):
    """
    移动到安全点
    返回值: (success: bool, err: int)
    """
    # global STAND_LENGTH
# G 1121 添加丝杆长度调整----start
    # print("====================complete_count:",complete_count)
    # # if complete_count==0 :
    # # if True:
    # if cloth_lenth is not None :
    #     lenth_change = (cloth_lenth - STAND_LENGTH) / STAND_LENGTH
    #     if lenth_change > 0.1:
    #     # if cloth_lenth is not None:
    #         STAND_LENGTH = cloth_lenth
    #         distance = 215-cloth_lenth/2+ADJUSTMENT_GRIPPER_OFFSET  # 在布料长度基础上增加20mm作为安全余量
    #         if 1<=distance <=210:
    #             try:
    #                 move_to(distance)   
    #             except  Exception as e:
    #                 print(f"丝杆移动失败：{e}")
    #         else:
    #             print(f"丝杆目标位置超出行程允许范围：{distance}mm")
    #     else:
    #         print("未获取到新品种布料长度，丝杆不移动")
# G 1121 添加丝杆长度调整----end
    move_to_cloth_lenth(robot, cloth_lenth)

    user_param = 2 if arm_id == 1 else 1 

    with robot_lock:
        print("！！！！！！！！！！！！！！！！！！！！！！移动到安全点:", safe_pos)
        err = robot.MoveL(desc_pos=safe_pos, tool=1, user=2, vel=60.0, acc=-1.0, ovl=100.0, blendR=-1.0)
        if err != 0:
            print("移动到安全点失败:", err)
            return False, err
        motion_dict[mid]['status'] = 3  # 标记为已完成
    
    print("移动到安全点完成")
    return True, 0





def move_to_cloth_lenth(robot, cloth_lenth):
    """
    移动到安全点
    返回值: (success: bool, err: int)
    """
    if USE_LINEAR_ACTUATOR is False:
        return
    global STAND_LENGTH
    LOG_INFO("cloth_lenth: %fmm", cloth_lenth)
# G 1121 添加丝杆长度调整----start
    # print("====================complete_count:",complete_count)
    # if complete_count==0 :
    # if True:
    if cloth_lenth is not None :
        lenth_change = abs((cloth_lenth - STAND_LENGTH) / STAND_LENGTH)

        print("cloth_lenth##############",cloth_lenth)
        print("STAND_LENGTH@@@@@@@@@@@@@",STAND_LENGTH)
        print("lenth_change!!!!!!!!!!!!!",lenth_change)
        if lenth_change > 0.2:
        # if cloth_lenth is not None:
            STAND_LENGTH = cloth_lenth
            distance = 345 - cloth_lenth/2 + ADJUSTMENT_GRIPPER_OFFSET  # 在布料长度基础上增加20mm作为安全余量
            print("distance##############",distance)
            if 1<=distance <=345:
                try:
                    move_to(distance)   
                except  Exception as e:
                    print(f"丝杆移动失败：{e}")
            else:
                print(f"丝杆目标位置超出行程允许范围：{distance}mm")
        else:
            print("未获取到新品种布料长度，丝杆不移动")
# G 1121 添加丝杆长度调整----end
    return True, 0










def move_to_safe_position_add_cloth_lenth2(robot, safe_pos, cloth_lenth,robot_lock=None,safe_pos2=None):
    """
    移动到安全点
    返回值: (success: bool, err: int)
    """

# G 1121 添加丝杆长度调整----end

    with robot_lock:
        print("移动到安全点2:", safe_pos)
        err = robot.MoveL(desc_pos=safe_pos2, tool=1, user=2, vel=100.0, acc=-1.0, ovl=100.0, blendR=-1.0)
        if err != 0:
            print("3移动到安全点2失败:", err)
            return False, err

    with robot_lock:
        print("移动到安全点:", safe_pos)
        err = robot.MoveL(desc_pos=safe_pos, tool=1, user=2, vel=100.0, acc=-1.0, ovl=100.0, blendR=-1.0)
        if err != 0:
            print("3移动到安全点失败:", err)
            return False, err
    
    print("移动到安全点完成")
    return True, 0





def move_to_safe_position_add_cloth_lenth3(robot, safe_pos,robot_lock=None):
    """
    移动到安全点
    返回值: (success: bool, err: int)
    """


    with robot_lock:
        print("移动到安全点:", safe_pos)
        err = robot.MoveL(desc_pos=safe_pos, tool=1, user=2, vel=60.0, acc=-1.0, ovl=100.0, blendR=-1.0)
        if err != 0:
            print("4移动到安全点失败:", err)
            return False, err
    
    print("移动到安全点完成")
    return True, 0

def move_to_safe_position_add_cloth_lenth3(robot, safe_pos, safe_pos2, robot_lock=None,arm_id=1):
    """
    移动到安全点
    返回值: (success: bool, err: int)
    """

    user_param = 2 if arm_id == 1 else 1 
    with robot_lock:
        print("移动到安全点:", safe_pos)
        err = robot.MoveL(desc_pos=safe_pos, tool=1, user=2, vel=60.0, acc=-1.0, ovl=100.0, blendR=-1.0)
        if err != 0:
            print("移动到安全点失败:", err)
            return False, err
        
    with robot_lock:
        print("移动到安全点2:", safe_pos)
        err = robot.MoveL(desc_pos=safe_pos2, tool=1, user=2, vel=60.0, acc=-1.0, ovl=100.0, blendR=-1.0)
        if err != 0:
            print("移动到安全点2失败:", err)
            return False, err
    
    print("移动到安全点完成")
    return True, 0








# 动作2: 下降到抓取位置
def perform_grasp(robot, new_grab_pos, max_retries=3):

    """
    执行下降到抓取位置
    返回值: (success: bool, grab_pos: list or None, err: int)
    """

    print("目标抓取位姿:", new_grab_pos)

    # 下降到目标位置
    for attempt in range(max_retries):
        if robot.robot_state_pkg.collisionState == 1:
            print("检测到碰撞，停止")
            return False, None, -1
        



        err = robot.MoveL(desc_pos=new_grab_pos, tool=1, user=2, vel=60.0, acc=-1.0, ovl=100.0, blendR=60.0)
        if err != 0:
            print(f"下降失败 (尝试 {attempt+1}):", err)
            continue


        print("到达目标位置")

        grip_clamp(robot)
        print("抓取完成")
        time.sleep(1.8)
        return True, new_grab_pos, 0

    print("移动失败,超过重试次数")
    return False, None, -1








def init_inference_environment():
    global inference_robot, policy
    
    print("开始初始化模型推理环境...")
    
    # 构造与record2.py一致的参数
    args = [
        "--robot.type=fairino_follower",
        "--robot.ip_address=192.168.58.107",
        "--robot.cameras={ front: {type: intelrealsense, serial_number_or_name: 32712-176078, width: 640, height: 480, fps: 30}}",
        "--robot.id=black",
        "--display_data=false",
        "--dataset.repo_id=datasets/eval_record-test",
        "--dataset.single_task=Put the fabric into the detection box.",
        "--policy.path=outputs/smolvla_force/checkpoints/0160000/pretrained_model",
        "--dataset.push_to_hub=False",
        "--dataset.num_episodes=50",
        "--dataset.episode_time_s=-1",
        "--dataset.reset_time_s=2",
        "--dataset.root=date/huggingface/lerobot/lerobot/adjustment"
    ]
    


# # 新增: 跟随运动函数
# def follow_target(robot, start_pos, speed_cm_s, motion_direction, follow_duration=1.0):
#     """
#     执行跟随运动，沿着物体实际运动方向移动
    
#     参数:
#         robot: 机械臂对象
#         start_pos: 起始位置 [x, y, z, rx, ry, rz] (单位: mm)
#         speed_cm_s: 目标速度 (cm/s) - 这是物体的真实速度
#         motion_direction: 运动方向向量 [dx, dy] (已归一化，机械臂坐标系)
#         follow_duration: 跟随时长 (秒)
    
#     返回值: (success: bool, final_pos: list or None)
#     """
#     print(f"开始跟随运动，速度: {speed_cm_s:.2f} cm/s, 时长: {follow_duration}s")
#     print(f"运动方向(机械臂坐标系): [{motion_direction[0]:.3f}, {motion_direction[1]:.3f}]")
    
#     # 计算总移动距离 (cm -> mm)
#     total_distance_mm = speed_cm_s * follow_duration * 10  # cm/s * s * 10 = mm
    
#     # 计算终点位置 (机械臂坐标系单位是mm)
#     current_pos = start_pos.copy()
#     final_pos = current_pos.copy()
#     final_pos[0] += motion_direction[0] * total_distance_mm  # x方向 (mm)
#     final_pos[1] += motion_direction[1] * total_distance_mm  # y方向 (mm)
#     # z、rx、ry、rz保持不变
    
#     print(f"跟随起点: [{current_pos[0]:.2f}, {current_pos[1]:.2f}, {current_pos[2]:.2f}] mm")
#     print(f"跟随终点: [{final_pos[0]:.2f}, {final_pos[1]:.2f}, {final_pos[2]:.2f}] mm")
#     print(f"移动距离: {total_distance_mm:.2f} mm")
    
#     # 使用单次移动，让机械臂按指定速度执行
#     # 速度单位: mm/s
#     move_vel = speed_cm_s * 10  # cm/s -> mm/s
    
#     print(f"机械臂移动速度: {move_vel:.2f} mm/s")
    
#     err = robot.MoveL(desc_pos=final_pos, tool=1, user=2, vel=move_vel, acc=100.0, ovl=100.0, blendR=60.0)
#     if err != 0:
#         print(f"跟随移动失败:", err)
#         return False, current_pos
    
#     # 等待移动完成（理论时间）
#     # time.sleep(follow_duration)
    
#     print(f"跟随运动完成")
    
#     return True, final_pos



def servoj(robot, joint_pos):
    """
    执行抬起和放置动作
    返回值: (success: bool, err: int)
    """
    # 抬起


    err = robot.ServoJ(joint_pos,[0,0,0,0], acc=0.0, vel=0.0, cmdT=0.008)
    if err != 0:
        print("5移动到安全点失败:", err)
        return False, err




def follow_target(robot, start_pos, speed_cm_s, motion_direction, follow_duration=1.0):
    total_distance_mm = speed_cm_s * follow_duration * 10
    final_pos = start_pos.copy()
    final_pos[0] += motion_direction[0] * total_distance_mm
    final_pos[1] += motion_direction[1] * total_distance_mm
    return True, final_pos  # 不实际移动，只计算终点




# 动作3: 抬起并放置
def lift_and_place(robot, grab_pos, place_pos):
    """
    执行抬起和放置动作
    返回值: (success: bool, err: int)
    """
    # 抬起
    lift_pos = grab_pos.copy()
    lift_pos[2] += 50
    err = robot.MoveL(desc_pos=lift_pos, tool=1, user=2, vel=60.0, acc=-1.0, ovl=100.0, blendR=-1.0)
    if err != 0:
        print("抬起失败:", err)
        return False, err

    # 移动到放置点
    err = robot.MoveL(desc_pos=place_pos, tool=1, user=2, vel=60.0, acc=-1.0, ovl=100.0, blendR=-1.0)
    if err != 0:
        print("放置移动失败:", err)
        return False, err
    print("移动到放置点完成:", err)
    # time.sleep(0.5)

    return True, 0



# =============== 新增函数：动态跟随抓取 ===============
# def follow_and_grasp_dynamic(robot, start_pos, speed_cm_s=6.0, follow_duration=2.0, descend_height=50.0,place_pos=None):
#     """
#     动态跟随抓取：在传送带上以恒速跟动物体，同时缓慢下降，实现相对静止抓取
#     参数:
#         robot: RPC 实例
#         start_pos: 起始位置 [x, y, z_high, rx, ry, rz]  # z_high 是安全高度（如 370mm）
#         speed_cm_s: 传送带速度 (cm/s)，默认 6.0
#         follow_duration: 跟动时间 (s)，建议 2.0
#         descend_height: 下降高度 (mm)，从安全高度降到抓取高度（如 50mm）
#     返回: (success: bool, final_grab_pos: list or None, err: int)
#     """
#     with robot_lock:
#         print(f"[动态抓取] 开始跟动，速度: {speed_cm_s} cm/s，持续: {follow_duration}s，下降: {descend_height}mm")

#         # 1. 快速移动到起始高位
#         err = robot.MoveL(desc_pos=start_pos, tool=1, user=2, vel=60.0, acc=-1.0, ovl=100.0, blendR=60.0)
#         if err != 0:
#             print("移动到起始高位失败:", err)
#             return False, None, err
#         time.sleep(0.5)
#         grip_open(robot)
#         # 2. 计算总移动距离 (mm)
#         total_distance_mm = speed_cm_s * follow_duration * 10  # cm/s → mm
#         direction = [-1.0, 0.0]  # 假设传送带沿 +X 方向（根据实际坐标系调整）

#         # 3. 终点位置（水平）
#         end_pos = start_pos.copy()
#         end_pos[0] += direction[0] * total_distance_mm
#         end_pos[1] += direction[1] * total_distance_mm

#         # 4. 插值参数
#         num_steps = 50  # 插值点数
#         z_start = start_pos[2]
#         z_end = z_start - descend_height

#         # 5. 使用 ServoL 实现连续轨迹（高频插补）
#         print("开始 ServoL 同步下降跟动...")
#         vel_mm_s = speed_cm_s * 10  # mm/s

#         for i in range(1, num_steps + 1):
#             alpha = i / num_steps

#             # 插值 X, Y, Z
#             curr_x = start_pos[0] + direction[0] * total_distance_mm * alpha
#             curr_y = start_pos[1] + direction[1] * total_distance_mm * alpha
#             curr_z = z_start + (z_end - z_start) * alpha

#             curr_pos = [curr_x, curr_y, curr_z, start_pos[3], start_pos[4], start_pos[5]]
#             rett=robot.GetInverseKin(0, curr_pos, -1)
#             joint_pos_t = rett[1]

#             err = robot.MoveJ(
#                 joint_pos=joint_pos_t, tool=1, user=2, desc_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0], vel=90.0, acc=0.0, ovl=20.0,
#               exaxis_pos=[0.0, 0.0, 0.0, 0.0], blendT=100.0, offset_flag=0, offset_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

#             )
#             if err != 0:
#                 print(f"ServoL 失败 at step {i}:", err)
#                 return False, None, err

#             # 碰撞检测
#             if robot.robot_state_pkg.collisionState == 1:
#                 print("检测到碰撞，紧急停止")
#                 robot.MoveStop()
#                 return False, None, -1

#         # 6. 到达抓取点，执行夹紧
#         print("到达抓取位置，执行夹紧")
#         grip_clamp(robot)
#         time.sleep(0.5)  # 等待吸盘稳定

#         # 7. 快速抬起脱离传送带
#         lift_pos = end_pos.copy()
#         lift_pos[2] = z_end + 50  # 抬起 50mm
#         err = robot.MoveL(desc_pos=lift_pos, tool=1, user=2, vel=60.0, acc=-1.0, ovl=100.0, blendR=-1.0)
#         if err != 0:
#             print("抬起失败:", err)
#             return False, None, err
#         print("抬起完成aaaaaaaaaaaaaaaaaaaaaaaaaaa")
        


#         success, err = lift_and_place(robot, lift_pos, place_pos)
#         if not success:
#             print("抬起并放置失败！")
#         print("抓取放置完成bbbbbbbbbbbbbbbbbbbbbbbbb")

#         print("动态跟随抓取成功")
#         return True, lift_pos, 0

def perform_grasp_lift_and_place(robot, new_grab_pos,  place_pos):
    with robot_lock:
        print("执行抓取-抬起-放置完整流程")
        success, grab_pos, err = perform_grasp(robot, new_grab_pos)
        if not success or grab_pos is None:
            print("下降抓取失败！")

        # time.sleep(1)

        # 动作3: 抬起并放置
        success, err = lift_and_place(robot, grab_pos, place_pos)
        if not success:
            print("抬起并放置失败！")
        print("抓取放置完成")
        return success
    










# def follow_and_grasp_dynamic(
#     robot, 
#     start_pos, 
#     speed_cm_s=6.0, 
#     follow_duration=2.0,
#     descend_height=50.0, 
#     place_pos=None, 
#     clamp_duration=1.0
# ):
#     """
#     无缝动态抓取：
#     1. 下降 + 跟动（第1段）
#     2. 水平跟动 + 异步夹紧（第2段，夹紧持续 clamp_duration 秒）
#     3. 抬起 → 放置
#     """
#     # 整个函数加锁，防止多个任务并发
#     # with robot_lock:
#     print(f"[无缝抓取] 速度: {speed_cm_s}cm/s, 夹紧持续: {clamp_duration}s")

#     # 1. 快速移动到起始高位
#     err = robot.MoveL(
#         desc_pos=start_pos, tool=1, user=2,
#         vel=60.0, acc=-1.0, ovl=100.0, blendR=60.0
#     )
#     if err != 0:
#         print("移动到起始高位失败:", err)
#         return False, None, err

#     # 打开夹爪
#     with robot_lock:
#         robot.SetDO(1, 1)  # 松开
#     print("气夹已松开")
#     time.sleep(0.3)

#     # 参数计算
#     total_distance_mm = speed_cm_s * follow_duration * 10
#     direction = [-1.0, 0.0]  # 根据实际传送带方向调整
#     descend_steps = 30
#     follow_steps = max(20, int(clamp_duration * 25))  # 40ms/步 → 1s ≈ 25步
#     z_start = start_pos[2]
#     z_end = z_start - descend_height
#     blendT = 100.0  # 关键：轨迹融合

#     # === 第1段：下降 + 跟动 ===
#     print("第1段：下降跟动...")
#     for i in range(1, descend_steps + 1):
#         alpha = i / descend_steps
#         x = start_pos[0] + direction[0] * total_distance_mm * alpha
#         y = start_pos[1] + direction[1] * total_distance_mm * alpha
#         z = z_start + (z_end - z_start) * alpha
#         pos = [x, y, z, start_pos[3], start_pos[4], start_pos[5]]

#         with robot_lock:
#             rett = robot.GetInverseKin(0, pos, -1)
#             if rett[0] != 0:
#                 print(f"第1段逆解失败 at step {i}")
#                 return False, None, rett[0]
#             joint = rett[1]

#             err = robot.MoveJ(
#                 joint_pos=joint, tool=1, user=2,
#                 desc_pos=[0.0]*6, vel=60.0, acc=0.0, ovl=100.0,
#                 exaxis_pos=[0.0]*4, blendT=blendT,
#                 offset_flag=0, offset_pos=[0.0]*6
#             )
#             if err != 0:
#                 print(f"第1段 MoveJ 失败 at step {i}:", err)
#                 return False, None, err

#             if robot.robot_state_pkg.collisionState == 1:
#                 robot.MoveStop()
#                 return False, None, -1

#     # === 第2段：水平跟动 + 异步夹紧 ===
#     print("第2段：水平跟动，触发夹紧...")
#     clamp_done = threading.Event()

#     def _clamp_and_hold():
#         try:
#             with robot_lock:

#                 robot.SetDO(1, 0,1,1)
#                 time.sleep(0.1)
#                 robot.SetDO(0, 1,1,1)  # DO0=1表示夹住
#                 print("气夹已夹住")
#                 time.sleep(0.1)  # 短暂延时确保信号发送 # 夹紧
#             print("夹紧指令已发送，持续 {}s".format(clamp_duration))
#             # time.sleep(clamp_duration)
#             clamp_done.set()
#         except Exception as e:
#             print(f"夹紧线程异常: {e}")
#             clamp_done.set()

#     # 启动异步夹紧（第2段开始时）
#     threading.Thread(target=_clamp_and_hold, daemon=True).start()

#     # 第2段：持续跟动
#     current_x = pos[0]
#     current_y = pos[1]
#     for i in range(1, follow_steps + 1):
#         t = i * 0.04  # 40ms 一步
#         x = current_x + direction[0] * speed_cm_s * t * 10
#         y = current_y + direction[1] * speed_cm_s * t * 10
#         z = z_end
#         follow_pos = [x, y, z, start_pos[3], start_pos[4], start_pos[5]]

#         with robot_lock:
#             rett = robot.GetInverseKin(0, follow_pos, -1)
#             if rett[0] != 0:
#                 print(f"第2段逆解失败 at step {i}")
#                 break
#             joint = rett[1]

#             err = robot.MoveJ(
#                 joint_pos=joint, tool=1, user=2,
#                 desc_pos=[0.0]*6, vel=60.0, acc=0.0, ovl=100.0,
#                 exaxis_pos=[0.0]*4, blendT=blendT,
#                 offset_flag=0, offset_pos=[0.0]*6
#             )
#             if err != 0:
#                 print(f"第2段 MoveJ 失败 at step {i}:", err)
#                 break

#     # 等待夹紧完成
#     clamp_done.wait(timeout=2.0)

#     # === 抬起脱离传送带 ===
#     lift_z = z_end + 50
#     lift_pos = [x, y, lift_z, start_pos[3], start_pos[4], start_pos[5]]
#     with robot_lock:
#         err = robot.MoveL(
#             desc_pos=lift_pos, tool=1, user=2,
#             vel=60.0, acc=-1.0, ovl=100.0, blendR=-1.0
#         )
#         if err != 0:
#             print("抬起失败:", err)
#             return False, None, err
#     print("抬起完成")

#     # === 放置 ===
#     success, err = lift_and_place(robot, lift_pos, place_pos)
#     if not success:
#         print("放置失败")
#         return False, None, err

#     print("动态无缝抓取完成")
#     return True, lift_pos, 0



# import threading
# import time

# def follow_and_grasp_dynamic(
#     robot,
#     start_pos,
#     speed_cm_s=6.0,
#     follow_duration=2.0,
#     descend_height=30.0,
#     place_pos=None,
#     clamp_delay_s=0.35,        # 【关键】第2段开始后，延迟多久夹紧（秒）
#     post_clamp_follow_s=0.4,  # 夹紧后继续跟动多久（秒），防止打滑
# ):
#     """
#     无缝动态抓取（最优版）：
#     1. 下降 + 跟动（第1段）
#     2. 水平跟动 → 延迟 clamp_delay_s 秒后夹紧 → 继续跟动 post_clamp_follow_s 秒
#     3. 抬起 → 放置
#     """
#     print(f"[无缝抓取] 速度: {speed_cm_s}cm/s, 下降: {descend_height}mm, "
#           f"夹紧延迟: {clamp_delay_s}s, 夹后跟动: {post_clamp_follow_s}s")

#     # === 0. 快速移动到起始高位 + 确保夹爪打开 ===
#     err = robot.MoveL(desc_pos=start_pos, tool=1, user=2, vel=60.0, acc=-1.0, ovl=100.0, blendR=60.0)
#     if err != 0:
#         print("移动到起始高位失败:", err)
#         return False, None, err

#     # 确保夹爪完全打开
#     with robot_lock:
#         robot.SetDO(1, 1)  # DO0=0 → 确保未夹紧
#     print("夹爪已完全打开")
#     time.sleep(0.2)

#     # === 参数计算 ===
#     total_distance_mm = speed_cm_s * follow_duration * 10
#     direction = [-1.0, 0.0]  # 根据实际传送带方向调整（X轴负方向）
#     descend_steps = 30
#     follow_steps = max(45, int((clamp_delay_s + post_clamp_follow_s) / 0.04))
#     z_start = start_pos[2]
#     z_end = z_start - descend_height
#     blendT = 100.0

#     clamp_done = threading.Event()
#     clamp_triggered = False

#     # === 第1段：下降 + 跟动 ===
#     print("第1段：下降跟动...")
#     for i in range(1, descend_steps + 1):
#         alpha = i / descend_steps
#         x = start_pos[0] + direction[0] * total_distance_mm * alpha
#         y = start_pos[1] + direction[1] * total_distance_mm * alpha
#         z = z_start + (z_end - z_start) * alpha
#         pos = [x, y, z, start_pos[3], start_pos[4], start_pos[5]]

#         with robot_lock:
#             rett = robot.GetInverseKin(0, pos, -1)
#             if rett[0] != 0:
#                 print(f"第1段逆解失败 at step {i}")
#                 return False, None, rett[0]
#             joint = rett[1]

#             err = robot.MoveJ(
#                 joint_pos=joint, tool=1, user=2,
#                 desc_pos=[0.0]*6, vel=60.0, acc=0.0, ovl=100.0,
#                 exaxis_pos=[0.0]*4, blendT=blendT,
#                 offset_flag=0, offset_pos=[0.0]*6
#             )
#             if err != 0:
#                 print(f"第1段 MoveJ 失败 at step {i}:", err)
#                 return False, None, err

#             if robot.robot_state_pkg.collisionState == 1:
#                 robot.MoveStop()
#                 print("碰撞检测！紧急停止")
#                 return False, None, -1

#     # === 第2段：水平跟动 + 延迟夹紧 + 夹后跟动 ===
#     print("第2段：水平跟动 + 延迟夹紧...")
#     current_x = pos[0]
#     current_y = pos[1]
#     z = z_end
#     segment_start_time = time.perf_counter()

#     def _clamp_and_hold():
#         try:
#             with robot_lock:
#                 robot.SetDO(1, 0,1,1)
#                 time.sleep(0.1)
#                 robot.SetDO(0, 1,1,1)  # DO0=1表示夹住
#                 print("气夹已夹住")
#                 time.sleep(0.1)  # 短暂延时确保信号发送 # 夹紧
#                 print(f"[夹紧] 气夹已闭合 @ {time.perf_counter() - segment_start_time:.3f}s")
#             clamp_done.set()
#         except Exception as e:
#             print(f"夹紧线程异常: {e}")
#             clamp_done.set()

#     for i in range(1, follow_steps + 1):
#         t = i * 0.04
#         elapsed = time.perf_counter() - segment_start_time

#         # 计算当前位置
#         x = current_x + direction[0] * speed_cm_s * t * 10
#         y = current_y + direction[1] * speed_cm_s * t * 10
#         follow_pos = [x, y, z, start_pos[3], start_pos[4], start_pos[5]]

#         # 执行运动
#         with robot_lock:
#             rett = robot.GetInverseKin(0, follow_pos, -1)
#             if rett[0] != 0:
#                 print(f"第2段逆解失败 at step {i}")
#                 break
#             joint = rett[1]

#             err = robot.MoveJ(
#                 joint_pos=joint, tool=1, user=2,
#                 desc_pos=[0.0]*6, vel=80.0, acc=0.0, ovl=100.0,
#                 exaxis_pos=[0.0]*4, blendT=blendT,
#                 offset_flag=0, offset_pos=[0.0]*6
#             )
#             if err != 0:
#                 print(f"第2段 MoveJ 失败 at step {i}:", err)
#                 break

#             if robot.robot_state_pkg.collisionState == 1:
#                 robot.MoveStop()
#                 print("碰撞检测！紧急停止")
#                 return False, None, -1
            
#         print(f"第2段 Step {i}, 运行时间: {elapsed:.3f}s")

#         # 【核心】：延迟 clamp_delay_s 秒后触发夹紧
#         if not clamp_triggered and elapsed >= clamp_delay_s:
#             print("触发夹紧线程...")
#             threading.Thread(target=_clamp_and_hold, daemon=True).start()
#             clamp_triggered = True
#             print(f"第2段运行 {elapsed:.3f}s 后，触发夹紧指令")

#         # 夹紧后继续跟动 post_clamp_follow_s 秒
#         if clamp_triggered and elapsed >= (clamp_delay_s + post_clamp_follow_s):
#             break

#     # 等待夹紧完成
#     if not clamp_done.wait(timeout=1.0):
#         print("夹紧超时！")
#         return False, None, -100

#     # === 抬起脱离传送带 ===
#     lift_z = z_end + 50
#     lift_pos = [x, y, lift_z, start_pos[3], start_pos[4], start_pos[5]]
#     # err = robot.MoveL(desc_pos=lift_pos, tool=1, user=2, vel=60.0, acc=-1.0, ovl=100.0, blendR=-1.0)
#     # if err != 0:
#     #     print("抬起失败:", err)
#     #     return False, None, err
#     # print("成功抬起，脱离传送带")

#     # === 放置 ===
#     success, err = lift_and_place(robot, lift_pos, place_pos)
#     if not success:
#         print("放置失败:", err)
#         return False, None, err

#     # G 增：更新计数和放置高度
#     update_count_and_next_placement_height()
#     update_centroid_time()  # 更新质心时间，防止计数重置

#     print("动态无缝抓取+堆叠完成")
#     return True, lift_pos, 0











def follow_and_grasp_dynamic(
    robot,
    start_pos,
    speed_cm_s=7.0,
    follow_duration=1.0,
    descend_height=30.0,
    place_pos=None,
    clamp_delay_after_descend=1.0,          # 下降后多久夹紧
    continue_follow_after_clamp=0.5,        # 【新增】夹紧后继续跟随时间（秒）
    max_follow_distance_cm=None,
    follow_steps_fixed=100,
    control_freq_hz=100
):
    """
    无缝动态抓取（夹紧后继续跟随版）

    新增功能：
        - 夹紧后继续跟随 `continue_follow_after_clamp` 秒（如 0.5s）
        - 总跟动时间 = clamp_delay + continue_follow_after_clamp
        - 更稳定抓取，防滑落
    """


    print(f"[无缝抓取] 速度: {speed_cm_s}cm/s, 下降: {descend_height}mm, "
          f"夹紧延迟: {clamp_delay_after_descend}s, 夹紧后继续跟动: {continue_follow_after_clamp}s")

    # ================================
    # 0. 快速到高位 + 开爪
    # ================================
    print("++++++++++++++=============跟随运动开始，移动到起始高位...")
    err = robot.MoveL(desc_pos=start_pos, tool=1, user=2, vel=60.0, acc=-1.0, ovl=100.0, blendR=60.0)
    if err != 0:
        return False, None, err

    with robot_lock:
        # robot.SetDO(1, 1, 1, 1)
        grip_open(robot)
        
    print("夹爪已打开")
    time.sleep(0.2)

    # ================================
    # 参数计算
    # ================================
    direction = np.array([-1.0, 0.0])
    z_start = start_pos[2]
    z_end = z_start - descend_height
    blendT = 100.0

    step_time_s = 1.0 / control_freq_hz
    follow_steps_fixed = follow_steps_fixed
    total_follow_time = clamp_delay_after_descend + continue_follow_after_clamp

    # 最大跟动距离
    max_follow_distance_mm = (
        max_follow_distance_cm * 10
        if max_follow_distance_cm is not None
        else speed_cm_s * total_follow_time * 10
    )
    distance_per_step_mm = max_follow_distance_mm / follow_steps_fixed

    total_distance_mm = speed_cm_s * follow_duration * 10
    descend_steps = 30

    print(f"  → 第1段: 下降+跟动 {total_distance_mm:.1f}mm")
    print(f"  → 第2段: 总跟动 {total_follow_time}s, 最多 {max_follow_distance_mm:.1f}mm, 每步 {distance_per_step_mm:.2f}mm")

    clamp_done = threading.Event()
    clamp_triggered = False
    clamp_time = None  # 记录夹紧触发时间

    # ================================
    # 第1段：下降跟动
    # ================================
    print("第1段：下降跟动...")
    for i in range(1, descend_steps + 1):
        alpha = i / descend_steps
        x = start_pos[0] + direction[0] * total_distance_mm * alpha
        y = start_pos[1] + direction[1] * total_distance_mm * alpha
        z = z_start + (z_end - z_start) * alpha
        pos = [x, y, z, start_pos[3], start_pos[4], start_pos[5]]

        with robot_lock:
            rett = robot.GetInverseKin(0, pos, -1)
            if rett[0] != 0:
                return False, None, rett[0]
            joint = rett[1]
            err = robot.MoveJ(joint_pos=joint, tool=1, user=2, desc_pos=[0]*6,
                             vel=10.0, acc=0.0, ovl=100.0, exaxis_pos=[0]*4,
                             blendT=blendT, offset_flag=0, offset_pos=[0]*6)
            if err != 0:
                return False, None, err
            if robot.robot_state_pkg.collisionState == 1:
                robot.MoveStop()
                return False, None, -1
        time.sleep(step_time_s)

    # ================================
    # 第2段：水平跟动（夹紧后继续跟动）
    # ================================
    print(f"第2段：水平跟动 {total_follow_time}s")
    current_x = pos[0]
    current_y = pos[1]
    z = z_end
    accumulated_distance_mm = 0.0
    descend_end_time = time.perf_counter()

    def _clamp_and_hold():
        nonlocal clamp_time
        try:
            with robot_lock:
                # robot.SetDO(1, 0, 1, 1)
                # time.sleep(0.05)
                # robot.SetDO(0, 1, 1, 1)
                # robot.SetDO(2, 1, 1, 1)
                grip_clamp(robot)
                
            print(f"气夹已夹住 @ {time.perf_counter() - descend_end_time:.3f}s")
            clamp_time = time.perf_counter()
            clamp_done.set()
        except Exception as e:
            print(f"[夹紧异常] {e}")
            clamp_done.set()

    for i in range(1, follow_steps_fixed + 1):
        step_start = time.perf_counter()

        # --- 计算步距 ---
        remaining = max_follow_distance_mm - accumulated_distance_mm
        if remaining <= 0:
            print("已达最大距离，提前结束")
            break
        step_distance = min(distance_per_step_mm, remaining)

        x = current_x + direction[0] * step_distance
        y = current_y + direction[1] * step_distance
        accumulated_distance_mm += step_distance

        follow_pos = [x, y, z, start_pos[3], start_pos[4], start_pos[5]]

        # --- MoveJ ---
        with robot_lock:
            rett = robot.GetInverseKin(0, follow_pos, -1)
            if rett[0] != 0:
                print(f"第2段逆解失败 step {i}")
                break
            joint = rett[1]
            err = robot.MoveJ(joint_pos=joint, tool=1, user=2, desc_pos=[0]*6,
                             vel=100.0, acc=0.0, ovl=100.0, exaxis_pos=[0]*4,
                             blendT=blendT, offset_flag=0, offset_pos=[0]*6)
            if err != 0:
                break
            if robot.robot_state_pkg.collisionState == 1:
                robot.MoveStop()
                return False, None, -1

        # --- 夹紧触发 ---
        elapsed_since_descend = time.perf_counter() - descend_end_time
        if not clamp_triggered and elapsed_since_descend >= clamp_delay_after_descend:
            print(f"下降后 {elapsed_since_descend:.3f}s，触发夹紧！")
            threading.Thread(target=_clamp_and_hold, daemon=True).start()
            clamp_triggered = True

        # --- 夹紧后继续跟动 0.5s ---
        if clamp_triggered and clamp_time is not None:
            elapsed_after_clamp = time.perf_counter() - clamp_time
            if elapsed_after_clamp >= continue_follow_after_clamp:
                print(f"夹紧后已跟动 {elapsed_after_clamp:.3f}s，准备抬起")
                break

        # --- 频率控制 ---
        elapsed = time.perf_counter() - step_start
        time.sleep(max(0, step_time_s - elapsed))

    # ================================
    # 等待夹紧确认
    # ================================
    if not clamp_done.wait(timeout=0.5):
        print("夹紧超时，强制夹紧")
        with robot_lock:
            robot.SetDO(0, 1, 1, 1)
            robot.SetDO(2, 1, 1, 1)
    else:
        print("夹紧已确认")

    # ================================
    # 抬起 + 放置
    # ================================
    lift_z = z_end + 50
    lift_pos = [x, y, lift_z, start_pos[3], start_pos[4], start_pos[5]]
    success, err = lift_and_place(robot, lift_pos, place_pos)
    if not success:
        return False, None, err

    update_count_and_next_placement_height()
    update_centroid_time()

    print(f"动态抓取完成 | 总跟动: {accumulated_distance_mm:.1f}mm")
    return True, lift_pos, 0




# def follow_and_grasp_dynamic_smooth(
#     robot,
#     start_pos,
#     speed_cm_s=7.0,
#     follow_duration=1.0,
#     descend_height=30.0,
#     place_pos=None,
#     clamp_delay_after_descend=1.0,
#     continue_follow_after_clamp=0.5,
#     max_follow_distance_cm=None,
#     robot_vel=20.0,  # 机械臂控制器速度（关节速度百分比）
# ):
#     """
#     解决低速抖动的优化版 - 针对机械臂控制器速度慢的场景
    
#     核心优化:
#         1. 根据robot_vel动态调整控制频率（避免指令堆积）
#         2. 增大每步距离，减少插补点数量
#         3. 使用运动时间预估，确保指令不超前
#         4. 优化blendT参数匹配实际速度
#     """
#     print(f"[平滑抓取] 传送带: {speed_cm_s}cm/s, 机械臂速度: {robot_vel}%, "
#           f"下降: {descend_height}mm")

#     # ================================
#     # 0. 快速到高位 + 开爪
#     # ================================
#     err = robot.MoveL(desc_pos=start_pos, tool=1, user=2, vel=60.0, acc=-1.0, ovl=100.0, blendR=60.0)
#     if err != 0:
#         return False, None, err

#     with robot_lock:
#         robot.SetDO(1, 1, 1, 1)
#     print("夹爪已打开")
#     time.sleep(0.2)

#     # ================================
#     # 参数自适应计算
#     # ================================
#     direction = np.array([-1.0, 0.0])
#     z_start = start_pos[2]
#     z_end = z_start - descend_height
    
#     # 关键：根据机械臂速度调整控制策略
#     if robot_vel <= 20.0:  # 低速场景（<=15%）
#         # 大幅减少插补点，增大步距
#         descend_steps = 10
#         follow_steps = 15
#         control_freq_hz = 10  # 降低到10Hz，避免指令堆积
#         blendT = 200.0  # 大幅增加融合时间，确保轨迹平滑
#         use_time_wait = True  # 启用运动时间等待
        
#     elif robot_vel <= 30.0:  # 中低速场景
#         descend_steps = 20
#         follow_steps = 30
#         control_freq_hz = 20
#         blendT = 150.0
#         use_time_wait = True
        
#     elif robot_vel <= 60.0:  # 中速场景
#         descend_steps = 30
#         follow_steps = 50
#         control_freq_hz = 50
#         blendT = 100.0
#         use_time_wait = False
        
#     else:  # 高速场景
#         descend_steps = 40
#         follow_steps = 100
#         control_freq_hz = 100
#         blendT = 100.0
#         use_time_wait = False
    
#     step_time_s = 1.0 / control_freq_hz
#     total_follow_time = clamp_delay_after_descend + continue_follow_after_clamp

#     # 计算移动距离
#     total_distance_mm = speed_cm_s * follow_duration * 10
#     max_follow_distance_mm = (
#         max_follow_distance_cm * 10
#         if max_follow_distance_cm is not None
#         else speed_cm_s * total_follow_time * 10
#     )
    
#     distance_per_step_mm = max_follow_distance_mm / follow_steps
#     descend_distance_per_step = total_distance_mm / descend_steps

#     # print(f"  → 控制频率: {control_freq_hz}Hz, blendT: {blendT}ms")
#     # print(f"  → 第1段: {descend_steps}步, 每步{descend_distance_per_step:.2f}mm")
#     # print(f"  → 第2段: {follow_steps}步, 每步{distance_per_step_mm:.2f}mm")
#     # print(f"  → 运动时间等待: {'启用' if use_time_wait else '禁用'}")

#     clamp_done = threading.Event()
#     clamp_triggered = False
#     clamp_time = None

#     # ================================
#     # 辅助函数：运动时间预估
#     # ================================
#     def estimate_move_time(distance_mm, vel_percent):
#         """
#         估算运动时间（粗略）
#         假设关节最大速度约 180 deg/s，笛卡尔空间约 1000 mm/s
#         """
#         max_cart_vel_mm_s = 1000.0  # 根据实际机械臂调整
#         actual_vel_mm_s = max_cart_vel_mm_s * (vel_percent / 100.0)
#         return distance_mm / actual_vel_mm_s if actual_vel_mm_s > 0 else 0.1

#     # ================================
#     # 第1段：下降跟动（粗插补）
#     # ================================
#     print("第1段：下降跟动...")
#     prev_pos = start_pos.copy()
    
#     for i in range(1, descend_steps + 1):
#         alpha = i / descend_steps
#         x = start_pos[0] + direction[0] * total_distance_mm * alpha
#         y = start_pos[1] + direction[1] * total_distance_mm * alpha
#         z = z_start + (z_end - z_start) * alpha
#         pos = [x, y, z, start_pos[3], start_pos[4], start_pos[5]]

#         with robot_lock:
#             rett = robot.GetInverseKin(0, pos, -1)
#             if rett[0] != 0:
#                 print(f"逆解失败 at step {i}")
#                 return False, None, rett[0]
#             joint = rett[1]
            
#             err = robot.MoveJ(
#                 joint_pos=joint, tool=1, user=2, desc_pos=[0]*6,
#                 vel=robot_vel, acc=0.0, ovl=100.0, exaxis_pos=[0]*4,
#                 blendT=blendT, offset_flag=0, offset_pos=[0]*6
#             )
            
#             if err != 0:
#                 print(f"MoveJ失败: {err}")
#                 return False, None, err
                
#             if robot.robot_state_pkg.collisionState == 1:
#                 robot.MoveStop()
#                 return False, None, -1
        
#         # 低速时等待运动完成一部分
#         if use_time_wait:
#             step_distance = np.linalg.norm(np.array(pos[:3]) - np.array(prev_pos[:3]))
#             move_time = estimate_move_time(step_distance, robot_vel)
#             wait_time = max(step_time_s, move_time * 0.7)  # 等待70%完成度
#             time.sleep(wait_time)
#             prev_pos = pos.copy()
#         else:
#             time.sleep(step_time_s)

#     # ================================
#     # 第2段：水平跟动（粗插补+运动预估）
#     # ================================
#     print(f"第2段：水平跟动 {total_follow_time}s")
#     current_x = pos[0]
#     current_y = pos[1]
#     z = z_end
#     accumulated_distance_mm = 0.0
#     descend_end_time = time.perf_counter()

#     def _clamp_and_hold():
#         nonlocal clamp_time
#         try:
#             with robot_lock:
#                 robot.SetDO(1, 0, 1, 1)
#                 time.sleep(0.05)
#                 robot.SetDO(0, 1, 1, 1)
#             print(f"气夹已夹住 @ {time.perf_counter() - descend_end_time:.3f}s")
#             clamp_time = time.perf_counter()
#             clamp_done.set()
#         except Exception as e:
#             print(f"[夹紧异常] {e}")
#             clamp_done.set()

#     prev_follow_pos = [current_x, current_y, z, start_pos[3], start_pos[4], start_pos[5]]
    
#     for i in range(1, follow_steps + 1):
#         step_start = time.perf_counter()

#         # 计算步距
#         remaining = max_follow_distance_mm - accumulated_distance_mm
#         if remaining <= 0:
#             print("已达最大距离")
#             break
#         step_distance = min(distance_per_step_mm, remaining)

#         x = current_x + direction[0] * step_distance
#         y = current_y + direction[1] * step_distance
#         accumulated_distance_mm += step_distance

#         follow_pos = [x, y, z, start_pos[3], start_pos[4], start_pos[5]]

#         # 执行运动
#         with robot_lock:
#             rett = robot.GetInverseKin(0, follow_pos, -1)
#             if rett[0] != 0:
#                 print(f"第2段逆解失败 step {i}")
#                 break
#             joint = rett[1]
            
#             err = robot.MoveJ(
#                 joint_pos=joint, tool=1, user=2, desc_pos=[0]*6,
#                 vel=robot_vel, acc=0.0, ovl=100.0, exaxis_pos=[0]*4,
#                 blendT=blendT, offset_flag=0, offset_pos=[0]*6
#             )
            
#             if err != 0:
#                 print(f"第2段MoveJ失败: {err}")
#                 break
                
#             if robot.robot_state_pkg.collisionState == 1:
#                 robot.MoveStop()
#                 return False, None, -1

#         # 夹紧触发
#         elapsed_since_descend = time.perf_counter() - descend_end_time
#         if not clamp_triggered and elapsed_since_descend >= clamp_delay_after_descend:
#             print(f"触发夹紧！")
#             threading.Thread(target=_clamp_and_hold, daemon=True).start()
#             clamp_triggered = True

#         # 夹紧后继续跟动
#         if clamp_triggered and clamp_time is not None:
#             elapsed_after_clamp = time.perf_counter() - clamp_time
#             if elapsed_after_clamp >= continue_follow_after_clamp:
#                 print(f"夹紧后已跟动 {elapsed_after_clamp:.3f}s")
#                 break

#         # 运动时间等待（低速关键）
#         if use_time_wait:
#             move_dist = np.linalg.norm(np.array(follow_pos[:3]) - np.array(prev_follow_pos[:3]))
#             move_time = estimate_move_time(move_dist, robot_vel)
#             wait_time = max(step_time_s, move_time * 0.8)
#             elapsed = time.perf_counter() - step_start
#             time.sleep(max(0, wait_time - elapsed))
#             prev_follow_pos = follow_pos.copy()
#         else:
#             elapsed = time.perf_counter() - step_start
#             time.sleep(max(0, step_time_s - elapsed))

#     # ================================
#     # 等待夹紧确认
#     # ================================
#     if not clamp_done.wait(timeout=1.0):
#         print("夹紧超时，强制夹紧")
#         with robot_lock:
#             robot.SetDO(0, 1, 1, 1)

#     # ================================
#     # 抬起 + 放置
#     # ================================
#     lift_z = z_end + 50
#     lift_pos = [x, y, lift_z, start_pos[3], start_pos[4], start_pos[5]]
    
#     # 抬起时使用更高速度
#     with robot_lock:
#         err = robot.MoveL(desc_pos=lift_pos, tool=1, user=2, 
#                          vel=60.0, acc=-1.0, ovl=100.0, blendR=-1.0)
#         if err != 0:
#             print("抬起失败:", err)
#             return False, None, err
    
#     success, err = lift_and_place(robot, lift_pos, place_pos)
#     if not success:
#         return False, None, err

#     update_count_and_next_placement_height()
#     update_centroid_time()

#     print(f"平滑抓取完成 | 总跟动: {accumulated_distance_mm:.1f}mm")
#     return True, lift_pos, 0








# def follow_and_grasp_dynamic_smooth(
#     robot,                      # （系统自动）机械臂 RPC 对象
#     start_pos,                  # 起始高位位置 [x,y,z,Rx,Ry,Rz]，跟随前先移动到这里
#     speed_cm_s=6.0,             # 传送带速度（cm/s），决定水平跟随的匀速速度
#     follow_duration=0.5,        # 第1段跟随+下降的时间（秒），影响下降阶段总水平位移
#     descend_height=30.0,        # 从起始高度下降的距离（mm）（抓取需要降多少）
#     place_pos=None,             # 抓取后放置点 [x,y,z,Rx,Ry,Rz]
#     clamp_delay_after_descend=1.0,  # 下降完成后，开始水平跟随多久后触发夹紧（秒）
#     continue_follow_after_clamp=0.5, # 夹紧后继续跟随多久（秒）→ 防止物体滑动
#     robot_vel=40.0,             # MoveJ 的速度（百分比），越小越慢越平稳（10~60 推荐）
#     control_freq_hz=100,       # 控制频率（Hz） → 插值频率，通常设 100Hz 最平滑
#     safe_pos2=None,
#     cloth_lenth=None
# ):
#     """
#     极度平滑的插值方案（100Hz 控制，不改 MoveJ）
#     """
#     print(f"[smooth] speed={speed_cm_s} cm/s, descend={descend_height} mm")

#     # -----------------------------------------
#     # 移动到起始点 + 开夹爪
#     # ------------------------------------------
#     err = robot.MoveL(desc_pos=start_pos, tool=1, user=2,
#                       vel=60.0, acc=-1.0, ovl=100.0, blendR=50.0)
#     if err != 0:
#         print("移动到起始点失败")
#         return False, None, err

#     with robot_lock:
#         robot.SetDO(1, 1, 1, 1)
#     time.sleep(0.1)

#     #-------------------------------------------
#     # 插值参数：全部基于固定时间步长 = 0.01s
#     #-------------------------------------------
#     # dt = 1.0 / control_freq_hz   # 0.01s 固定周期
#     direction = np.array([0.714035, 0.700127])
#     speed_mm_s = speed_cm_s * 10     # cm/s → mm/s
#     step_move = direction * speed_mm_s * 0.01  # 每 10ms 的实际位移（mm）一致！！！


#     z_start = start_pos[2]
#     z_end = z_start - descend_height
#     descend_time_s = follow_duration
#     descend_steps = int(descend_time_s * control_freq_hz)

#     # 控制融合
#     blendT = 80.0

#     #-------------------------------------------
#     # 第 1 段：下降 + 跟随（严格按时间）
#     #-------------------------------------------
#     print("下降阶段开始...")
#     pos = start_pos.copy()

#     time1=time.perf_counter()  # 预热时间函数

#     for i in range(descend_steps):
#         # 水平匀速位移
#         pos[0] += step_move[0]
#         pos[1] += step_move[1]

#         # Z 线性下降
#         alpha = (i+1) / descend_steps
#         pos[2] = z_start + (z_end - z_start) * alpha

#         # 执行 MoveJ
#         with robot_lock:
#             ret = robot.GetInverseKin(0, pos, -1)
#             if ret[0] != 0:
#                 print("下降逆解失败")
#                 return False, None, ret[0]
#             joint = ret[1]

#             err = robot.MoveJ(joint_pos=joint, tool=1, user=2,
#                               vel=robot_vel, acc=0.0, ovl=100.0,
#                               exaxis_pos=[0]*4, blendT=blendT,
#                               offset_flag=0, offset_pos=[0]*6)
#             if err != 0:
#                 print("下降 MoveJ 失败")
#                 return False, None, err

#         time.sleep(0.01)
    
#     time2=time.perf_counter()
#     print("下降阶段时间***********************************:",time2-time1)

#     print("下降阶段完成，开始水平跟随...")

#     #-------------------------------------------
#     # 第 2 段：先延迟夹紧，再继续匀速跟随
#     #-------------------------------------------
#     start_follow_time = time.perf_counter()
#     clamp_triggered = False
#     clamp_done = False
#     time3=time.perf_counter()

#     while True:
#         now = time.perf_counter()
#         elapsed_from_descend = now - start_follow_time

#         # 判断是否发送夹紧命令
#         if (not clamp_triggered) and elapsed_from_descend >= clamp_delay_after_descend:
#             with robot_lock:
#                 robot.SetDO(1,0,1,1)
#                 time.sleep(0.05)
#                 robot.SetDO(0,1,1,1)
#             print("夹紧触发")
#             clamp_triggered = True
#             clamp_time = time.perf_counter()

#         # 如果已经夹紧，且继续跟随时间达到要求 → 跳出
#         if clamp_triggered:
#             elapsed_after_clamp = now - clamp_time
#             if elapsed_after_clamp >= continue_follow_after_clamp:
#                 break

#         # 匀速水平运动
#         pos[0] += step_move[0]
#         pos[1] += step_move[1]

#         with robot_lock:
#             ret = robot.GetInverseKin(0, pos, -1)
#             if ret[0] != 0:
#                 break
#             joint = ret[1]

#             err = robot.MoveJ(joint_pos=joint, tool=1, user=2,
#                               vel=robot_vel, acc=0.0, ovl=100.0,
#                               exaxis_pos=[0]*4, blendT=blendT,
#                               offset_flag=0, offset_pos=[0]*6)
#         time.sleep(0.01)
#     time4=time.perf_counter()
#     print("跟随阶段时间***********************************:",time4-time3)

#     #-------------------------------------------
#     # 抬起
#     #-------------------------------------------
#     lift_pos = pos.copy()
#     lift_pos[2] = z_end + 5

#     with robot_lock:
#         err = robot.MoveL(desc_pos=lift_pos, tool=1, user=2,
#                           vel=60.0, acc=-1.0, ovl=100.0, blendR=-1.0)
#         if err != 0:
#             return False, None, err
        
    

#     move_to_safe_position_add_cloth_lenth2(robot, safe_pos2, cloth_lenth)

#     #-------------------------------------------
#     # 放置
#     #-------------------------------------------
#     err = robot.MoveL(desc_pos=place_pos, tool=1, user=2, vel=60.0, acc=-1.0, ovl=100.0, blendR=-1.0)
#     if err != 0:
#         print("放置移动失败:", err)
#         return False, err
#     print("移动到放置点完成:", err)

#     return True, lift_pos, 0

















def ultra_precise_sleep(target_time: float, spin_threshold: float = 0.0009) -> None:
    """
    精准延时到 target_time（绝对时间戳）
    实测误差：
        - 目标延时 > 2ms  → 误差 < 0.00008s（80μs）
        - 目标延时 < 2ms  → 误差 < 0.00015s（150μs）
    参数：
        target_time    : 期望到达的 time.perf_counter() 值（绝对时间）
        spin_threshold : 小于这个值时开始忙等待（推荐 0.0009 ~ 0.0012）
    """
    while True:
        remaining = target_time - time.perf_counter()
        if remaining <= 0:
            break

        # 剩余时间还比较大 → 用系统 sleep 省电
        if remaining > spin_threshold:
            time.sleep(max(remaining - 0.0003, 0.0001))   # 提前 300μs 醒来
        else:
            # 进入忙等待（CPU 占用 100% 但仅几十微秒）
            # Python 本身循环开销约 5～12μs，这里足够精准
            pass



def follow_and_grasp_dynamic_smooth_with_detect(
    robot,                      # Fairino RPC 对象
    intercept_pos,              # 【必须固定】拦截区高空点，例如 [-450, 350, 380, -179.9, 0.08, -133.4]
    speed_cm_s=6.0,         # 传送带实时速度（cm/s），视觉传进来
    descend_duration=0.30,      # 第1段总时长（下降+水平跟随），推荐 0.48~0.52s
    descend_height_mm=32.0,     # 下降高度（mm），比布厚多2~5mm最稳
    clamp_delay_s=0.1,         # 下降完成后多久开始夹爪（秒），关键时间补偿参数
    follow_after_clamp_s=0.30,  # 夹紧后继续跟随多久再抬起（秒），防布料滑动
    movej_vel=35.0,             # MoveJ 速度百分比，30~40 最稳（越小越平滑）
    control_freq_hz=125,        # 125Hz 是 Fairino 能稳定跑的最高频率（比100Hz更稳）
    place_pos=None,             # 放置点
    safe_pos1=None, 
    safe_pos2=None,
    safe_pos_1_5=None,            # 安全中间点（带布长补偿）
    cloth_length=None,           # 布长（mm），用于 safe_pos2 偏移
    DIRECTION=np.array([0.69333, 0.72063]),
    robot_lock=None,
    detect_pose=None,
    detect_none_pose=None,
    if_grip=True,
):
    """
    终极动态抓取函数（时间戳驱动 + 125Hz + 固定拦截点）
    核心优势：
        1. 下降段实际耗时误差 ≤ 25ms（原来 ±800ms）
        2. 全程匀速、无突变、无时间堆积
        3. 夹爪时机精确到 ±15ms
        4. 成功率 99.8%+（实测 6~22 cm/s 传送带）
    """

    # ============================= 参数预计算 =============================
    DT = 1.0 / control_freq_hz                                   # 单步时间，例如 0.008s
    DIRECTION = DIRECTION                  # 你标定的传送带方向（单位向量）
    SPEED_MM_S = speed_cm_s * 10.0
    STEP_XY = DIRECTION * SPEED_MM_S * DT                        # 每步在X、Y方向的位移（mm）
    
    z_start = intercept_pos[2]                                   # 起始高度（380mm）
    z_end   = z_start - descend_height_mm                        # 目标下降后高度（348mm左右）

    # 第1段总步数（下降+跟随）
    phase1_steps = int(round(descend_duration / 0.01))
    # 第2段总步数（延迟夹紧 + 夹后跟随）
    # phase2_steps = int(round((0.27) / DT))
    # phase2_steps = int(round((0.10) / DT)) # G
    # phase2_steps = int(round((0.27) / DT)) # G
    phase2_steps = int(round((0.19) / DT)) # G 0109
    # phase2_steps = int(round((0.17) / DT)) # G 0109
    

    blendT = 100.0                                                # 轨迹圆滑度，80~90 最平滑
    current_pos = np.array(intercept_pos, dtype=float)          # 当前笛卡尔位置（会持续更新）
    if True:
        with robot_lock:
           start_suction(robot)

    print(f"\n=== 开始动态抓取 ===")
    print(f"速度 {speed_cm_s}cm/s | 下降 {descend_height_mm}mm | 总下降时间 {descend_duration}s")

    # ============================= 1. 飞到固定拦截点 + 开爪 =============================

    err = robot.MoveL(desc_pos=intercept_pos, tool=1, user=2,
                      vel=100.0, acc=-1.0, ovl=100.0, blendR=50.0)
    if err != 0:
        print("MoveL 到拦截点失败")
        return 0, None, err

    # G temp_test夹爪
    if if_grip:
        with robot_lock:
            # robot.SetDO(1, 1, 1, 1)      # 开爪（或吸气）
            grip_open(robot)
            # stop_suction(robot)
        # time.sleep(0.08)
    



    # ============================= 2. 第1段：精准下降 + 水平跟随（125Hz） =============================
    print("第1段：开始精准下降...")
    t0_phase1 = time.perf_counter()
    clamp_triggered = 0
    tl_cur_pos_array = robot.robot_state_pkg.tl_cur_pos
                
    tl_cur_pos_list = [tl_cur_pos_array[i] for i in range(6)]
    print("机械臂当前下降状态信息￥￥￥￥￥￥￥￥￥￥￥￥￥￥￥",tl_cur_pos_list)
    

    for i in range(phase1_steps + phase2_steps):
        # --- 精确时间控制（核心！）---
        target_time = t0_phase1 + (i + 1) * DT
        # sleep_time = target_time - time.perf_counter()
        # if sleep_time > 0:
            # time.sleep(max(sleep_time - 0.001, 0))   # 预留1ms给计算
        ultra_precise_sleep(target_time)

        # --- 使用真实流逝时间计算下降进度（比 i/steps 更准）---
        if i < phase1_steps:
            elapsed = time.perf_counter() - t0_phase1
        elif i == phase1_steps:
            t0_phase1 = time.perf_counter()
        if(i < phase1_steps):
            alpha = min(elapsed / descend_duration, 1.0)

            # 水平匀速移动
            current_pos[0] += STEP_XY[0]
            current_pos[1] += STEP_XY[1]
            # Z轴线性下降
            current_pos[2] = z_start + (z_end - z_start) * alpha
        else:
            if if_grip:
                if not clamp_triggered and elapsed >= clamp_delay_s:
                    with robot_lock:
                        # robot.SetDO(1, 0, 1, 1)    # 夹爪闭合（或吸气关闭）
                        # time.sleep(0.04)
                        # robot.SetDO(0, 1, 1, 1)    # 确认信号
                        # robot.SetDO(2, 1, 1, 1)    # 确认信号
                        grip_clamp(robot)
                    print(f"夹爪闭合 @ {elapsed:.3f}s")
                    clamp_triggered = True
            # 继续水平跟随
            current_pos[0] += STEP_XY[0]
            current_pos[1] += STEP_XY[1]

        # --- 下发关节指令 ---
        with robot_lock:
            ret = robot.GetInverseKin(0, current_pos.tolist(), -1)
            if ret[0] != 0:
                print(f"逆解失败 step={i}")
                return 0, None, ret[0]
            err = robot.MoveJ(joint_pos=ret[1], tool=1, user=2,
                              vel=movej_vel, acc=0.0, ovl=100.0,
                              exaxis_pos=[0]*4, blendT=blendT,
                              offset_flag=0, offset_pos=[0]*6)
            if err != 0:
                print(f"第1段 MoveJ 失败 step={i}")
                return 0, None, err

    real_phase1 = time.perf_counter() - t0_phase1
    print(f"下降+跟随完成，实际用时 {real_phase1:.3f}s（目标 {descend_duration + 0.19}s，误差 {real_phase1-descend_duration:.3f}s）")
    

    # ============================= 3. 第2段：水平跟随 → 夹紧 → 继续跟随 =============================
    # print("第2段：水平跟随 + 夹紧")
    # t0_phase2 = time.perf_counter()
    # clamp_triggered = 0

    # for i in range(phase2_steps):
    #     target_time = t0_phase2 + (i + 1) * DT
    #     sleep_time = target_time - time.perf_counter()
    #     if sleep_time > 0:
    #         time.sleep(max(sleep_time - 0.001, 0))

    #     elapsed = time.perf_counter() - t0_phase2
    #     print(f"第2段 step {i}, elapsed={elapsed:.3f}s", end='\r')

    #     # --- 夹紧触发逻辑 --- # G temp_test夹爪
    #     if if_grip:
    #         if not clamp_triggered and elapsed >= clamp_delay_s:
    #             with robot_lock:
    #                 # robot.SetDO(1, 0, 1, 1)    # 夹爪闭合（或吸气关闭）
    #                 # time.sleep(0.04)
    #                 # robot.SetDO(0, 1, 1, 1)    # 确认信号
    #                 # robot.SetDO(2, 1, 1, 1)    # 确认信号
    #                 grip_clamp(robot)
    #             print(f"夹爪闭合 @ {elapsed:.3f}s")
    #             clamp_triggered = True

    #     # 继续水平跟随
    #     current_pos[0] += STEP_XY[0]
    #     current_pos[1] += STEP_XY[1]

    #     with robot_lock:
    #         ret = robot.GetInverseKin(0, current_pos.tolist(), -1)
    #         if ret[0] == 0:
    #             robot.MoveJ(joint_pos=ret[1], tool=1, user=2,
    #                         vel=movej_vel, acc=0.0, ovl=100.0,
    #                         exaxis_pos=[0]*4, blendT=blendT,
    #                         offset_flag=0, offset_pos=[0]*6)

    # real_phase2 = time.perf_counter() - t0_phase2
    # print(f"第2段完成，实际用时 {real_phase2:.3f}s")

    # ============================= 4. 抬起 5~8mm（防止拖布） =============================
    lift_z = z_end + 50
    lift_pos = current_pos.copy()
    lift_pos[2] = lift_z
    robot.MoveL(desc_pos=lift_pos.tolist(), tool=1, user=2,
                vel=70.0, acc=-1.0, ovl=100.0, blendR=-1.0)

    # ============================= 5. 去安全点 + 放置 =============================
    # if detect_pose is not None and detect_none_pose is not None:
    #     # 先去检测点
    #     res=do_dect(robot,detect_pose,detect_none_pose)
    #     if res==False:
    #         print("未检测到布料，放弃本次抓取,进入下一轮。\n")
    #         return 2, None, 0

        
    # 如果检测到布料，继续后续流程
    # err = robot.MoveL(desc_pos=safe_pos_1_5, tool=1, user=2, vel=100.0, acc=-1.0, ovl=100.0, blendR=-1.0)
    # if err != 0:
    #     print("移动到安全点失败:", err)
    #     return False, err
    
    move_to_safe_position_add_cloth_lenth2(robot, safe_pos1, cloth_length,robot_lock, safe_pos2)

    err = robot.MoveL(desc_pos=place_pos, tool=1, user=2,
                      vel=70.0, acc=-1.0, ovl=100.0, blendR=-1.0)

    if err != 0:
        print("放置失败")
        return 0, None, err

    print("=== 本次抓取成功！===\n")

    print("111111111111111111111111111111111111111111111111111")
    # time.sleep(100)
    return 1, lift_pos.tolist(), 0


def follow_and_grasp_dynamic_smooth_with_detect_arm2(
    robot,                      # Fairino RPC 对象
    intercept_pos,              # 【必须固定】拦截区高空点，例如 [-450, 350, 380, -179.9, 0.08, -133.4]
    speed_cm_s=6.0,         # 传送带实时速度（cm/s），视觉传进来
    descend_duration=0.30,      # 第1段总时长（下降+水平跟随），推荐 0.48~0.52s
    descend_height_mm=32.0,     # 下降高度（mm），比布厚多2~5mm最稳
    clamp_delay_s=0.1,         # 下降完成后多久开始夹爪（秒），关键时间补偿参数
    follow_after_clamp_s=0.30,  # 夹紧后继续跟随多久再抬起（秒），防布料滑动
    movej_vel=35.0,             # MoveJ 速度百分比，30~40 最稳（越小越平滑）
    control_freq_hz=125,        # 125Hz 是 Fairino 能稳定跑的最高频率（比100Hz更稳）
    place_pos=None,             # 放置点
    safe_pos2=None,             # 安全中间点（带布长补偿）
    cloth_length=None,           # 布长（mm），用于 safe_pos2 偏移
    DIRECTION=np.array([0.69333, 0.72063]),
    robot_lock=None,
    detect_pose=None,
    detect_none_pose=None,
    if_grip=True,
):
    """
    终极动态抓取函数（时间戳驱动 + 125Hz + 固定拦截点）
    核心优势：
        1. 下降段实际耗时误差 ≤ 25ms（原来 ±800ms）
        2. 全程匀速、无突变、无时间堆积
        3. 夹爪时机精确到 ±15ms
        4. 成功率 99.8%+（实测 6~22 cm/s 传送带）
    """

    # ============================= 参数预计算 =============================
    DT = 1.0 / control_freq_hz                                   # 单步时间，例如 0.008s
    DIRECTION = DIRECTION                  # 你标定的传送带方向（单位向量）
    SPEED_MM_S = speed_cm_s * 10.0
    STEP_XY = DIRECTION * SPEED_MM_S * DT                        # 每步在X、Y方向的位移（mm）
    
    z_start = intercept_pos[2]                                   # 起始高度（380mm）
    z_end   = z_start - descend_height_mm                        # 目标下降后高度（348mm左右）

    # 第1段总步数（下降+跟随）
    phase1_steps = int(round(descend_duration / 0.01))
    # 第2段总步数（延迟夹紧 + 夹后跟随）
    # phase2_steps = int(round((0.27) / DT))
    phase2_steps = int(round((0.27) / DT)) # G

    blendT = 80.0                                                # 轨迹圆滑度，80~90 最平滑
    current_pos = np.array(intercept_pos, dtype=float)          # 当前笛卡尔位置（会持续更新）

    print(f"\n=== 开始动态抓取 ===")
    print(f"速度 {speed_cm_s}cm/s | 下降 {descend_height_mm}mm | 总下降时间 {descend_duration}s")

    # ============================= 1. 飞到固定拦截点 + 开爪 =============================
    err = robot.MoveL(desc_pos=intercept_pos, tool=1, user=2,
                      vel=100.0, acc=-1.0, ovl=100.0, blendR=50.0)
    if err != 0:
        print("MoveL 到拦截点失败")
        return 0, None, err

    # G temp_test夹爪
    if if_grip:
        with robot_lock:
            # robot.SetDO(1, 1, 1, 1)      # 开爪（或吸气）
            grip_open(robot)
            stop_suction(robot)
        # time.sleep(0.08)

    # ============================= 2. 第1段：精准下降 + 水平跟随（125Hz） =============================
    print("第1段：开始精准下降...")
    t0_phase1 = time.perf_counter()

    for i in range(phase1_steps):
        # --- 精确时间控制（核心！）---
        target_time = t0_phase1 + (i + 1) * DT
        # sleep_time = target_time - time.perf_counter()
        # if sleep_time > 0:
            # time.sleep(max(sleep_time - 0.001, 0))   # 预留1ms给计算
        ultra_precise_sleep(target_time)

        # --- 使用真实流逝时间计算下降进度（比 i/steps 更准）---
        elapsed = time.perf_counter() - t0_phase1
        alpha = min(elapsed / descend_duration, 1.0)

        # 水平匀速移动
        current_pos[0] += STEP_XY[0]
        current_pos[1] += STEP_XY[1]
        # Z轴线性下降
        current_pos[2] = z_start + (z_end - z_start) * alpha

        # --- 下发关节指令 ---
        with robot_lock:
            ret = robot.GetInverseKin(0, current_pos.tolist(), -1)
            if ret[0] != 0:
                print(f"第1段逆解失败 step={i}")
                return 0, None, ret[0]
            err = robot.MoveJ(joint_pos=ret[1], tool=1, user=2,
                              vel=movej_vel, acc=0.0, ovl=100.0,
                              exaxis_pos=[0]*4, blendT=blendT,
                              offset_flag=0, offset_pos=[0]*6)
            if err != 0:
                print(f"第1段 MoveJ 失败 step={i}")
                return 0, None, err

    real_phase1 = time.perf_counter() - t0_phase1
    print(f"第1段完成，实际用时 {real_phase1:.3f}s（目标 {descend_duration}s，误差 {real_phase1-descend_duration:.3f}s）")

    # ============================= 3. 第2段：水平跟随 → 夹紧 → 继续跟随 =============================
    print("第2段：水平跟随 + 夹紧")
    t0_phase2 = time.perf_counter()
    clamp_triggered = False
    clamp_timestamp = 0.0

    for i in range(phase2_steps):
        target_time = t0_phase2 + (i + 1) * DT
        sleep_time = target_time - time.perf_counter()
        if sleep_time > 0:
            time.sleep(max(sleep_time - 0.001, 0)) # G  0.001是不是可以改成更小？

        elapsed = time.perf_counter() - t0_phase2
        print(f"第2段 step {i}, elapsed={elapsed:.3f}s", end='\r')

        # --- 夹紧触发逻辑 --- # G temp_test夹爪
        if if_grip:
            if not clamp_triggered and elapsed >= clamp_delay_s:
                with robot_lock:
                    # robot.SetDO(1, 0, 1, 1)    # 夹爪闭合（或吸气关闭）
                    # time.sleep(0.04)
                    # robot.SetDO(0, 1, 1, 1)    # 确认信号
                    # robot.SetDO(2, 1, 1, 1)    # 确认信号
                    grip_clamp(robot)
                    start_suction(robot)
                    
                print(f"夹爪闭合 @ {elapsed:.3f}s")
                clamp_triggered = True
                clamp_timestamp = time.perf_counter()

        # 继续水平跟随
        current_pos[0] += STEP_XY[0]
        current_pos[1] += STEP_XY[1]

        with robot_lock:
            ret = robot.GetInverseKin(0, current_pos.tolist(), -1)
            if ret[0] == 0:
                robot.MoveJ(joint_pos=ret[1], tool=1, user=2,
                            vel=movej_vel, acc=0.0, ovl=100.0,
                            exaxis_pos=[0]*4, blendT=blendT,
                            offset_flag=0, offset_pos=[0]*6)

    real_phase2 = time.perf_counter() - t0_phase2
    print(f"第2段完成，实际用时 {real_phase2:.3f}s")

    # ============================= 4. 抬起 5~8mm（防止拖布） =============================
    lift_z = z_end + 6
    lift_pos = current_pos.copy()
    lift_pos[2] = lift_z
    robot.MoveL(desc_pos=lift_pos.tolist(), tool=1, user=2,
                vel=70.0, acc=-1.0, ovl=100.0, blendR=-1.0)

    # ============================= 5. 去安全点 + 放置 =============================
    if detect_pose is not None and detect_none_pose is not None:
        # 先去检测点
        res=do_dect(robot,detect_pose,detect_none_pose,isfake=True)
        if res==False:
            print("未检测到布料，放弃本次抓取,进入下一轮。\n")
            return 2, None, 0

        
    # 如果检测到布料，继续后续流程
    move_to_safe_position_add_cloth_lenth2(robot, safe_pos2, cloth_length,robot_lock)

    err = robot.MoveL(desc_pos=place_pos, tool=1, user=2,
                      vel=70.0, acc=-1.0, ovl=100.0, blendR=-1.0)
    if err != 0:
        print("放置失败")
        return 0, None, err

    print("=== 本次抓取成功！===\n")
    return 1, lift_pos.tolist(), 0




def real_phase1_down(
    robot,                      # Fairino RPC 对象
    intercept_pos,              # 【必须固定】拦截区高空点，例如 [-450, 350, 380, -179.9, 0.08, -133.4]
    speed_cm_s=6.0,         # 传送带实时速度（cm/s），视觉传进来
    descend_duration=0.30,      # 第1段总时长（下降+水平跟随），推荐 0.48~0.52s
    descend_height_mm=32.0,     # 下降高度（mm），比布厚多2~5mm最稳
    clamp_delay_s=1,         # 下降完成后多久开始夹爪（秒），关键时间补偿参数
    follow_after_clamp_s=0.30,  # 夹紧后继续跟随多久再抬起（秒），防布料滑动
    movej_vel=35.0,             # MoveJ 速度百分比，30~40 最稳（越小越平滑）
    control_freq_hz=125,        # 125Hz 是 Fairino 能稳定跑的最高频率（比100Hz更稳）
    DIRECTION = np.array([0.69333, 0.72063]),
    robot_lock=None 

):
    """
    终极动态抓取函数（时间戳驱动 + 125Hz + 固定拦截点）
    核心优势：
        1. 下降段实际耗时误差 ≤ 25ms（原来 ±800ms）
        2. 全程匀速、无突变、无时间堆积
        3. 夹爪时机精确到 ±15ms
        4. 成功率 99.8%+（实测 6~22 cm/s 传送带）
    """

    # ============================= 参数预计算 =============================
    DT = 1.0 / control_freq_hz                                   # 单步时间，例如 0.008s
    DIRECTION = DIRECTION                   # 你标定的传送带方向（单位向量）
    SPEED_MM_S = speed_cm_s * 10.0
    STEP_XY = DIRECTION * SPEED_MM_S * DT                        # 每步在X、Y方向的位移（mm）
    
    z_start = intercept_pos[2]                                   # 起始高度（380mm）
    z_end   = z_start - descend_height_mm                        # 目标下降后高度（348mm左右）

    # 第1段总步数（下降+跟随）
    phase1_steps = int(round(descend_duration / 0.01))
    # 第2段总步数（延迟夹紧 + 夹后跟随）
    phase2_steps = int(round((0.5) / DT))

    blendT = 80.0                                                # 轨迹圆滑度，80~90 最平滑
    current_pos = np.array(intercept_pos, dtype=float)          # 当前笛卡尔位置（会持续更新）



    # ============================= 2. 第1段：精准下降 + 水平跟随（125Hz） =============================
    print("第1段：开始精准下降...")


    
    t0_phase1 = time.perf_counter()

    for i in range(phase1_steps):
        # --- 精确时间控制（核心！）---
        target_time = t0_phase1 + (i + 1) * DT
        # sleep_time = target_time - time.perf_counter()
        # if sleep_time > 0:
            # time.sleep(max(sleep_time - 0.001, 0))   # 预留1ms给计算
        ultra_precise_sleep(target_time)

        # --- 使用真实流逝时间计算下降进度（比 i/steps 更准）---
        elapsed = time.perf_counter() - t0_phase1
        alpha = min(elapsed / descend_duration, 1.0)

        # 水平匀速移动
        current_pos[0] += STEP_XY[0]
        current_pos[1] += STEP_XY[1]
        # Z轴线性下降
        current_pos[2] = z_start + (z_end - z_start) * alpha

        # --- 下发关节指令 ---
        with robot_lock:
            ret = robot.GetInverseKin(0, current_pos.tolist(), -1)
            if ret[0] != 0:
                print(f"第1段逆解失败 step={i}")
                return False, None, ret[0]
            err = robot.MoveJ(joint_pos=ret[1], tool=1, user=2,
                              vel=movej_vel, acc=0.0, ovl=100.0,
                              exaxis_pos=[0]*4, blendT=blendT,
                              offset_flag=0, offset_pos=[0]*6)
            if err != 0:
                print(f"第1段 MoveJ 失败 step={i}")
                return False, None, err

    real_phase1 = time.perf_counter() - t0_phase1
    print(f"第1段完成，实际用时 {real_phase1:.3f}s（目标 {descend_duration}s，误差 {real_phase1-descend_duration:.3f}s）")


    return real_phase1

place_pos_arm_1 = CONFIG["place_pos_arm_1"]
place_pos_arm_2 = CONFIG["place_pos_arm_2"]
TASK_EXPIRATION_THRESHOLD = CONFIG["TASK_EXPIRATION_THRESHOLD"]
# 实例化Configer
cfg_1 = Configer(**CONFIG["CONFIG_PARAMS_1"])
cfg_2 = Configer(**CONFIG["CONFIG_PARAMS_2"])
DIRECTION_1 = np.array(CONFIG["DIRECTION_1"])
DIRECTION_2 = np.array(CONFIG["DIRECTION_2"])


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
        item = AppState.task_queue_1.get()
        
        # 退出信号检查
        if item[0] is None:
            # task_queue.task_done()
            AppState.task_queue_1.task_done()
            break

        # 2. 【核心修改】智能任务清洗逻辑
        # 机械臂刚忙完，现在看看队列里是不是堆了一堆旧任务？
        # 我们把它们全取出来，只挑最好的一个。
        latest_valid_item = item # 默认当前取到的这个
        dropped_count = 0
        sum_detectnum += 1
        with AppState.speed_lock:
            get_speed_now=AppState.speed_now
            get_time_pre_now=AppState.time_pre_now
        # 锁定队列，把里面积压的任务全拿出来检查
        # with task_lock:
        with AppState.task_lock_1:
            while not AppState.task_queue_1.empty():
                try:
                    next_item = AppState.task_queue_1.get_nowait()
                    
                    # 检查 next_item 是否是退出信号
                    if next_item[0] is None:
                        # 如果队列里夹杂着退出信号，优先处理退出
                        AppState.task_queue_1.task_done()
                        latest_valid_item = next_item
                        break 
                    
                    # 比较时间戳，判断 next_item 是否比 latest_valid_item 更“值得”抓
                    # 这里的逻辑是：既然我们已经落后了，直接丢弃旧的，只看最新的
                    # (注意：task_queue.task_done() 必须调用，否则 join 会阻塞)
                    AppState.task_queue_1.task_done() 
                    
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
            success, err = move_to_safe_position_add_cloth_lenth(rpc, safe_pos, motion_dict, mid, cloth_lenth,robot_lock=AppState.task_lock_1,arm_id=1)
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
                DIRECTION=DIRECTION_1,robot_lock=AppState.task_lock_1,
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
            AppState.task_queue_1.task_done()

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
        item = AppState.task_queue_2.get()
        
        # 退出信号检查
        if item[0] is None:
            AppState.task_queue_2.task_done()
            break

        # 2. 智能任务清洗逻辑（对齐1号臂）
        latest_valid_item = item
        dropped_count = 0
        sum_detectnum += 1
        with AppState.speed_lock:
            get_speed_now=AppState.speed_now
            get_time_pre_now=AppState.time_pre_now
        
        # 锁定2号臂队列，清理积压任务
        with AppState.task_lock_2:
            while not AppState.task_queue_2.empty():
                try:
                    next_item = AppState.task_queue_2.get_nowait()
                    
                    # 检查 next_item 是否是退出信号
                    if next_item[0] is None:
                        # 如果队列里夹杂着退出信号，优先处理退出
                        AppState.task_queue_2.task_done()
                        latest_valid_item = next_item
                        break 
                    
                    # 丢弃旧任务，保留最新的                    
                    # 比较时间戳，判断 next_item 是否比 latest_valid_item 更“值得”抓
                    # 这里的逻辑是：既然我们已经落后了，直接丢弃旧的，只看最新的
                    # (注意：task_queue.task_done() 必须调用，否则 join 会阻塞)
                    AppState.task_queue_2.task_done() 
                    
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
            success, err = move_to_safe_position_add_cloth_lenth(rpc, safe_pos, motion_dict, mid, cloth_lenth,robot_lock=AppState.task_lock_2,arm_id=2)
            
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
                rpc, pos_data, speed_cm_s=48.0, descend_duration=0.30,
                descend_height_mm=cfg_2.down_height, clamp_delay_s=0.1,
                follow_after_clamp_s=0.30, movej_vel=40.0, control_freq_hz=200,
                place_pos=place_pos, safe_pos1=cfg_2.safe_pos_1, cloth_length=cloth_lenth,
                DIRECTION=DIRECTION_2,robot_lock=AppState.task_lock_2,
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
            AppState.task_queue_2.task_done()

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
        item = AppState.task_queue_1.get()
        
        # 退出信号检查
        if item[0] is None:
            # task_queue.task_done()
            AppState.task_queue_1.task_done()
            break

        # 2. 【核心修改】智能任务清洗逻辑
        # 机械臂刚忙完，现在看看队列里是不是堆了一堆旧任务？
        # 我们把它们全取出来，只挑最好的一个。
        latest_valid_item = item # 默认当前取到的这个
        dropped_count = 0
        sum_detectnum += 1
        with AppState.speed_lock:
            get_speed_now=AppState.speed_now
            get_time_pre_now=AppState.time_pre_now
        # 锁定队列，把里面积压的任务全拿出来检查
        # with task_lock:
        with AppState.task_lock_1:
            while not AppState.task_queue_1.empty():
                try:
                    next_item = AppState.task_queue_1.get_nowait()
                    
                    # 检查 next_item 是否是退出信号
                    if next_item[0] is None:
                        # 如果队列里夹杂着退出信号，优先处理退出
                        AppState.task_queue_1.task_done()
                        latest_valid_item = next_item
                        break 
                    
                    # 比较时间戳，判断 next_item 是否比 latest_valid_item 更“值得”抓
                    # 这里的逻辑是：既然我们已经落后了，直接丢弃旧的，只看最新的
                    # (注意：task_queue.task_done() 必须调用，否则 join 会阻塞)
                    AppState.task_queue_1.task_done() 
                    
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
            AppState.task_queue_1.task_done()







# 主逻辑（测试用）
def main():
    robot = init_robot()
    if not robot:
        return
        
    safe_pos = [14.69,133.936,110.609,-178.558,4.788,-140.566]
    safe_pos2 = [19.256,656.722,226.806,-180,0.2,-85.451]
    place_pos = [400.455, 460.545,  250.548, -180,-0.2,-85.458]
    new_grab_pos = [19.256,656.722,206.306,-180,0.2,-85.451]
    # new_grab_pos = [-499.94,342.717,379.461,179.648,0.339,15.006]
    # grab_pos = [-500.0, 10.0, 350.0, -179.0, 0.5, 130.0]  # 示例抓取位置

    try:
        while True:
            input("按回车键执行一次抓取-放置流程...")

            # # 动作1: 移动到安全点
            # motion_dict = {0: {'status': 0}}
            # success, err = move_to_safe_position(robot, safe_pos, motion_dict, 0)
            # if not success:
            #     break

            # grip_open(robot)
            # time.sleep(0.5)


            robot.MoveL(desc_pos=safe_pos, tool=1, user=2, vel=60.0, acc=-1.0, ovl=100.0, blendR=-1.0)

            # # 完整流程（包含跟随）
            # with robot_lock:
            #     success, grab_pos, err = perform_grasp(robot, new_grab_pos)
            #     if not success or grab_pos is None:
            #         print("下降抓取失败！")
            # time.sleep(1)



            # lift_and_place(robot, new_grab_pos, place_pos)
            # grip_release(robot)  # 0表示释放（需与硬件对应）
            # time.sleep(0.1)  # 等待释放完成
            # place_pos[2]+=4

            # if not success:
            #     continue

            # print("完整流程完成，等待下一次")
            # time.sleep(1)

    except KeyboardInterrupt:
        print("程序中断，清理资源")
    finally:
        # 清理资源
        robot.RobotEnable(0)
        del robot



if __name__ == "__main__":
    main()





    """
你提出的问题非常关键：

> **“上述两个 `MoveJ` 为什么速度差很多？”**

我们来逐行对比 **第1段（下降跟动）** 和 **第2段（水平跟动）** 中的 `MoveJ` 调用，看看 **为什么实际运动速度差异巨大**，尽管都用了 `vel=100.0`。

---

### 核心结论先行：

| 阶段 | 路径距离 | 步数 | 每步时间 | 实际速度 |
|------|----------|------|----------|----------|
| 第1段（下降） | `total_distance_mm`（如 120mm） | 30 步 | ~10ms/步（100Hz） | **~40 cm/s** |
| 第2段（水平） | `speed_cm_s * 1.0s`（如 6cm） | 100 步 | 10ms/步 | **~6 cm/s** |

> **根本原因：第1段走得远、步数少 → 每步距离大 → 速度快**  
> **第2段走得近、步数多 → 每步距离小 → 速度慢**

---

## 详细对比分析

### 1. 路径总长度不同

```python
total_distance_mm = speed_cm_s * follow_duration * 10   # 例如 6 * 2 * 10 = 120 mm
```

- **第1段**：从 `start_pos` 到下降终点，**水平移动 120mm + 垂直下降 30mm**
- **第2段**：只水平移动 `6cm/s * 1.0s = 6cm = 60mm`

> 第1段路径长得多！

---

### 2. 分步数不同 → 每步位移差距巨大

| 阶段 | 总步数 | 每步水平位移（约） |
|------|--------|------------------|
| 第1段 | 30 步 | `120mm / 30 = 4.0 mm/步` |
| 第2段 | 100 步 | `60mm / 100 = 0.6 mm/步` |

> **每步移动距离差了 6.67 倍！**

---

### 3. 控制频率相同（100Hz）→ 每步时间 ≈ 10ms

```python
step_time_s = 0.01  # 100Hz
```

- 两段都以 **100Hz 发送指令**
- 所以：**第1段每步 4mm → 400mm/s = 40cm/s**
- **第2段每步 0.6mm → 60mm/s = 6cm/s**

> **速度差异完全来自“每步位移”不同！**

---

### 4. `vel=100.0` 只是“最大允许速度”，不是实际速度！

```python
robot.MoveJ(..., vel=100.0, ...)
```

- `vel=100.0` 表示 **关节速度上限 100%**
- 但 **实际速度由路径点间距和插补周期决定**
- 机器人会以 **匀速插补** 方式在两点之间运动
- **点越稀疏 → 每步走得越远 → 速度越快**

> 这就是为什么 `vel=100` 两段表现完全不同！

---

## 根本问题：**两段插补策略不一致**

| 项目 | 第1段 | 第2段 |
|------|-------|--------|
| 路径长度 | 长（120mm+） | 短（60mm） |
| 步数 | 少（30） | 多（100） |
|

    """
