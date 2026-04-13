import threading
import time
import numpy as np
from Robot import RPC
from control_air_close_open import grip_clamp, grip_open, grip_release, start_suction, stop_suction, photoelectric_sensor
from  linear_actuator_long import move_to
from  detect_test import do_dect
import json
import logger
from logger import LOG_DEBUG, LOG_INFO, LOG_WARN, LOG_ERROR, LOG_CRITICAL
from global_state import AppState
import queue
import threading
from motor_control import KINCO_Motor

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

place_pos_arm_1 = CONFIG["place_pos_arm_1"]
place_pos_arm_2 = CONFIG["place_pos_arm_2"]
TASK_EXPIRATION_THRESHOLD = CONFIG["TASK_EXPIRATION_THRESHOLD"]
DIRECTION_1 = np.array(CONFIG["DIRECTION_1"])
DIRECTION_2 = np.array(CONFIG["DIRECTION_2"])

LENTH_CHANGE_THRESHOLD= CONFIG["MOVE_PARAMS_1"]["LENTH_CHANGE_THRESHOLD"]
 
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
    # err = robot.MoveL(desc_pos=place_pos, tool=1, user=AppState.cfg_1.id_wcs, vel=60.0, acc=-1.0, ovl=100.0, blendR=-1.0)
    # if err != 0:
    #     print("移动到放置点失败:", err)
    #     return False
    # print("移动到动态放置点上方完成")

    # 1. 移动到动态放置位置（使用更新后的高度）
    print(f"第{complete_count + 1}次放置，目标高度: {dynamic_place_pos[2]}mm")
    err = robot.MoveL(desc_pos=dynamic_place_pos, tool=1, user=AppState.cfg_1.id_wcs, vel=100.0, acc=30.0, ovl=100.0, blendR=100.0)
    stop_suction(robot)
    if err != 0:
        print("移动到放置点失败:", err)
        return False
    print("移动到动态放置点完成")



    dynamic_place_pos[2] = current_place_height
    print("current_place_height: %s", current_place_height)  # 参数测试

    err = robot.MoveL(desc_pos=dynamic_place_pos, tool=1, user=AppState.cfg_1.id_wcs, vel=100.0, acc=-1.0, ovl=100.0, blendR=100.0)
    if err != 0:
        return False

    # 2. 释放吸盘（假设使用数字输出口1控制，根据实际硬件调整）
    grip_release(robot)  # 0表示释放（需与硬件对应） # G temp_test夹爪
    
    time.sleep(3.0)  # 等待释放完成




    # 3. 移动到安全等待位置
    err = robot.MoveL(desc_pos=safe_pos, tool=1, user=AppState.cfg_1.id_wcs, vel=60.0, acc=-1.0, ovl=100.0, blendR=-1.0)
    if err != 0:
        print("移动到安全等待位置失败:", err)
        return False

    err = robot.MoveL(desc_pos=safe_pos2, tool=1, user=AppState.cfg_1.id_wcs, vel=60.0, acc=-1.0, ovl=100.0, blendR=-1.0)
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

    # 1. 移动到动态放置位置（使用更新后的高度）
    print(f"第{complete_count_arm2 + 1}次放置，目标高度: {dynamic_place_pos[2]}mm")
    err = robot.MoveL(desc_pos=dynamic_place_pos, tool=1, user=AppState.cfg_2.id_wcs, vel=100.0, acc=30.0, ovl=100.0, blendR=100.0)
    if err != 0:
        print("移动到放置点失败:", err)
        return False
    print("移动到动态放置点完成")

    dynamic_place_pos[2] = current_place_height

    err = robot.MoveL(desc_pos=dynamic_place_pos, tool=1, user=AppState.cfg_2.id_wcs, vel=100.0, acc=-1.0, ovl=100.0, blendR=100.0)
    if err != 0:
        return False

    # 2. 释放吸盘（假设使用数字输出口1控制，根据实际硬件调整）
    # grip_release(robot)  # 0表示释放（需与硬件对应）# G temp_test夹爪
    time.sleep(0.1)  # 等待释放完成

    # 3. 移动到安全等待位置
    err = robot.MoveL(desc_pos=safe_pos, tool=1, user=AppState.cfg_2.id_wcs, vel=60.0, acc=-1.0, ovl=100.0, blendR=-1.0)
    if err != 0:
        print("移动到安全等待位置失败:", err)
        return False
    print("已返回安全等待位置")


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
        err = robot.MoveL(desc_pos=safe_pos, tool=1, user=AppState.cfg_1.id_wcs, vel=60.0, acc=-1.0, ovl=100.0, blendR=-1.0)
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
        err = robot.MoveL(desc_pos=safe_pos, tool=1, user=AppState.cfg_1.id_wcs, vel=60.0, acc=-1.0, ovl=100.0, blendR=-1.0)
        if err != 0:
            print("移动到安全点失败:", err)
            return False, err
        motion_dict[mid]['status'] = 3  # 标记为已完成
    
    print("移动到安全点完成")
    return True, 0


def move_to_cloth_lenth(cloth_lenth):
    """
    移动到安全点
    返回值: (success: bool, err: int)
    """
    global STAND_LENGTH
# G 1121 添加丝杆长度调整----start
    # print("====================complete_count:",complete_count)
    # if complete_count==0 :
    # if True:
    if cloth_lenth is not None :
        # lenth_change = abs((cloth_lenth - STAND_LENGTH) / STAND_LENGTH)
        lenth_change = abs((cloth_lenth - STAND_LENGTH))

        print(f"》》》》》》》》》》》》》》》》》》》》cloth_lenth:{cloth_lenth},STAND_LENGTH:{STAND_LENGTH},lenth_change:{lenth_change}##############")
        # print("STAND_LENGTH@@@@@@@@@@@@@",STAND_LENGTH)
        # print("lenth_change!!!!!!!!!!!!!",lenth_change)
        # if lenth_change > 0.2:

        if lenth_change > LENTH_CHANGE_THRESHOLD:
            with AppState.changeScrew_lock:
                AppState.changeScrew = True
            # if cloth_lenth is not None:
            # STAND_LENGTH = cloth_lenth
            distance = 335 - cloth_lenth/2 + ADJUSTMENT_GRIPPER_OFFSET  # 在布料长度基础上增加20mm作为安全余量
            # print("》》》》》》》》》》》》》》》》》》》》distance##############",distance)
            
            # 先判断当前操作的机械臂是否超限
            if 1 <= distance <= 335:
                try:
                    move_to(distance)
                except Exception as e:
                    print(f"1号臂丝杆移动失败：{e}")
            else:
                print(f"1号臂丝杆目标位置超出行程允许范围（1-330mm）：当前计算值为 {distance:.2f}mm")
        else:
            print("未获取到新品种布料长度，丝杆不移动")            
# G 1121 添加丝杆长度调整----end
    return True, 0


# def move_to_cloth_lenth(robot, cloth_lenth):
#     """
#     移动到安全点
#     返回值: (success: bool, err: int)
#     """
#     if USE_LINEAR_ACTUATOR is False:
#         return
#     global STAND_LENGTH
#     LOG_INFO("cloth_lenth: %fmm", cloth_lenth)
# # G 1121 添加丝杆长度调整----start
#     # print("====================complete_count:",complete_count)
#     # if complete_count==0 :
#     # if True:
#     if cloth_lenth is not None :
#         lenth_change = abs((cloth_lenth - STAND_LENGTH) / STAND_LENGTH)

#         print("cloth_lenth##############",cloth_lenth)
#         print("STAND_LENGTH@@@@@@@@@@@@@",STAND_LENGTH)
#         print("lenth_change!!!!!!!!!!!!!",lenth_change)
#         if lenth_change > 0.2:
#         # if cloth_lenth is not None:
#             STAND_LENGTH = cloth_lenth
#             distance = 335 - cloth_lenth/2 + ADJUSTMENT_GRIPPER_OFFSET  # 在布料长度基础上增加20mm作为安全余量
#             print("distance##############",distance)
#             if 1<=distance <=335:
#                 try:
#                     move_to(distance)   
#                 except  Exception as e:
#                     print(f"丝杆移动失败：{e}")
#             else:
#                 print(f"丝杆目标位置超出行程允许范围：{distance}mm")
#         else:
#             print("未获取到新品种布料长度，丝杆不移动")
# # G 1121 添加丝杆长度调整----end
#     return True, 0


def move_to_safe_position_add_cloth_lenth2(robot, safe_pos, cloth_lenth,robot_lock=None,safe_pos2=None):
    """
    移动到安全点
    返回值: (success: bool, err: int)
    """

# G 1121 添加丝杆长度调整----end

    with robot_lock:
        print("移动到安全点2:", safe_pos)
        err = robot.MoveL(desc_pos=safe_pos2, tool=1, user=AppState.cfg_2.id_wcs, vel=100.0, acc=-1.0, ovl=100.0, blendR=-1.0)
        if err != 0:
            print("3移动到安全点2失败:", err)
            return False, err

    with robot_lock:
        print("移动到安全点:", safe_pos)
        err = robot.MoveL(desc_pos=safe_pos, tool=1, user=AppState.cfg_2.id_wcs, vel=100.0, acc=-1.0, ovl=100.0, blendR=-1.0)
        if err != 0:
            print("3移动到安全点失败:", err)
            return False, err
    
    print("移动到安全点完成")
    return True, 0


def move_to_safe_position_add_cloth_lenth_init(robot, safe_pos,robot_lock=None):
    """
    移动到安全点
    返回值: (success: bool, err: int)
    """

    with robot_lock:
        print("移动到安全点:", safe_pos)
        err = robot.MoveL(desc_pos=safe_pos, tool=1, user=AppState.cfg_1.id_wcs, vel=60.0, acc=-1.0, ovl=100.0, blendR=-1.0)
        if err != 0:
            print("4移动到安全点失败:", err)
            return False, err
    
    print("移动到安全点完成")
    return True, 0


def move_to_safe_position_add_cloth_lenth_init(robot, safe_pos, safe_pos2, robot_lock=None,arm_id=1):
    """
    移动到安全点
    返回值: (success: bool, err: int)
    """

    user_param = 2 if arm_id == 1 else 1 
    with robot_lock:
        print("移动到安全点1:", safe_pos)
        err = robot.MoveL(desc_pos=safe_pos, tool=1, user=AppState.cfg_1.id_wcs, vel=60.0, acc=-1.0, ovl=100.0, blendR=-1.0)
        if err != 0:
            print("移动到安全点1失败:", err)
            return False, err
        
    with robot_lock:
        print("移动到安全点2:", safe_pos)
        err = robot.MoveL(desc_pos=safe_pos2, tool=1, user=AppState.cfg_1.id_wcs, vel=60.0, acc=-1.0, ovl=100.0, blendR=-1.0)
        if err != 0:
            print("移动到安全点2失败:", err)
            return False, err
    
    print("移动到安全点完成")
    return True, 0


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

    err = robot.MoveL(desc_pos=intercept_pos, tool=1, user=AppState.cfg_1.id_wcs,
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
            err = robot.MoveJ(joint_pos=ret[1], tool=1, user=AppState.cfg_1.id_wcs,
                              vel=movej_vel, acc=0.0, ovl=100.0,
                              exaxis_pos=[0]*4, blendT=blendT,
                              offset_flag=0, offset_pos=[0]*6)
            if err != 0:
                print(f"第1段 MoveJ 失败 step={i}")
                return 0, None, err

    real_phase1 = time.perf_counter() - t0_phase1
    print(f"下降+跟随完成，实际用时 {real_phase1:.3f}s（目标 {descend_duration + 0.19}s，误差 {real_phase1-descend_duration:.3f}s）")

    # ============================= 4. 抬起 5~8mm（防止拖布） =============================
    lift_z = z_end + 50
    lift_pos = current_pos.copy()
    lift_pos[2] = lift_z
    robot.MoveL(desc_pos=lift_pos.tolist(), tool=1, user=AppState.cfg_1.id_wcs,
                vel=70.0, acc=-1.0, ovl=100.0, blendR=-1.0)

    # ============================= 5. 去安全点 + 放置 =============================
    # if detect_pose is not None and detect_none_pose is not None:
    #     # 先去检测点
    #     res=do_dect(robot,detect_pose,detect_none_pose)
    #     if res==False:
    #         print("未检测到布料，放弃本次抓取,进入下一轮。\n")
    #         return 2, None, 0

        
    # 如果检测到布料，继续后续流程
    # err = robot.MoveL(desc_pos=safe_pos_1_5, tool=1, user=AppState.cfg_1.id_wcs, vel=100.0, acc=-1.0, ovl=100.0, blendR=-1.0)
    # if err != 0:
    #     print("移动到安全点失败:", err)
    #     return False, err
    
    move_to_safe_position_add_cloth_lenth2(robot, safe_pos1, cloth_length,robot_lock, safe_pos2)

    err = robot.MoveL(desc_pos=place_pos, tool=1, user=AppState.cfg_1.id_wcs,
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
    err = robot.MoveL(desc_pos=intercept_pos, tool=1, user=AppState.cfg_2.id_wcs,
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
            err = robot.MoveJ(joint_pos=ret[1], tool=1, user=AppState.cfg_2.id_wcs,
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
                robot.MoveJ(joint_pos=ret[1], tool=1, user=AppState.cfg_2.id_wcs,
                            vel=movej_vel, acc=0.0, ovl=100.0,
                            exaxis_pos=[0]*4, blendT=blendT,
                            offset_flag=0, offset_pos=[0]*6)

    real_phase2 = time.perf_counter() - t0_phase2
    print(f"第2段完成，实际用时 {real_phase2:.3f}s")

    # ============================= 4. 抬起 5~8mm（防止拖布） =============================
    lift_z = z_end + 6
    lift_pos = current_pos.copy()
    lift_pos[2] = lift_z
    robot.MoveL(desc_pos=lift_pos.tolist(), tool=1, user=AppState.cfg_2.id_wcs,
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

    err = robot.MoveL(desc_pos=place_pos, tool=1, user=AppState.cfg_2.id_wcs,
                      vel=70.0, acc=-1.0, ovl=100.0, blendR=-1.0)
    if err != 0:
        print("放置失败")
        return 0, None, err

    print("=== 本次抓取成功！===\n")
    return 1, lift_pos.tolist(), 0


def movel_to_pose(robot,intercept_pos,vel=100.0):
    err = robot.MoveL(desc_pos=intercept_pos, tool=1, user=2,
                    vel=vel, acc=-1.0, ovl=100.0, blendR=50.0)
    if err != 0:
        print(f"MoveL 失败")
        return False
    return True

def static_grap(robot, up_pose, wait_pose, grasp_pose, detect_pose1, detect_pose2, stop_pose, place_pose, robot_lock):
    """静态抓取函数"""
    # move to wait
    with robot_lock:
        err = movel_to_pose(robot, up_pose)
        if err == 0:
            print(f"移动到上位姿失败，错误码: {err}")
            return False
        print("抓取开始")
    
    # move to wait
    with robot_lock:
        err = movel_to_pose(robot, wait_pose)
        if err == 0:
            print(f"移动到等待位姿失败，错误码: {err}")
            return False
    print("抓取等待位置")

    grip_open(robot)
        
    with AppState.changeScrew_lock:
        if AppState.changeScrew is True:
            time.sleep(3)
            AppState.changeScrew = False
        else:
            time.sleep(0.1)
    
    # move to down
    with robot_lock:
        err = movel_to_pose(robot, grasp_pose)
        if err == 0:
            print(f"移动到抓取位姿失败，错误码: {err}")
            return False
    print("抓取位姿")
    time.sleep(0.1)
    grip_clamp(robot)
    time.sleep(0.4)  # 等待夹爪闭合

    # move to up
    with robot_lock:
        grasp_pose2 = grasp_pose.copy()
        grasp_pose2[2] += 20
        err = movel_to_pose(robot, grasp_pose2, vel=50)
        err = movel_to_pose(robot, detect_pose1)


        # ret = 0
        # time_start = time.time()
        # while time.time() - time_start < 1:
        #     if photoelectric_sensor(robot) != 0:
        #         ret = 1

        ret = 0
        with AppState.detectSucceed_cond:
            AppState.detectNow = True
            AppState.detectSucceed_cond.notify_all()
            err = movel_to_pose(robot, detect_pose2, vel=50)
            while AppState.detectNow is True:
                AppState.detectSucceed_cond.wait()
            ret = AppState.graspSucceed
            AppState.graspSucceed = False
        

        # ret = photoelectric_sensor(robot)
        LOG_INFO("ret: %d", ret)
        if ret == 0:
            LOG_INFO("抓取失败，放回去")
            stop_suction(robot)
            # time.sleep(3)
            err = movel_to_pose(robot, up_pose)
            # with AppState.armCanMove_lock:
            #     AppState.armCanMove = False
            time.sleep(1)
            with AppState.armCanMove_cond:
                AppState.armCanMove = False
                AppState.armCanMove_cond.notify_all()  # ✅ 唤醒所有等待线程
            if err == 0:
                print(f"移动到安全位置失败，错误码: {err}")
                return False
            return False
        else:
            LOG_INFO("抓取成功")
        err = movel_to_pose(robot, stop_pose)

        # with AppState.canDetect_lock:
        #     AppState.canDetect = True
        if err == 0:
            print(f"移动到上位姿失败，错误码: {err}")
            return False
    print("抬起位姿")

    with robot_lock:
        stop_suction(robot)
        err = movel_to_pose(robot, place_pose)
        if err == 0:
            print(f"移动到放置位置失败，错误码: {err}")
            return False
    print("放置")

    # with AppState.armCanMove_lock:
    #     AppState.armCanMove = False
    with AppState.armCanMove_cond:
        AppState.armCanMove = False
        AppState.armCanMove_cond.notify_all()  # ✅ 唤醒所有等待线程

    grip_release(robot)

    time.sleep(1) 

    with robot_lock:
        err = movel_to_pose(robot, up_pose)
        if err == 0:
            print(f"移动到安全位置失败，错误码: {err}")
            return False
    print("回到初始位置")
    return True


def get_task_abnormal_info(motion_dict, mid):
    """
    从视觉输出中获取异常信息
    约定视觉侧在 motion_dict[mid] 中写入:
        abnormal: bool
        abnormal_name: str   # "", "卷曲", "卡料"
        static_grab: dict    # 静态抓取相关位姿
    """
    motion           = motion_dict.get(mid, {})
    abnormal         = bool(motion.get("abnormal", False))
    abnormal_name    = str(motion.get("abnormal_name", "") or "").strip()
    static_grab_info = motion.get("static_grab", None)
    return abnormal, abnormal_name, static_grab_info


def process_tasks_1(rpc, motor):
    """
    处理抓取任务的线程函数（智能过滤版）
    """
    global DIRECTION_1
    
    place_pos = place_pos_arm_1
    sum_detectnum=0
    sum_abort=0
    sum_to_do=0
    print(">>> 1号机械臂处理线程已启动，等待任务...")
    LOG_INFO("1号机械臂处理线程已启动, 等待任务")

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

        # -------------------------------------------------
        # 2. 解析任务
        # -------------------------------------------------
        motion_dict, mid, pos_data, safe_pos = item
        motion = motion_dict.get(mid, {})

        abnormal, abnormal_name, static_grab_info = get_task_abnormal_info(motion_dict, mid)


        motion_dict, mid, pos_data, safe_pos = item
        # logger.info(motion_dict[mid]['motion_center'])

        # -------------------------------------------------
        # 3. 异常布料：卷曲 -> 停带 + 静态抓取
        # -------------------------------------------------
        # if abnormal and abnormal_name == "卷曲":
        if True:
            LOG_INFO("准备静态抓取")
            cloth_lenth = motion_dict[mid]['long_side_length']
            move_to_cloth_lenth(cloth_lenth)

            motor.stop()
            with AppState.task_lock_1:
                start_suction(rpc)
                LOG_INFO("检测到新布料，开始吸气")
            while True:
                if motor.get_speed() < 0.1:
                    break
                else:
                    time.sleep(0.1)
            time.sleep(1)
            LOG_INFO("motor speed: %f", motor.get_speed())
            # with AppState.armCanMove_lock:
            #     AppState.armCanMove = True
            LOG_INFO("传送带静止，开始停止视觉检测")
            with AppState.armCanMove_cond:
                AppState.armCanMove = True

            print(f"[1号臂-异常] mid={mid} 准备静态抓取")

            # if not static_grab_info:
            #     print(f"[1号臂-异常] mid={mid} 缺少 static_grab 位姿信息，无法执行静态抓取")
            #     sum_abort += 1
            #     continue

            # up_pose           = static_grab_info.get("up_pose")
            # wait_pose         = static_grab_info.get("wait_pose")
            # grasp_pose        = static_grab_info.get("grasp_pose")
            # stop_pose         = static_grab_info.get("stop_pose")
            # place_pose_static = static_grab_info.get("place_pose", place_pos)

            up_pose = [AppState.cfg_1.safe_x,
            AppState.cfg_1.safe_y,
            AppState.cfg_1.safe_z,
            AppState.cfg_1.stand_rx,
            AppState.cfg_1.stand_ry,
            AppState.cfg_1.stand_rz]
            def transform_point(affine_matrix, point):
                x, y = point
                vec = np.array([x, y, 1])
                x_robot, y_robot = affine_matrix @ vec
                return x_robot, y_robot
            # realX, realY = transform_point(AppState.AFFINE_MATRIX_1, (AppState.centroid[mid][0] + 54, AppState.centroid[mid][1] + 12.5))
            realX, realY = transform_point(AppState.AFFINE_MATRIX_1, (AppState.centroid[mid][0], AppState.centroid[mid][1] + 23))
            LOG_INFO("task use global centroid: id: %d, visionx: %f, visiony: %f, x: %f, y: %f", mid, AppState.centroid[mid][0], AppState.centroid[mid][1], realX, realY)
            wait_pose = [realX,
            realY,
            AppState.cfg_1.stand_z + 10,
            AppState.cfg_1.stand_rx,
            AppState.cfg_1.stand_ry,
            pos_data[5]]
            grasp_pose = [realX,
            realY,
            AppState.cfg_1.stand_z,
            AppState.cfg_1.stand_rx,
            AppState.cfg_1.stand_ry,
            pos_data[5]]
            detect_pose1 = [AppState.cfg_1.detect_x,
            AppState.cfg_1.detect_y,
            AppState.cfg_1.stand_z + 80,
            AppState.cfg_1.stand_rx,
            AppState.cfg_1.stand_ry,
            AppState.cfg_1.stand_rz]
            detect_pose2 = [AppState.cfg_1.detect_x,
            AppState.cfg_1.detect_y,
            AppState.cfg_1.stand_z + 40,
            AppState.cfg_1.stand_rx,
            AppState.cfg_1.stand_ry,
            AppState.cfg_1.stand_rz]
            stop_pose = [AppState.cfg_1.safe_x,
            AppState.cfg_1.safe_y,
            AppState.cfg_1.stand_z + 20,
            AppState.cfg_1.stand_rx,
            AppState.cfg_1.stand_ry,
            AppState.cfg_1.stand_rz]
            place_pose_static = [AppState.cfg_1.safe_x,
            AppState.cfg_1.safe_y,
            PLACE_HEIGHT_INCREMENT,
            AppState.cfg_1.stand_rx,
            AppState.cfg_1.stand_ry,
            AppState.cfg_1.stand_rz]

            if not all([up_pose, wait_pose, grasp_pose, stop_pose, place_pose_static]):
                print(f"[1号臂-异常] mid={mid} 静态抓取位姿不完整")
                sum_abort += 1
                continue

            sum_to_do += 1

            # 先停传送带
            # conveyor_stop()
            print("[1号臂-异常] 传送带已停止，开始静态抓取")

            ok = static_grap(
                robot=rpc,
                up_pose=up_pose,
                wait_pose=wait_pose,
                grasp_pose=grasp_pose,
                detect_pose1=detect_pose1,
                detect_pose2=detect_pose2,
                stop_pose=stop_pose,
                place_pose=place_pose_static,
                robot_lock=AppState.task_lock_1
            )
            motor.set_speed(AppState.cfg_1.speed)

            result = "静态抓取失败"
            if ok:
                result = "静态抓取成功"
                update_count_and_next_placement_height()
                update_centroid_time()
            else:
                sum_abort += 1

            sum_real_done = get_count_arm1()
            # logger.info(
            #     f"====================1号臂(静态抓取): "
            #     f"当前处理结果: {result} ; "
            #     f"总检测量：{sum_detectnum}; "
            #     f"实际成功总量:{sum_real_done}; "
            #     f"积压抛弃总量:{sum_abort}; "
            #     f"机械臂处理数量:{sum_to_do}; "
            #     f"机械臂处理失败总量:{sum_to_do - sum_real_done}; "
            #     f"===================="
            # )
            #     f"机械臂处理失败总量:{sum_to_do - sum_real_done}; "
            LOG_INFO("1号臂(静态抓取): 当前处理结果: %s, 总检测量: %d, 实际成功总量: %d, 积累抛弃总量: %d, 机械臂处理数量: %d, 机械臂处理失败总量: %d", result, sum_detectnum, sum_real_done,sum_abort,sum_to_do ,sum_to_do - sum_real_done)
            continue

        if False:
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
                # wait_time_final = AppState.cfg_1.time_pre - (time.perf_counter() - motion_dict[mid]['line2_time'])
                
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
                    descend_height_mm=AppState.cfg_1.down_height, clamp_delay_s=0.1,
                    follow_after_clamp_s=0.30, movej_vel=40.0, control_freq_hz=200,
                    place_pos=place_pos, safe_pos1=AppState.cfg_1.safe_pos_1, safe_pos2=AppState.cfg_1.safe_pos_2, 
                    cloth_length=cloth_lenth,safe_pos_1_5=AppState.cfg_1.safe_pos_1_5,
                    DIRECTION=DIRECTION_1,robot_lock=AppState.task_lock_1,
                    detect_pose=AppState.cfg_1.detect_pose, detect_none_pose=AppState.cfg_1.detect_none_pose,
                    if_grip=False
                )
                
                result='失败'
                if res==1:
                    result='成功'
                    tl_cur_pos_array = rpc.robot_state_pkg.tl_cur_pos
                    tl_cur_pos_list = [tl_cur_pos_array[i] for i in range(6)]
                    print("tl_cur_pos_list@@@@@@@@@@@@@@@",tl_cur_pos_list)
                    place_and_move_to_safe(rpc, AppState.cfg_1.safe_pos_1, tl_cur_pos_list, AppState.cfg_1.safe_pos_2)
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
                descend_height_mm=AppState.cfg_2.down_height, clamp_delay_s=0.1,
                follow_after_clamp_s=0.30, movej_vel=40.0, control_freq_hz=200,
                place_pos=place_pos, safe_pos1=AppState.cfg_2.safe_pos_1, cloth_length=cloth_lenth,
                DIRECTION=DIRECTION_2,robot_lock=AppState.task_lock_2,
                detect_pose=AppState.cfg_2.detect_pose, detect_none_pose=AppState.cfg_2.detect_none_pose,
                if_grip=False
            )

            # 抓取结果处理
            result='失败'
            if res==1:
                result='成功'
                place_and_move_to_safe_arm2(rpc, AppState.cfg_2.safe_pos_1, place_pos)
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
            # wait_time_final = AppState.cfg_1.time_pre - (time.perf_counter() - motion_dict[mid]['line2_time'])
            
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
                descend_height_mm=AppState.cfg_1.down_height, clamp_delay_s=0.1,
                follow_after_clamp_s=0.30, movej_vel=40.0, control_freq_hz=200,
                place_pos=place_pos, safe_pos1=AppState.cfg_1.safe_pos_1, safe_pos2=AppState.cfg_1.safe_pos_2, 
                cloth_length=cloth_lenth,safe_pos_1_5=AppState.cfg_1.safe_pos_1_5,
                DIRECTION=DIRECTION_1,robot_lock=AppState.task_lock_1,
                detect_pose=AppState.cfg_1.detect_pose, detect_none_pose=AppState.cfg_1.detect_none_pose,
                if_grip=False
            )

            
            result='失败'
            if res==1:
                result='成功'
                tl_cur_pos_array = rpc.robot_state_pkg.tl_cur_pos
                tl_cur_pos_list = [tl_cur_pos_array[i] for i in range(6)]
                print("tl_cur_pos_list@@@@@@@@@@@@@@@",tl_cur_pos_list)
                place_and_move_to_safe(rpc, AppState.cfg_1.safe_pos_1, tl_cur_pos_list, AppState.cfg_1.safe_pos_2)
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


# 初始化机械臂
def init_robot(ip="192.168.57.4",arm=1):
    # robot = RPC(ip="192.168.58.2")
    robot = RPC(ip)
    print("######################机械臂ip:", ip)
    
    if arm==1:
        if USE_LINEAR_ACTUATOR:
            move_to(-1)      # 回原点
            time.sleep(5)
            move_to(150)
        time.sleep(0.5)    # 等待2秒
    elif arm==2: #G 待 右臂需要跟丝杆做绑定
        print("GGG二号臂暂不用丝杆")
        time.sleep(0.5)    # 等待2秒

    # 机械臂使能并切换到自动模式
    if robot.RobotEnable(1) != 0 or robot.Mode(0) != 0:
        print("机械臂初始化失败")
        return None

    print("机械臂初始化成功")
    return robot


def grasp_succeed_detection(robot):
    LOG_INFO("grasp_succeed_detection thread started")  # 添加打印
    while True:
        with AppState.detectSucceed_cond:
            while AppState.detectNow is False:
                LOG_INFO("detectNow: %d", AppState.detectNow)
                AppState.detectSucceed_cond.wait()
            time_start = time.time()
            while time.time() - time_start < 2:
                if photoelectric_sensor(robot) != 0:
                    AppState.graspSucceed = True
            AppState.detectNow = False
            AppState.detectSucceed_cond.notify_all()


def init():
    robot_ip1  = CONFIG["robot_ip1"]
    robot_ip2  = CONFIG["robot_ip2"]
    robot_mode = CONFIG["robot_mode"]
    motor = KINCO_Motor(node_id=1)
    motor.start()
    motor.enable()
    motor.set_accel_cmss(30)
    motor.set_decel_cmss(50)
    motor.set_speed(AppState.cfg_1.speed)
    # motor = []
    if robot_mode==1:
        rpc_1 = init_robot(ip=robot_ip1,arm=1)  # 1号机械臂IP
        if not (rpc_1):
            print("机器人初始化失败")
            return False
        move_to_safe_position_add_cloth_lenth_init(rpc_1, AppState.cfg_1.safe_pos_1,AppState.cfg_1.safe_pos_2, robot_lock=AppState.task_lock_1,arm_id=1)
        rpc_2 = []
    elif robot_mode==2:
        rpc_2 = init_robot(ip=robot_ip2,arm=2)  # 2号机械臂IP
        if not (rpc_2):
            print("机器人初始化失败")
            return False
        move_to_safe_position_add_cloth_lenth_init(rpc_2, AppState.cfg_2.safe_pos_1,robot_lock=AppState.task_lock_2,arm_id=2)
        rpc_1 = []
    elif robot_mode==3:
        rpc_1 = init_robot(ip=robot_ip1,arm=1)  # 1号机械臂IP
        rpc_2 = init_robot(ip=robot_ip2,arm=2)  # 2号机械臂IP（根据实际调整）#G 待 右臂需要跟丝杆做绑定
        if not (rpc_1 and rpc_2):
            print("机器人初始化失败")
            return False
        move_to_safe_position_add_cloth_lenth_init(rpc_1, AppState.cfg_1.safe_pos_1,robot_lock=AppState.task_lock_1,arm_id=1)
        move_to_safe_position_add_cloth_lenth_init(rpc_2, AppState.cfg_2.safe_pos_1,robot_lock=AppState.task_lock_2,arm_id=2)  

    # 4. 启动任务线程
    if robot_mode==1:
        threading.Thread(target=process_tasks_1, args=(rpc_1, motor), daemon=True).start()
        threading.Thread(target=grasp_succeed_detection, args=(rpc_1,), daemon=True).start()
    elif robot_mode==2:
        threading.Thread(target=process_tasks_2, args=(rpc_2,), daemon=True).start()
    elif robot_mode==3:
        threading.Thread(target=process_tasks_1, args=(rpc_1, motor), daemon=True).start()
        threading.Thread(target=process_tasks_2, args=(rpc_2,), daemon=True).start()
    else:
        print("请选择正确的模式")
        return False
    
    return True


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


            robot.MoveL(desc_pos=safe_pos, tool=1, user=AppState.cfg_1.id_wcs, vel=60.0, acc=-1.0, ovl=100.0, blendR=-1.0)

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