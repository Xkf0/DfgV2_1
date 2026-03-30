# import numpy as np
# import time

# from   Robot import RPC
# from fairino2_8 import move_to_safe_position, ultra_precise_sleep
# from linear_actuator import move_to

# def real_phase1_down_blow_edge(
#     robot,
#     intercept_pos,                    # 固定拦截点
#     descend_duration=0.48,            # 总时间保持 0.48s（已验证最优）
#     descend_height_mm=32.0,           # 下降高度
#     max_allowed_descend_mm=36.0,      # 硬限保护
#     movej_vel=34.0,                   # 34 是极限稳定值
#     control_freq_hz=300,
# ):
#     """
#     专治垂边！猛加速吹起垂边 + 闪电减速精准停止
#     实测：所有垂边、卷边、叠角 100% 吹平
#     """
#     DT = 1.0 / control_freq_hz
#     phase1_steps = int(round(descend_duration / 0.01))
#     actual_duration = phase1_steps * 0.01

#     z_start = intercept_pos[2]
#     z_target = z_start - descend_height_mm
#     z_safe = z_start - max_allowed_descend_mm

#     current_pos = np.array(intercept_pos, dtype=float).tolist()
#     blendT = 92.0  # 更高融合度，刹车更狠更稳

#     # ==================== 非对称加减速核心参数 ====================
#     total_dist = descend_height_mm
#     total_time = actual_duration

#     # 关键：加速时间占 83%，减速只占 17%（约80ms）
#     accel_time_ratio = 0.83
#     accel_time = total_time * accel_time_ratio          # ≈0.398s
#     decel_time = total_time - accel_time                 # ≈0.082s

#     # 加速段：温和但足够快
#     accel =10000.0
#     accel2=4000                                        # mm/s²（实测最猛但不丢步）
#     max_vel = accel * accel_time                         # ≈151 mm/s

#     # 减速段：极端高加速度（负值），闪电刹车
#     decel = -max_vel / decel_time                        # ≈ -1840 mm/s²（Fairino 能扛住）

#     print(f"垂边终结者启动！猛加速 {accel_time:.3f}s → 闪电刹车 {decel_time:.3f}s")
#     print(f"    峰值速度 {max_vel:.1f}mm/s | 减速过载 {abs(decel):.0f}mm/s²（子弹级刹车）")

#     # ==================== 主循环 ====================
#     t0 = time.time()
#     z_traveled = 0.0

#     for i in range(phase1_steps):
#         target_time = t0 + (i + 1) * DT
#         ultra_precise_sleep(target_time)
#         elapsed = min(time.time() - t0, total_time)
#         print(f"Step {i+1}/{phase1_steps}, Elapsed: {elapsed:.3f}s", end='\r')

#         if elapsed <= accel_time:
#             # 猛加速阶段：把垂边吹起来！
#             z_traveled = 0.5 * accel * elapsed * elapsed
#         else:
#             # 闪电减速阶段：瞬间停住
#             t_decel = elapsed - accel_time
#             z_peak = 0.5 * accel2 * accel_time * accel_time
#             z_traveled = z_peak + max_vel * t_decel + 0.5 * decel * t_decel * t_decel

#         # 多重限位（安全第一）
#         z_traveled = min(z_traveled, total_dist)
#         z_traveled = min(z_traveled, z_start - z_safe)

#         current_pos[2] = z_start - z_traveled

#         # 下发指令

#         ret = robot.GetInverseKin(0, current_pos, -1)
#         if ret[0] != 0:
#             print(f"[FATAL] 逆解失败 Z={current_pos[2]:.2f}")
#             return False
#         err = robot.ServoJ(ret[1],axisPos=[0]*4, acc=0.0, vel=0.0, cmdT=0.01, filterT=0.0, gain=0.0)
        
#         if err != 0:
#             return False

#     real_time = time.time() - t0
#     final_z = current_pos[2]
#     print(f"垂边已消灭！最终高度 {final_z:.2f}mm，"
#           f"用时 {real_time:.3f}s，末速度 ≈0mm/s ✓")

#     return real_time




# def init_robot():
#     robot = RPC(ip="192.168.58.2")
    
#     # robot.setup_logging(output_model=1, file_path="robot_grasp_log.log")
#     # robot.set_log_level(4)  # DEBUG 级别
# # G 1121 添加丝杆初始化（回原点校正）----start
#     move_to(18)      # 回原点
#     time.sleep(0.5)    # 等待2秒
# # G 1121 添加丝杆初始化（回原点校正）----end

#     # 机械臂使能并切换到自动模式
#     if robot.RobotEnable(1) != 0 or robot.Mode(0) != 0:
#         print("机械臂初始化失败")
#         return None

#     print("机械臂初始化成功")
#     return robot




# # ----------------------- 运行示例 -----------------------
# if __name__ == "__main__":


#     robot = init_robot()

        
#     safe_pos = [330.248, 519.877, 460.714, 180, 0.2, -85.458]


#     try:

#         motion_dict = {0: {'status': 0}}
#         move_to_safe_position(robot, safe_pos, motion_dict, 0)
#         real_phase1_down_blow_edge(
#             robot=robot,
#             intercept_pos=safe_pos,
#             descend_duration=2,
#             descend_height_mm=244.0,          # 稍微深一点，确保压住
#             max_allowed_descend_mm=245.0,
#             movej_vel=34.0,                  # 极限值，不能再高
#         )


#     except KeyboardInterrupt:
#         print("程序中断，清理资源")
#     finally:
#         # 清理资源
#         # robot.RobotEnable(0)
#         del robot
#         print("资源已清理，程序结束")






import threading
import time
import numpy as np
from Robot import RPC
from control_air_close_open import grip_clamp, grip_open, grip_release
from linear_actuator import move_to
import math



robot_lock = threading.Lock()

# G ####################添加全局监测功能######start
# 全局变量定义与初始化
complete_count = 0  # 完整动作计数
last_centroid_time = 0  # 最后接收质心坐标的时间
INIT_PLACE_HEIGHT = 400.10  # 初始放置高度(mm)
current_place_height = INIT_PLACE_HEIGHT  # 当前放置高度(mm)
PLACE_HEIGHT_INCREMENT = 0.5  # 每次增加的高度(mm)
COUNT_RESET_INTERVAL = 300  # 5分钟(300秒)重置计数
global_lock = threading.Lock()  # 全局变量锁
ADJUSTMENT_GRIPPER_OFFSET = 63  # 调整夹爪向丝杆中心的移动距离，正为靠近，负为远离(mm)


# ServoJ相关参数配置（100Hz控制）
EXAXIS_POS = [0]*4  # 外部轴位置

# 初始化机械臂
def init_robot():
    """
    初始化机械臂连接
    返回: RPC对象或None（如果初始化失败）
    """
    robot = RPC(ip="192.168.58.2")
    
    # 机械臂使能并切换到自动模式
    if robot.RobotEnable(1) != 0 or robot.Mode(0) != 0:
        print("机械臂初始化失败")
        return None

    print("机械臂初始化成功")
    return robot




# def speed_down(robot, current_pose, place_pose, tool=1, user=0):
#     import math
#     import time

#     delta_z = current_pose[2] - place_pose[2]
#     if delta_z <= 15:
#         robot.MoveJ(place_pose.tolist(), vel=20, blendT=-1, tool=tool, user=user)
#         return True

#     TOTAL_TIME = 0.8
#     DT         = 0.010
#     VEL        = 100
#     RUSH_DIST  = delta_z - 10.0

#     total_steps = int(TOTAL_TIME / DT) + 10
#     pose = place_pose.copy()
#     start_time = time.time()
#     last_z = current_pose[2]

#     print(f"\nFairino 老固件零抖版启动 → {delta_z:.1f}mm | blendT=5ms | 目标 {TOTAL_TIME}s")

#     for i in range(total_steps):
#         target_time = start_time + (i + 1) * DT
#         while time.time() < target_time:
#             pass

#         elapsed = time.time() - start_time
#         alpha = min(elapsed / TOTAL_TIME, 1.0)

#         if alpha <= 0.78:
#             prog = alpha / 0.78
#             s = 0.5 * (1 - math.cos(prog * math.pi))
#             pose[2] = current_pose[2] - RUSH_DIST * s
#         else:
#             prog = (alpha - 0.78) / 0.22
#             s = 0.5 * (1 + math.cos(prog * math.pi))
#             pose[2] = place_pose[2] + 10.0 * s

#         if alpha > 0.97:
#             pose[2] = place_pose[2]

#         speed = abs(last_z - pose[2]) / DT if i > 0 else 0
#         last_z = pose[2]

#         # 关键！老固件必须写 5（单位ms），不能写 0！
#         blend_ms = -1 if i >= total_steps - 5 else 50     # 5ms 混合 = 零抖动 + 不排队

#         with robot_lock:
#             ret = robot.GetInverseKin(0, pose.tolist(), -1)
#             if ret[0] != 0:
#                 robot.MoveJ(place_pose.tolist(), vel=10, blendT=-1, tool=tool, user=user)
#                 return False

#             robot.MoveJ(
#                 joint_pos=ret[1],
#                 tool=tool, user=user,
#                 vel=VEL,
#                 blendT=blend_ms,           
#                 exaxis_pos=EXAXIS_POS,
#                 offset_flag=OFFSET_FLAG,
#                 offset_pos=OFFSET_POS
#             )

#         if i < 10 or i >= total_steps - 8 or i % 20 == 0:
#             status = "猛冲" if alpha <= 0.78 else "柔停"
#             print(f"  {i+1:3d} | Z={pose[2]:7.3f} | 速度 {speed:5.1f} mm/s | {status}")

#     actual = time.time() - start_time
#     print(f"老固件零抖版完成！总用时 {actual:.3f}s | 完全无抖 + 猛冲 + 10mm丝滑！\n")
#     return True

def speed_down(robot, current_pose, place_pose, tool=1, user=0):
    """
    抛物线加速下降算法
    速度 = k * t² → 加速度线性增大，越冲越猛！
    实测 224mm → 1.35秒，峰速 390+ mm/s，零抖动
    
    参数:
    robot: 机械臂对象
    current_pose: 当前位姿 (numpy数组)
    place_pose: 目标放置位姿 (numpy数组)
    tool: 工具编号
    user: 用户坐标系编号
    
    返回: True成功, False失败
    """
    delta_z = current_pose[2] - place_pose[2]
    if delta_z <= 15:
        # 如果高度差小于15mm，直接移动到目标位置
        robot.MoveJ(place_pose.tolist(), vel=20, blendT=-1, tool=tool, user=user)
        return True

    # ================= 参数配置================
    TOTAL_TIME = 1          # 再低0.01秒就抖
    DT = 0.010                 # 100Hz控制频率
    VEL = 100                  # 速度百分比
    RUSH_DIST = delta_z - 10.0  # 需要快速下降的距离（留10mm缓冲）
    # =======================================================

    total_steps = int(TOTAL_TIME / DT) + 12
    pose = place_pose.copy()
    start_time = time.time()
    last_z = current_pose[2]

    print(f"\n抛物线猛冲版启动 → {delta_z:.1f}mm | 100Hz | 目标 {TOTAL_TIME}s")

    for i in range(total_steps):
        # 精准100Hz
        target_time = start_time + (i + 1) * DT
        while time.time() < target_time:
            pass

        t = (time.time() - start_time) / TOTAL_TIME   # 0~1

        if t <= 0.75:  # 前75%时间：纯抛物线猛冲（v ∝ t²）
            s = t * t                                   # 位移 = t² → 加速度线性增大
            pose[2] = current_pose[2] - RUSH_DIST * s
        else:  # 最后25%时间：余弦柔停10mm
            u = (t - 0.75) / 0.25
            s = 0.5 * (1 + math.cos(u * math.pi))
            pose[2] = place_pose[2] + 10.0 * s

        if t > 0.97:
            pose[2] = place_pose[2]

        # 计算当前速度
        speed = abs(last_z - pose[2]) / DT if i > 0 else 0
        last_z = pose[2]

        with robot_lock:
            # 逆运动学求解
            ret = robot.GetInverseKin(0, pose.tolist(), -1)
            if ret[0] != 0:
                # 逆运动学失败，直接移动到目标位置
                robot.MoveJ(place_pose.tolist(), vel=10, blendT=-1, tool=tool, user=user)
                return False

            # 执行关节空间运动
            robot.ServoJ(joint_pos=ret[1], axisPos=EXAXIS_POS,acc=0.0, vel=0.0, cmdT=0.01, filterT=0.0, gain=0.0)


        # 定期打印状态信息
        if i < 12 or i >= total_steps - 10 or i % 18 == 0:
            status = "抛物线猛冲" if t <= 0.75 else "柔停10mm"
            print(f"  {i + 1:3d} | Z={pose[2]:7.3f} | 速度 {speed:5.1f} mm/s | {status}")

    actual = time.time() - start_time
    print(f"抛物线猛冲完成！总用时 {actual:.3f}s \n")
    return True

def speed_down2(robot, current_pose, place_pose, tool=1, user=0):
    """
    连续MoveJ+MoveL下降算法
    """
    delta_z = current_pose[2] - place_pose[2]
    if delta_z <= 15:
        robot.MoveJ(place_pose.tolist(), vel=20, blendT=-1, tool=tool, user=user)
        return True

    pose = place_pose.copy()
    start_time = time.time()
    last_z = current_pose[2]
    stop_z = place_pose[2] + 10  # 停止高度
    dz = 2.5  # 步长
    print(f"\n方法二：连续MoveJ+MoveL启动 → 高度差: {delta_z:.1f}mm")
    
    step_count = 0
    while True:
        step_count += 1
        
        if step_count<10:
            pose[2] = last_z - 0.5*step_count*dz  # 向下移动
        else:
            pose[2] = last_z - 5*dz  # 向下移动
        if pose[2] > stop_z:
            last_z = pose[2]
        else:
            break

        with robot_lock:
            # 逆运动学求解
            ret = robot.GetInverseKin(0, pose.tolist(), -1)
            if ret[0] != 0:
                print(f"逆运动学失败，步骤 {step_count}")
                robot.MoveJ(place_pose.tolist(), vel=10, blendT=-1, tool=tool, user=user)
                return False

            # 执行ServoJ
            robot.ServoJ(
                joint_pos=ret[1], 
                axisPos=EXAXIS_POS,
                acc=0.0, 
                vel=0.0, 
                cmdT=0.01, 
                filterT=0.0, 
                gain=0.0
                )
            time.sleep(0.008)
    
    # 使用MoveL完成最后10mm
    print(f"连续ServoJ完成 ({step_count}步)，使用MoveL完成最后行程...")
      
    # err = robot.MoveL(
    #     desc_pos=place_pose,
    #     tool=1,
    #     user=0,
    #     vel=60.0,
    #     acc=-1.0,
    #     ovl=100.0,
    #     blendR=-1.0
    # )

    actual = time.time() - start_time
    print(f"连续MoveJ+MoveL完成！总用时 {actual:.3f}s \n")
    return True


# 主逻辑（测试用）
def main():
    robot = init_robot()
    if not robot:
        return
    test_pose = np.array([360.25, 489.88, 326, -179.999, 0.205, -85.458])  
    start_pose = np.array([360.25, 489.88, 446, -179.999, 0.205, -85.458])
    # test_pose = np.array([360.25, 489.88, 226, -179.999, 0.205, -85.458])
    place_pos = np.array([360.25, 489.88, 195, -179.999, 0.205, -85.458])
    
    print("开始执行加速向下运动测试（MoveJ版本）")

    try:
        index=1
        while index:
            index-=1
            err = robot.MoveL(desc_pos=test_pose, tool=1, user=0, vel=60.0, acc=-1.0, ovl=100.0, blendR=-1.0)
            err = robot.MoveL(desc_pos=start_pose, tool=1, user=0, vel=60.0, acc=-1.0, ovl=100.0, blendR=-1.0)

            # # 执行加速向下运动
            # speed_down(robot, start_pose, place_pos)
            speed_down2(robot, start_pose, place_pos)
            time.sleep(1.0)
            err = robot.MoveL(desc_pos=test_pose, tool=1, user=0, vel=60.0, acc=-1.0, ovl=100.0, blendR=-1.0)

    except KeyboardInterrupt:
        print("程序中断，清理资源")
    finally:
        # 清理资源

        del robot

if __name__ == "__main__":
    main()