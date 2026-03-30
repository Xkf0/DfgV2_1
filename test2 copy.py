
import time
import numpy as np
from Robot import RPC
from linear_actuator import move_to
from detect_test import do_dect
from control_air_close_open import grip_clamp, grip_open, grip_release, start_suction, stop_suction
import threading

robot_lock = threading.Lock()
if_grap=True
# if_grap=True
# 初始化机械臂
def init_robot():
    robot = RPC(ip="192.168.57.4")
    
    time.sleep(0.5)    # 等待0.5秒

    # 机械臂使能并切换到自动模式
    if robot.RobotEnable(1) != 0 or robot.Mode(0) != 0:
        print("机械臂初始化失败")
        return None
    ret = robot.GetSlaveHardVersion()
    print("GetSlaveHardVersion()：", ret)
    return robot

def movel_to_pose(robot,intercept_pos,vel=70.0):
    err = robot.MoveL(desc_pos=intercept_pos, tool=1, user=2,
                    vel=vel, acc=-1.0, ovl=100.0, blendR=50.0)
    if err != 0:
        print(f"MoveL 失败")
        return False
    return True

def static_grap(robot, up_pose, wait_pose, grasp_pose,stop_pose, place_pose):
    """静态抓取函数"""
    # move to wait
    time.sleep(2)    # 等待2秒
    with robot_lock:
        err =movel_to_pose(robot,up_pose)
         
        if err == 0:
            print(f"移动到上位姿失败，错误码: {err}")
            return False
        print("=== 抓取开始 ===")
    
    # move to wait
    with robot_lock:
        err =movel_to_pose(robot,wait_pose)
        if err == 0:
            print(f"移动到等待位姿失败，错误码: {err}")
            return False
    print("抓取等待位置")
    if if_grap:
        grip_open(robot)
        start_suction(robot)
        
    time.sleep(1.08)
    
    # move to down
    with robot_lock:
        err =movel_to_pose(robot,grasp_pose)
        if err == 0:
            print(f"移动到抓取位姿失败，错误码: {err}")
            return False
    print("抓取位姿")
    time.sleep(1)
    if if_grap:
        grip_clamp(robot)
    time.sleep(1.5)  # 等待夹爪闭合
    

    # move to up
    with robot_lock:
        grasp_pose2=grasp_pose.copy()
        grasp_pose2[2]+=10
        err =movel_to_pose(robot,grasp_pose2,vel=5)
        # time.sleep(0.01) 
        time.sleep(0.5) 
        err =movel_to_pose(robot,stop_pose)
        if err == 0:
            print(f"移动到上位姿失败，错误码: {err}")
            return False
    print("抬起位姿")

    # move to up
    with robot_lock:
        err =movel_to_pose(robot,place_pose)
        # time.sleep(0.01) 
        if err == 0:
            print(f"移动到放置位置失败，错误码: {err}")
            return False
    print("放置")

    if if_grap:
        grip_release(robot)
        stop_suction(robot)

    time.sleep(3) 

    with robot_lock:
        err =movel_to_pose(robot,up_pose)
        # time.sleep(0.01) 
        if err == 0:
            print(f"移动到安全位置失败，错误码: {err}")
            return False
    print("放置")
    return True
# 主逻辑（测试用）
def main():
    robot = init_robot()
    if not robot:
        return
        
    # 定义位姿
    # up_pose    = [-65, 656, 350.6, -180, -0.2, -85.458]
    # wait_pose  = [-65, 656, 220.6, -180, -0.2, -85.451]
    # grasp_pose = [-65, 656, 180.0, -180, -0.2, -85.458]
    # stop_pose  = [-65, 656, 340.6, -180, -0.2, -85.458]

    up_pose    = [1390.239,
        -326.035,
        208.205,
        179.518,
        -0.283,
        -64.101]
    wait_pose  = [1328.845,
        171.842,
        94.777,
        179.518,
        -0.283,
        -64.101]
    grasp_pose = [1328.845,
        171.842,
        86.591,
        179.518,
        -0.283,
        -64.101]
    stop_pose  = [1390.239,
        -326.035,
        208.205,
        179.518,
        -0.283,
        -64.101]
    place_pose = [1390.239,
        -326.035,
        5.137,
        179.518,
        -0.283,
        -64.101]

    try:
        # 执行静态抓取
        success = static_grap(robot, up_pose, wait_pose, grasp_pose,stop_pose, place_pose)
        if success:
            print("抓取操作完成")
        else:
            print("抓取操作失败")

    except KeyboardInterrupt:
        print("程序中断，清理资源")
    finally:
        # 清理资源
        # robot.RobotEnable(0)
        del robot
        

if __name__ == "__main__":
    main()
