import time
from Robot import RPC

from control_air_close_open import grip_clamp, grip_open, grip_release

detect_pose=[251, 540, 342.979, -180, -0.2, -85.458]
detect_none_pose=[251, 540, 300, -180, -0.2, -85.458]
def movel_to_pose(robot,intercept_pos):
    err = robot.MoveL(desc_pos=intercept_pos, tool=1, user=0,
                    vel=70.0, acc=-1.0, ovl=100.0, blendR=-1.0)
    if err != 0:
        print(f"移动到监测位MoveL 失败")
        return False
    return True

# 初始化机械臂
def init_robot():
    """
    初始化机械臂连接
    返回: RPC对象或None（如果初始化失败）
    """
    robot = RPC(ip="192.168.57.3")
    
    # 机械臂使能并切换到自动模式
    if robot.RobotEnable(1) != 0 or robot.Mode(0) != 0:
        print("机械臂初始化失败")
        return None

    print("机械臂初始化成功")
    return robot


def if_on(robot,id):
    block = 0
    error,di = robot.GetDI(id, block)
    # print(f"di0: {di}")
    # print(f"di1: {di}") #G
    return di

def dect_geted(robot):
    repeat=0
    for i in range(5):
        time.sleep(0.005)  #G
        if if_on(robot,1) == 1:
            repeat+=1
        
        if repeat>2: # G
            break # G
    return repeat>2
def do_dect(robot,pose_dect,pose_set,isfake=False):

    # 移动到检测位
    movel_to_pose(robot,pose_dect)
    time.sleep(0.1)
    if isfake:
        return True
    if dect_geted(robot):
        print("检测到布料，进行放置操作。\n")
        return True
    else:
        print("未检测到布料，进行废弃操作。\n")
        # 移动到放置位
        movel_to_pose(robot,pose_set)
        grip_release(robot)
        time.sleep(0.5)
        # 移回检测位
        movel_to_pose(robot,pose_dect)
        return False


def main():
    robot = init_robot()
    if not robot:
        return
    try:
        # while True: 
        #     res=if_on(robot,1)
        #     print("res",res)
        do_dect(robot,detect_pose,detect_none_pose)
    except KeyboardInterrupt:
        print("程序中断，清理资源")
    finally:
        # 清理资源

        del robot

if __name__ == "__main__":
    main()
