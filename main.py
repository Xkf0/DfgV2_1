import vision_utils as vu # 视觉文件
import fairino2_8 # 机械臂控制文件
from modules.module_speeddect import speedDetectInit # 速度传感器检测文件

def main(): 
    # 机械臂线程
    succeed = fairino2_8.init()
    if succeed is False:
        return

    # 速度传感器线程
    speedDetectInit()

    # 视觉检测线程
    vu.Loop()

if __name__ == "__main__":
    main()