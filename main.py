import vision_utils as vu
import fairino2_8
from modules.module_speeddect import speedDetectInit

from logger import LOG_DEBUG, LOG_INFO, LOG_WARN, LOG_ERROR, LOG_CRITICAL

# --- 主函数 main 修改 ---
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



