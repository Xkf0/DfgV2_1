import queue
import threading
import json
def load_config():
    with open("CONFIG.json", "r", encoding="utf-8") as f:
        config = json.load(f)
    return config
CONFIG = load_config()
from config_loader import Configer
# global_state.py
class AppState:
    # 静态变量（类变量）作为共享数据
    cfg_1            = Configer(**CONFIG["CONFIG_PARAMS_1"])
    cfg_2            = Configer(**CONFIG["CONFIG_PARAMS_2"])
    AFFINE_MATRIX_1  = None
    
    # 1号机械臂队列与锁
    task_queue_1     = queue.Queue()
    task_lock_1      = threading.Lock()
    # 2号机械臂新增队列与锁
    task_queue_2     = queue.Queue()
    task_lock_2      = threading.Lock()

    speed_lock       = threading.Lock()  # 速度变量的锁，可以删除
    speed_now        = cfg_1.speed
    time_pre_now     = cfg_1.time_pre

    blocked_state    = False
    state_lock       = threading.Lock() # 后续需要改为条件变量
    wait_detect      = False

    centroid         = []
    centroid_lock    = threading.Lock() # 暂时未使用

    armCanMove       = False
    # armCanMove_lock  = threading.Lock()
    armCanMove_cond   = threading.Condition()

    changeScrew      = True
    changeScrew_lock = threading.Lock() # 可删除

    graspSucceed     = False
    detectSucceed_cond= threading.Condition()
    detectNow         = False