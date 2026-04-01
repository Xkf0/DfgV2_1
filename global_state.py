import queue
import threading
# global_state.py
class AppState:
    # 静态变量（类变量）作为共享数据
    speed_now = 0
    time_pre_now = 0
    # 1号机械臂队列与锁
    task_queue_1 = queue.Queue()
    task_lock_1 = threading.Lock()
    # 2号机械臂新增队列与锁
    task_queue_2 = queue.Queue()
    task_lock_2 = threading.Lock()
    speed_lock = threading.Lock()  # 新增：速度变量的锁