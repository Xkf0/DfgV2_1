# linear_actuator.py
# 一行代码控制丝杠：move_to(150.5) / move_to(-1) 回原点 / move_to(0) 急停

import time
from threading import RLock

from host_ui.actuator_client import ActuatorSettings, LinearActuatorClient
from host_ui.protocol import crc16_modbus, encode_write_register

# ====================== 配置区（只需改这里）======================
SERIAL_PORT = "/dev/ttyUSB0"      # 修改为你的实际串口
BAUD_RATE = 9600
PULSES_PER_MM = 200.0             # 200脉冲/mm（你的参数）
# MOVE_RANGE = (1.0, 220.0)         # 有效行程
MOVE_RANGE = (1.0, 210.0)         # 有效行程
DRIVER_ADDR = 0x01

# 寄存器地址（不要动，已验证正确）
REG_ABS_POS_H   = 0x0024
REG_ABS_POS_L   = 0x0025
REG_RUN_CMD     = 0x0027   # 5=启动绝对运动
REG_HOMING_CMD  = 0x0030   # 1=启动回原点
REG_ESTOP_CMD   = 0x0028   # 0x0001=急停
# =================================================================

# 全局客户端 + 线程锁（多线程安全）
_client = None
_lock = RLock()

def _build_client() -> LinearActuatorClient:
    settings = ActuatorSettings(
        driver_addr=DRIVER_ADDR,
        pulses_per_mm=PULSES_PER_MM,
        move_range_min=MOVE_RANGE[0],
        move_range_max=MOVE_RANGE[1],
        baud_rate=BAUD_RATE,
    )
    return LinearActuatorClient.from_serial(port=SERIAL_PORT, baud_rate=BAUD_RATE, settings=settings)


def _get_client() -> LinearActuatorClient:
    """懒加载客户端 + 自动初始化"""
    global _client
    if _client is None:
        try:
            _client = _build_client()
            _client.connect()
            print(f"丝杠串口已打开：{SERIAL_PORT}")
            _init_driver()
        except Exception as e:
            _client = None
            raise RuntimeError(f"无法打开串口 {SERIAL_PORT}：{e}")
    elif not _client.is_connected():
        _client.connect()
    return _client

def _crc16(data: bytes) -> bytes:
    crc = crc16_modbus(data)
    return bytes([crc & 0xFF, (crc >> 8) & 0xFF])

def _write_reg(reg: int, value: int):
    """底层写寄存器（带CRC，兼容旧接口）"""
    client = _get_client()
    _ = encode_write_register(DRIVER_ADDR, reg, value)  # 维持协议层复用路径（调试时可抓包）
    with _lock:
        client.write_register(reg, value)
        time.sleep(0.015)  # 与旧实现保持节奏，避免设备侧跟不上

def _init_driver():
    """上电自动初始化（只执行一次）"""
    cmds = [
        (0x0011, 0x0008),  # 细分1000
        (0x0026, 0x0001),  # 绝对位置模式
        (0x0020, 0x0064),  # 起始速度
        (0x0021, 0x0064),  # 加速时间
        (0x0022, 0x0064),  # 减速时间
        (0x0023, 0x0320),  # 最大速度800r/min
        (0x0010, 0x0006),  # 电流
        (0x0031, 0x0001),  # 回原点模式
        (0x0032, 0x0320),  # 原点查找速度
        (0x0033, 0x001E),  # 爬行速度
        (0x0034, 0x0064),  # 回原点加减速
    ]
    for reg, val in cmds:
        _write_reg(reg, val)
        time.sleep(0.01)
    print("丝杠驱动器初始化完成")

# ====================== 对外暴露的超级简洁API ======================
def move_to(distance_mm: float):
    """
    丝杠移动核心函数（一行调用）
    
    参数:
        distance_mm: 
            -1     → 回原点
             0     → 急停
             1~220 → 移动到指定位置（支持小数，如 88.8）
    """
    if distance_mm == -1:
        print("执行回原点...")
        with _lock:
            _get_client().home()
        return
    if distance_mm == 0:
        print("急停！")
        with _lock:
            _get_client().estop()
        return
    
    if not (MOVE_RANGE[0] <= distance_mm <= MOVE_RANGE[1]):
        print(f"超出行程！仅支持 {MOVE_RANGE[0]} ~ {MOVE_RANGE[1]} mm")
        return
    
    pulses = int(round(distance_mm * PULSES_PER_MM))
    print(f"移动到 {distance_mm} mm（{pulses} 脉冲）")

    with _lock:
        _get_client().move_to(distance_mm)





if __name__ == "__main__":
    # 测试代码
    move_to(-1)      # 回原点
    time.sleep(0.5)    # 等待5秒
    move_to(210)   # 移动到150.5mm
    time.sleep(0.5)    # 等待5秒
    move_to(100)   # 移动到150.5mm
    time.sleep(0.5)    # 等待5秒
    move_to(200)   # 移动到150.5mm
    time.sleep(0.5)    # 等待5秒
    move_to(0)       # 急停    

# # 别名（更直观）
# home = lambda: move_to(-1)   # 回原点
# estop = lambda: move_to(0)   # 急停
# # =================================================================

# # 可选：程序退出时自动关闭串口
# import atexit
# atexit.register(lambda: _client.disconnect() if _client and _client.is_connected() else None)
