# linear_actuator.py
# 一行代码控制丝杠：move_to(150.5) / move_to(-1) 回原点 / move_to(0) 急停

import serial
import time
from threading import Lock

# ====================== 配置区（只需改这里）======================
SERIAL_PORT = "/dev/ttyUSB0"      # 修改为你的实际串口
BAUD_RATE = 9600
PULSES_PER_MM = 200.0             # 200脉冲/mm（你的参数）
# MOVE_RANGE = (1.0, 220.0)         # 有效行程
MOVE_RANGE = (1.0, 345.0)         # 有效行程
DRIVER_ADDR = 0x01

# 寄存器地址（不要动，已验证正确）
REG_ABS_POS_H   = 0x0024
REG_ABS_POS_L   = 0x0025
REG_RUN_CMD     = 0x0027   # 5=启动绝对运动
REG_HOMING_CMD  = 0x0030   # 1=启动回原点
REG_ESTOP_CMD   = 0x0028   # 0x0001=急停
# =================================================================

# 全局串口 + 线程锁（多线程安全）
_ser = None
_lock = Lock()

def _get_serial():
    """懒加载串口 + 自动初始化"""
    global _ser
    if _ser is None or not _ser.is_open:
        try:
            _ser = serial.Serial(
                port=SERIAL_PORT,
                baudrate=BAUD_RATE,
                bytesize=8, parity='N', stopbits=1,
                timeout=0.1, xonxoff=False, rtscts=False
            )
            print(f"丝杠串口已打开：{SERIAL_PORT}")
            _init_driver()
        except Exception as e:
            raise RuntimeError(f"无法打开串口 {SERIAL_PORT}：{e}")
    return _ser

def _crc16(data: bytes) -> bytes:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return bytes([crc & 0xFF, crc >> 8])

def _write_reg(reg: int, value: int):
    """底层写寄存器（带CRC）"""
    ser = _get_serial()
    frame = bytearray([
        DRIVER_ADDR, 0x06,
        reg >> 8, reg & 0xFF,
        value >> 8, value & 0xFF
    ])
    frame.extend(_crc16(frame))
    
    with _lock:
        ser.write(frame)
        ser.flush()
        time.sleep(0.03)  # 必须延时，否则驱动器跟不上

def _init_driver():
    """上电自动初始化（只执行一次）"""
    cmds = [
        (0x0011, 0x0008),  # 细分1000
        (0x0026, 0x0001),  # 绝对位置模式
        (0x0020, 0x0064),  # 起始速度
        (0x0021, 0x00C8),  # 加速时间
        (0x0022, 0x00C8),  # 减速时间
        (0x0023, 0x03E8),  # 最大速度700r/min
        (0x0010, 0x0006),  # 电流
        (0x0031, 0x0001),  # 回原点模式
        (0x0032, 0x02BC),  # 原点查找速度
        (0x0033, 0x0064),  # 爬行速度
        (0x0034, 0x00C8),  # 回原点加减速
        (0x0043, 0x0001),  # 设置X0输入端信号模式 


    ]
    for reg, val in cmds:
        _write_reg(reg, val)
        time.sleep(0.03)
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
        _write_reg(REG_HOMING_CMD, 1)
        return
    if distance_mm == 0:
        print("急停！")
        _write_reg(REG_ESTOP_CMD, 0x0001)
        return
    
    if not (MOVE_RANGE[0] <= distance_mm <= MOVE_RANGE[1]):
        print(f"超出行程！仅支持 {MOVE_RANGE[0]} ~ {MOVE_RANGE[1]} mm")
        return
    
    pulses = int(round(distance_mm * PULSES_PER_MM))
    print(f"移动到 {distance_mm} mm（{pulses} 脉冲）")
    
    _write_reg(REG_ABS_POS_H, (pulses >> 16) & 0xFFFF)
    _write_reg(REG_ABS_POS_L, pulses & 0xFFFF)
    _write_reg(REG_RUN_CMD, 5)  # 启动运动





if __name__ == "__main__":
    # 测试代码
    move_to(-1)      # 回原点s
    time.sleep(4)    # 等待
    move_to(100)     # 移动
    time.sleep(4)    # 等待
    move_to(200)     # 移动
    time.sleep(4)    # 等待
    move_to(300)     # 移动
    time.sleep(4)    # 等待
    # move_to(335)     # 移动
    # time.sleep(4)    # 等待
    move_to(0)       # 急停

    # move_to(179)

# # 别名（更直观）
# home = lambda: move_to(-1)   # 回原点
# estop = lambda: move_to(0)   # 急停
# # =================================================================

# # 可选：程序退出时自动关闭串口
# import atexit
# atexit.register(lambda: _ser.close() if _ser and _ser.is_open else None)