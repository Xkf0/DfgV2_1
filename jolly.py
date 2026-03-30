import serial
import time

# -------------------------- 核心配置参数 --------------------------
SERIAL_PORT = "/dev/ttyUSB0"  # 替换为实际串口（Windows：COMx；Linux：/dev/ttyUSBx；Mac：/dev/tty.usbserial-xxx）
BAUD_RATE = 9600       # 驱动器波特率（必须与驱动器一致）
TIMEOUT = 0.1          # 串口超时时间（秒）
RS485_DE_RE_PIN = None # 若需控制RS485 DE/RE，需配置GPIO（如树莓派22引脚），无则设为None
DRIVER_ADDR = 0x01     # 驱动器地址

# 硬件与范围参数
PULSES_PER_MM = 200.0   # 200脉冲/mm（1000脉冲/转 ÷ 5mm/转）
MOVE_RANGE_MIN = 1.0    # 最小移动距离（mm）
MOVE_RANGE_MAX = 238.0  # 最大移动距离（mm）

# 寄存器地址定义（与驱动器协议对应）
REG_SUB_DIV = 0x0011    # 细分设置寄存器
REG_CTRL_MODE = 0x0026  # 控制模式寄存器（1=绝对位置模式）
REG_START_SPEED = 0x0020# 起始速度寄存器
REG_ACCEL_TIME = 0x0021 # 加速时间寄存器
REG_DECEL_TIME = 0x0022 # 减速时间寄存器
REG_MAX_SPEED = 0x0023  # 最大速度寄存器
REG_CURRENT = 0x0010    # 电流设置寄存器
REG_ORIGIN_SIG = 0x0043 # 原点信号端子寄存器
REG_HOMING_MODE = 0x0031# 回原点模式寄存器
REG_HOMING_SPEED = 0x0032# 原点查找速度寄存器
REG_HOMING_CRAWL = 0x0033# 原点爬行速度寄存器
REG_HOMING_ACCEL = 0x0034# 回原点加减速寄存器
REG_ABS_POS_H = 0x0024  # 绝对位置高位寄存器
REG_ABS_POS_L = 0x0025  # 绝对位置低位寄存器
REG_RUN_CMD = 0x0027    # 运行命令寄存器（5=启动绝对位置运动）
REG_HOMING_CMD = 0x0030 # 回原点命令寄存器（1=启动回原点）
REG_ESTOP_CMD = 0x0028  # 急停命令寄存器（0x0001=急停）

# -------------------------- Modbus-RTU核心函数 --------------------------
def crc16_modbus(data: bytes) -> bytes:
    """计算Modbus-RTU CRC16校验码（返回2字节，低位在前）"""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if (crc & 0x0001) else crc >> 1
    return bytes([crc & 0xFF, (crc >> 8) & 0xFF])

def modbus_write_reg(ser: serial.Serial, reg_addr: int, reg_value: int):
    """Modbus功能码0x06：写单个寄存器（核心485通信函数）"""
    # 构建Modbus帧：地址(1) + 功能码(1) + 寄存器地址(2) + 寄存器值(2)
    frame = bytearray()
    frame.append(DRIVER_ADDR)
    frame.append(0x06)
    frame.extend(reg_addr.to_bytes(2, byteorder='big'))  # 地址高位在前
    frame.extend(reg_value.to_bytes(2, byteorder='big'))  # 值高位在前
    frame.extend(crc16_modbus(frame))  # 追加CRC校验码
    
    # 控制RS485 DE/RE（发送模式）
    if RS485_DE_RE_PIN is not None:
        # 若使用树莓派GPIO，需添加GPIO库并配置（示例：GPIO.output(RS485_DE_RE_PIN, GPIO.HIGH)）
        pass
    
    # 发送数据
    ser.write(frame)
    ser.flush()
    time.sleep(0.015)  # 等待发送完成（匹配原代码delay(15)）
    
    # 切换RS485为接收模式（若需）
    if RS485_DE_RE_PIN is not None:
        # GPIO.output(RS485_DE_RE_PIN, GPIO.LOW)
        pass

# -------------------------- 业务功能函数 --------------------------
def init_driver(ser: serial.Serial):
    """初始化驱动器（485发送配置指令）"""
    modbus_write_reg(ser, REG_SUB_DIV, 0x0008)    # 细分=1000脉冲/转
    modbus_write_reg(ser, REG_CTRL_MODE, 1)       # 绝对位置模式
    modbus_write_reg(ser, REG_START_SPEED, 0x0064)# 起始速度=10r/min
    modbus_write_reg(ser, REG_ACCEL_TIME, 0x0064) # 加速时间=100ms
    modbus_write_reg(ser, REG_DECEL_TIME, 0x0064) # 减速时间=100ms
    modbus_write_reg(ser, REG_MAX_SPEED, 0x0320)  # 最大速度=800r/min
    modbus_write_reg(ser, REG_CURRENT, 0x0006)    # 电流=2.3A（锁机半流）
    modbus_write_reg(ser, REG_ORIGIN_SIG, 0x0001) # X0端子=原点信号
    modbus_write_reg(ser, REG_HOMING_MODE, 0x0001)# 回原点模式1（反向找原点）
    modbus_write_reg(ser, REG_HOMING_SPEED, 0x0320)# 原点查找速度=800r/min
    modbus_write_reg(ser, REG_HOMING_CRAWL, 0x001E)# 爬行速度=30r/min
    modbus_write_reg(ser, REG_HOMING_ACCEL, 0x0064)# 回原点加减速=100ms
    print("驱动器初始化完成（485配置已发送）！")

def move_to_distance(ser: serial.Serial, distance_mm: float):
    """通过485控制移动到目标距离（1-220mm）"""
    if not (MOVE_RANGE_MIN <= distance_mm <= MOVE_RANGE_MAX):
        print(f"无效距离！仅支持{int(MOVE_RANGE_MIN)}-{int(MOVE_RANGE_MAX)}mm")
        return
    
    # 计算目标脉冲数（四舍五入为整数）
    target_pulses = int(round(distance_mm * PULSES_PER_MM))
    print(f"485发送移动指令：{distance_mm:.1f}mm（脉冲：{target_pulses}）")
    
    # 485发送32位绝对位置（高位+低位寄存器）
    modbus_write_reg(ser, REG_ABS_POS_H, (target_pulses >> 16) & 0xFFFF)
    modbus_write_reg(ser, REG_ABS_POS_L, target_pulses & 0xFFFF)
    modbus_write_reg(ser, REG_RUN_CMD, 5)  # 485发送启动运动指令

# -------------------------- 主程序 --------------------------
def main():
    # 初始化485串口
    try:
        ser = serial.Serial(
            port=SERIAL_PORT,
            baudrate=BAUD_RATE,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=TIMEOUT
        )
    except Exception as e:
        print(f"485串口初始化失败：{e}")
        return
    
    print(f"485串口已打开：{SERIAL_PORT}")
    init_driver(ser)
    print("="*60)
    print("485控制指令说明：")
    print("  输入 -1 → 回原点（485发送回原点命令）")
    print("  输入  0 → 急停（485发送急停命令）")
    print("  输入1-220 → 移动到对应毫米位置（支持小数，如50.5）")
    print("  输入 q → 退出程序")
    print("="*60)
    
    try:
        while True:
            # 读取串口输入指令（控制485发送）
            input_data = input("请输入指令：").strip()
            
            # 退出程序
            if input_data.lower() == 'q':
                print("退出程序...")
                break
            
            # 解析指令
            try:
                cmd_num = float(input_data)
            except ValueError:
                print("无效指令！请输入-1、0、1-220之间的数字，或q退出")
                continue
            
            # 485发送对应指令
            if cmd_num == -1.0:
                print("485发送回原点命令...")
                modbus_write_reg(ser, REG_HOMING_CMD, 1)
            elif cmd_num == 0.0:
                print("485发送急停命令...")
                modbus_write_reg(ser, REG_ESTOP_CMD, 0x0001)
            elif MOVE_RANGE_MIN <= cmd_num <= MOVE_RANGE_MAX:
                move_to_distance(ser, cmd_num)
            else:
                print(f"无效指令！仅支持-1、0、1-220mm（当前输入：{cmd_num}）")
    
    except KeyboardInterrupt:
        print("\n强制退出程序...")
    finally:
        ser.close()
        print("485串口已关闭")

if __name__ == "__main__":
    main()