import serial
import time

class UltrasonicSensor:
    def __init__(self, port="/dev/ttyUSB1", baudrate=9600):
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        # Modbus 指令: 读地址 0002 的寄存器 (距离)
        self.cmd_read_dist = bytes.fromhex("01 03 00 01 00 01 D5 CA")

    def connect(self):
        """初始化并打开串口"""
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1
            )
            print(f" 串口 {self.port} 连接成功")
            return True
        except serial.SerialException as e:
            print(f" 串口连接失败: {e}")
            return False

    def get_distance(self):
        """获取一次距离数据"""
        if not self.ser or not self.ser.is_open:
            print("串口未连接")
            return None

        try:
            self.ser.reset_input_buffer()
            self.ser.write(self.cmd_read_dist)
            time.sleep(0.05) # 等待响应
            
            # 读取返回帧，通常 Modbus 返回格式为: ID(1) + Func(1) + Len(1) + Data(2) + CRC(2)
            recv = self.ser.read(7) 

            if len(recv) >= 5:
                # 简单校验：检查返回的 ID 和功能码是否匹配 (可选)
                # if recv[0] == 0x01 and recv[1] == 0x03:
                
                # 解析数据：高位 * 256 + 低位
                distance_mm = (recv[3] << 8) + recv[4]
                
                # 过滤无效数据 (根据传感器手册，65535或其他特定值可能代表错误)
                if distance_mm > 3000: 
                    return None # 超出量程或错误
                
                return distance_mm
            else:
                return None
                
        except Exception as e:
            print(f"读取错误: {e}")
            return None

    def close(self):
        """关闭串口"""
        if self.ser and self.ser.is_open:
            self.ser.close()
            print(" 串口已关闭")

# --- 使用示例 ---
if __name__ == "__main__":
    sensor = UltrasonicSensor()
    
    if sensor.connect():
        try:
            while True:
                dist = sensor.get_distance()
                if dist is not None:
                    print(f"当前距离: {dist} mm")
                else:
                    print(" 读取超时或数据无效")
                time.sleep(0.5) # 每0.5秒读取一次
        except KeyboardInterrupt:
            print("\n程序退出")
        finally:
            sensor.close()