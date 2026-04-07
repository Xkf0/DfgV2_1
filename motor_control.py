import ctypes
import time
import threading

# 周立功CAN驱动定义
VCI_USBCAN2 = 4
ERR_SUCCESS = 1

# CAN帧结构体
class VCI_CAN_OBJ(ctypes.Structure):
    _fields_ = [
        ("ID", ctypes.c_uint),
        ("TimeStamp", ctypes.c_uint),
        ("TimeFlag", ctypes.c_ubyte),
        ("SendType", ctypes.c_ubyte),
        ("RemoteFlag", ctypes.c_ubyte),
        ("ExternFlag", ctypes.c_ubyte),
        ("DataLen", ctypes.c_ubyte),
        ("Data", ctypes.c_ubyte * 8),
        ("Reserved", ctypes.c_ubyte * 3)
    ]

canDLL = ctypes.cdll.LoadLibrary("/home/xf/hangzhou/DfgV2_1/libcontrolcan.so")

# 电机控制类
class KINCO_Motor:
    def __init__(self, dev_type=VCI_USBCAN2, dev_idx=0, can_idx=0, node_id=1):
        self.dev_type = dev_type
        self.dev_idx = dev_idx
        self.can_idx = can_idx
        self.node_id = node_id
        self.actual_rpm = 0
        self.running = False

    # 一键初始化
    def start(self):
        if not self.can_init():
            return False
        self.nmt_start_node()
        self.set_speed_mode()
        print("✅ 电机初始化完成，等待控制指令")
        return True

    # CAN初始化
    def can_init(self):
        ret = canDLL.VCI_OpenDevice(self.dev_type, self.dev_idx, 0)
        if ret != ERR_SUCCESS:
            print("❌ 打开设备失败")
            return False
        print("✅ 调用 VCI_OpenDevice成功")

        class VCI_INIT_CONFIG(ctypes.Structure):
            _fields_ = [
                ("AccCode", ctypes.c_uint),
                ("AccMask", ctypes.c_uint),
                ("Reserved", ctypes.c_uint),
                ("Filter", ctypes.c_ubyte),
                ("Timing0", ctypes.c_ubyte),
                ("Timing1", ctypes.c_ubyte),
                ("Mode", ctypes.c_ubyte)
            ]

        init_config = VCI_INIT_CONFIG()
        init_config.AccCode = 0x00000000
        init_config.AccMask = 0xFFFFFFFF
        init_config.Filter = 1
        init_config.Timing0 = 0x00
        init_config.Timing1 = 0x1C
        init_config.Mode = 0

        ret = canDLL.VCI_InitCAN(self.dev_type, self.dev_idx, self.can_idx, ctypes.byref(init_config))
        if ret != ERR_SUCCESS:
            print("❌ 初始化CAN失败")
            return False
        print("✅ 调用 VCI_InitCAN成功")

        ret = canDLL.VCI_StartCAN(self.dev_type, self.dev_idx, self.can_idx)
        if ret != ERR_SUCCESS:
            print("❌ 启动CAN失败")
            return False
        print("✅ 调用 VCI_StartCAN成功")

        self.running = True
        threading.Thread(target=self._recv_thread, daemon=True).start()
        return True

    # CAN发送
    def can_send(self, can_id, data):
        frame = VCI_CAN_OBJ()
        frame.ID = can_id
        frame.SendType = 0
        frame.RemoteFlag = 0
        frame.ExternFlag = 0
        frame.DataLen = 8
        for i in range(8):
            frame.Data[i] = data[i] if i < len(data) else 0
        ret = canDLL.VCI_Transmit(self.dev_type, self.dev_idx, self.can_idx, ctypes.byref(frame), 1)
        return ret == ERR_SUCCESS

    # NMT启动
    def nmt_start_node(self):
        data = [0x01, self.node_id, 0,0,0,0,0,0]
        self.can_send(0x000, data)
        print("✅ NMT 启动节点命令已发送")
        time.sleep(0.2)

    # 设置速度模式
    def set_speed_mode(self):
        data = [0x2F, 0x60, 0x60, 0x00, 0x03, 0x00, 0x00, 0x00]
        self.can_send(0x600 + self.node_id, data)
        print("✅ 已设置：速度模式")
        time.sleep(0.2)

    # 使能
    def enable(self):
        data = [0x2B, 0x40, 0x60, 0x00, 0x0F, 0x00, 0x00, 0x00]
        self.can_send(0x600 + self.node_id, data)
        print("✅ 电机已使能")
        time.sleep(0.5)

    # 失能
    def disable(self):
        data = [0x2B, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00]
        self.can_send(0x600 + self.node_id, data)
        print("✅ 电机已失能")

    # 直接设置传送带线速度 cm/s
    # 已包含 1:10 减速器
    def set_speed(self, cm_s):
        # 线速度 → 实际输出轴转速 → 电机转速（×10）
        rpm = cm_s / 0.01937
        dec = int((rpm * 512 * 65536) / 1875 + 0.5)
        data = [0x23, 0xFF, 0x60, 0x00, dec&0xFF, (dec>>8)&0xFF, (dec>>16)&0xFF, (dec>>24)&0xFF]
        self.can_send(0x600 + self.node_id, data)
        print(f"✅ 传送带速度: {cm_s:.2f} cm/s")

    # 停止
    def stop(self):
        self.set_speed(0)
        print("✅ 电机已停止")

    # 获取实际线速度 cm/s
    # 已包含 1:10 减速器
    def get_speed(self):
        return self.actual_rpm * 0.01937
    

        # ==========================
    # 一步到位：直接设置 传送带加速度 cm/s²
    # 内部自动 → rps/s → DEC → 发指令
    # ==========================
    def set_accel_cmss(self, a_cm):
        # 1. cm/s² → 电机 rps/s
        a_rps = a_cm / 1.16239

        # 2. 按你指定公式算 DEC
        dec = int((a_rps * 65536 * 65536) / 4000000 + 0.5)

        # 3. 发加速度指令 0x6083
        data = [0x23, 0x83, 0x60, 0x00,
                dec & 0xFF,
                (dec >> 8) & 0xFF,
                (dec >> 16) & 0xFF,
                (dec >> 24) & 0xFF]
        self.can_send(0x600 + self.node_id, data)
        print(f"✅ 加速度: {a_cm:.2f} cm/s²  (电机:{a_rps:.2f} rps/s)")

    def set_decel_cmss(self, d_cm):
        # 1. cm/s² → 电机 rps/s
        d_rps = d_cm / 1.16239

        # 2. 按你指定公式算 DEC
        dec = int((d_rps * 65536 * 65536) / 4000000 + 0.5)

        # 3. 发减速度指令 0x6084
        data = [0x23, 0x84, 0x60, 0x00,
                dec & 0xFF,
                (dec >> 8) & 0xFF,
                (dec >> 16) & 0xFF,
                (dec >> 24) & 0xFF]
        self.can_send(0x600 + self.node_id, data)
        print(f"✅ 减速度: {d_cm:.2f} cm/s²  (电机:{d_rps:.2f} rps/s)")


    # TPDO 解析线程
    def _recv_thread(self):
        while self.running:
            frame = VCI_CAN_OBJ()
            ret = canDLL.VCI_Receive(self.dev_type, self.dev_idx, self.can_idx, ctypes.byref(frame), 1, 200)
            if ret > 0 and frame.ID == 0x180 + self.node_id and frame.DataLen >= 6:
                raw = frame.Data[2] | (frame.Data[3]<<8) | (frame.Data[4]<<16) | (frame.Data[5]<<24)
                if raw & 0x80000000:
                    raw -= 0x100000000
                self.actual_rpm = (raw * 1875) // (512 * 65536)



if __name__ == "__main__":
    motor = KINCO_Motor(node_id=1)
    motor.start()
    motor.enable()

    # 设置传送带 10 cm/s
    motor.set_speed(15)
    
    motor.set_accel_cmss(30)
    motor.set_decel_cmss(5)

    # 实时输出线速度
    try:
        while True:
            print(f" {motor.get_speed():.2f} cm/s")
            time.sleep(0.1)
    except KeyboardInterrupt:
        motor.stop()
        motor.disable()