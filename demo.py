from motor_control import KINCO_Motor
import time

# 1. 创建电机对象
motor = KINCO_Motor(node_id=1)
motor.start()
motor.enable()
motor.stop()

# try:
#     # 2. 初始化
#     motor.start()
#     motor.enable()

#     # 3. 设置参数
#     motor.set_accel_cmss(30)
#     motor.set_decel_cmss(5)
#     motor.set_speed(20)

#     # 4. 循环实时打印速度
#     while True:
#         print(f"速度：{motor.get_speed():.2f} cm/s")
#         time.sleep(0.1)

# # ======================
# # 按下 Ctrl+C 会触发这里
# # ======================
# except KeyboardInterrupt:
#     print("\n⚠️  检测到 Ctrl+C，正在停止电机...")
    
#     # 停止 + 失能
#     motor.stop()
#     time.sleep(0.2)
#     motor.disable()

#     print("✅ 电机已安全停止")
