import pyrealsense2 as rs

# ----------- 列出所有连接的 RealSense 设备 -----------
ctx = rs.context()
devices = ctx.query_devices()
if len(devices) == 0:
    raise RuntimeError("未检测到任何 RealSense 设备！")

print("检测到的 RealSense 设备：")
for i, dev in enumerate(devices):
    print(f"[{i}] {dev.get_info(rs.camera_info.name)} - 序列号: {dev.get_info(rs.camera_info.serial_number)}")