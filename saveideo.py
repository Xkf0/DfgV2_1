import pyrealsense2 as rs
import numpy as np
import cv2
import datetime

# 配置深度和颜色流
pipeline = rs.pipeline()
config = rs.config()

# 启用彩色流：1920x1080，30fps，RGB格式
config.enable_stream(rs.stream.color, 1920, 1080, rs.format.bgr8, 30)

# 开始流
pipeline.start(config)

# 准备视频写入器
fourcc = cv2.VideoWriter_fourcc(*'XVID')
timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
out = cv2.VideoWriter(f'realsense_video_{timestamp}.avi', 
                     fourcc, 30.0, (1920, 1080))

try:
    while True:
        # 等待帧
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        
        if not color_frame:
            continue
        
        # 转换为numpy数组
        color_image = np.asanyarray(color_frame.get_data())
        
        # 显示视频
        cv2.imshow('RealSense D415', color_image)
        
        # 写入视频文件
        out.write(color_image)
        
        # 按'q'退出
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
            
finally:
    # 清理
    pipeline.stop()
    out.release()
    cv2.destroyAllWindows()
    print(f"视频已保存")
