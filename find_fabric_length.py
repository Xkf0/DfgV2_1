# main.py
#!/usr/bin/env python3
import time
import cv2
import threading
import datetime  # 导入时间模块，用于获取当前时间

# 引入 Configer 和新的 ObjectDetector
from object_detector import ObjectDetector
from camera_handler1 import CameraHandler  # 新增导入

# 自定义模块导入
import vision_utils as vu
from get_points import get_points
from global_state import AppState


# AFFINE_MATRIX = None
AFFINE_MATRIX_1 = None
AFFINE_MATRIX_2 = None

IS_USE_SAM = True
# IS_USE_SAM = False

# --- 主函数 main 修改 ---
def main():
    global AFFINE_MATRIX_1,AFFINE_MATRIX_2
    # 初始化两个机械臂（假设init_robot支持指定IP，若同IP则传相同参数）
    # robot_mode=3

    # 2. 初始化相机（使用新的CameraHandler类）
    try:
        camera = CameraHandler(
            is_real_sense=AppState.cfg_1.is_real_sense,
            camera_id=AppState.cfg_1.camera_no,
            warmup_frames=30  # 前30帧预热
        )
        print("相机初始化完成")
    except Exception as e:
        print(f"相机初始化失败: {e}")
        return

    # 3. 加载坐标标定并计算仿射矩阵
    # ROBOT_POINTS, REAL_PTS = get_points(cfg.points_file_path)
    # AFFINE_MATRIX = vu.compute_affine_transform(REAL_PTS, ROBOT_POINTS)
    ROBOT_POINTS_1, REAL_PTS_1 = get_points(AppState.cfg_1.points_file_path)
    ROBOT_POINTS_2, REAL_PTS_2 = get_points(AppState.cfg_2.points_file_path)
    AFFINE_MATRIX_1 = vu.compute_affine_transform(REAL_PTS_1, ROBOT_POINTS_1)
    AFFINE_MATRIX_2 = vu.compute_affine_transform(REAL_PTS_2, ROBOT_POINTS_2) # G 需要根据右侧机械臂调整
     
    
    print("仿射矩阵计算完成")

    # 4. 初始化对象检测器
    detector = ObjectDetector(AppState.cfg_1, IS_USE_SAM)

    print("开始主循环...")

    try:
        while True:
            time_start = time.perf_counter()
            
            # 获取最新帧（使用新的相机类）
            current_t = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]  # 格式：年-月-日 时:分:秒.毫秒
            content = f"{current_t} 准备获得图像帧\n"  # 加换行符，让每条记录单独一行

            frame = camera.get_frame_directly()
            if frame is None:
                print("未能获取有效帧，跳过本次循环")
                continue

            # 获取最新帧（使用新的相机类）
            current_t = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]  # 格式：年-月-日 时:分:秒.毫秒
            content = f"{current_t} 已获得图像帧\n"  # 加换行符，让每条记录单独一行
                    
            current_time = time.perf_counter()
            
            # 让检测器递增帧计数器
            detector.increment_frame_counter()

            # 尝试初始化背景
            detector.initialize_background(frame)
            warped_frame, mask, display_img = detector.detect_and_track(frame, AFFINE_MATRIX_1, current_time, 0)# G 这里AFFINE_MATRIX是用来计算机械臂的移动方向（当前并未使用，使用的是写死的值）
            
            
            # 获取检测结果
            motion_dict = detector.get_motion_dict()
            tracked_objects = detector.get_tracked_objects()

            # 绘制FPS
            fps = 1 / (time.perf_counter() - time_start)
            cv2.putText(display_img, f"FPS:{fps:.2f}", (AppState.cfg_1.output_w - 500, AppState.cfg_1.output_h - 20), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 8)

            # 显示或保存
            display_img = cv2.resize(display_img, (display_img.shape[1]//3, display_img.shape[0]//3))
            
            cv2.imshow('combined1', display_img)
            cv2.waitKey(1)
            
            current_t = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]  # 格式：年-月-日 时:分:秒.毫秒
            content = f"{current_t} 结束当前图像帧\n"  # 加换行符，让每条记录单独一行
            # print(f'{content} ----------------------- ')
                    
            

    finally:
        # 清理资源
        print("正在清理资源...")
        
        # 停止相机
        camera.stop()
        
        cv2.destroyAllWindows()
        print("程序退出")

if __name__ == "__main__":
    main()



