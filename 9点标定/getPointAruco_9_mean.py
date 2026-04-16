import cv2
import numpy as np
import pyrealsense2 as rs
import csv  # 新增：用于CSV文件操作
import os  # 新增：用于判断文件是否存在
from camera_handler_jiaqin import CameraHandler

# -------------------------------
# 全局常量（新增CSV文件路径）
# -------------------------------
CSV_FILE_PATH = "cv_aruco_0.csv"  # CSV文件保存路径

# -------------------------------
# ArUco 参数
# -------------------------------
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
if hasattr(cv2.aruco, "DetectorParameters_create"):
    aruco_params = cv2.aruco.DetectorParameters_create()
else:
    aruco_params = cv2.aruco.DetectorParameters()

# 新 API: ArucoDetector
if hasattr(cv2.aruco, "ArucoDetector"):
    detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
else:
    detector = None

# 四个角的 ArUco ID
ARUCO_IDS_CORNER = {
    'top_left': 0,
    'top_right': 8,
    'bottom_right': 24,
    'bottom_left': 16
}

# 输出矩形尺寸（像素）
# REAL_W, REAL_H = 64, 92.05   # cm
REAL_W, REAL_H = 85, 41.1   # cm
RATIO_WH = 20

OUTPUT_W, OUTPUT_H = int(REAL_W * RATIO_WH), int(REAL_H * RATIO_WH)
dst_points = np.float32([
    [0, 0],   #左上
    [OUTPUT_W - 1, 0],   #右上
    [OUTPUT_W - 1, OUTPUT_H - 1],   #右下
    [0, OUTPUT_H - 1]   #左下
])

# -------------------------------
# 配置参数
# -------------------------------
IS_REAL_SENSE = False

# TODO: 修改为你的RealSense设备编号
# REAL_SENSE_NO = '204222061636'
REAL_SENSE_NO = 2
AUTO_CALIBRATION_FRAME = 60  # 在第60帧自动校准
OUTPUT_INTERVAL_FRAMES = 100  # 每100帧输出一次坐标均值

# -------------------------------
# 工具函数（新增CSV辅助函数，修改打印函数）
# -------------------------------
def detect_aruco_corners(frame):
    """检测 ArUco 并返回四个角的指定角点字典"""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    if detector is not None:
        corners_list, ids, _ = detector.detectMarkers(gray)
    else:
        # fallback 方案，老版本可能使用 detectMarkers
        corners_list, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)

    if ids is None:
        return None

    id_to_corners = {}
    ids = ids.flatten()
    for corner, id_val in zip(corners_list, ids):
        c = corner.reshape(4, 2)
        id_to_corners[id_val] = c

    # 检查是否检测到所有四个角点
    required_ids = list(ARUCO_IDS_CORNER.values())
    missing_ids = [id_val for id_val in required_ids if id_val not in id_to_corners]

    if missing_ids:
        print(f"缺少ArUco标记: {missing_ids}")
        return None

    # 返回四个角的顺序 [左上, 右上, 右下, 左下]
    try:
        # ArUco角点顺序：[左上, 右上, 右下, 左下]
        top_left_corners = id_to_corners[ARUCO_IDS_CORNER['top_left']]  # ID: 0
        top_right_corners = id_to_corners[ARUCO_IDS_CORNER['top_right']]  # ID: 8
        bottom_right_corners = id_to_corners[ARUCO_IDS_CORNER['bottom_right']]  # ID: 24
        bottom_left_corners = id_to_corners[ARUCO_IDS_CORNER['bottom_left']]  # ID: 16

        # 使用四个透视变换码的指定角点
        src_pts = np.float32([
            top_left_corners[3],  # 左上ArUco的右下角点     2
            top_right_corners[2],  # 右上ArUco的左下角点    3
            bottom_right_corners[1],  # 右下ArUco的左上角点    0
            bottom_left_corners[0]  # 左下ArUco的右上角点    1
        ])
        return src_pts
    except KeyError as e:
        print(f"提取角点时出错: {e}")
        return None


def detect_aruco_in_warped_frame(warped_frame, points_history):
    """在warped_frame中检测所有ArUco码，记录坐标历史"""
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
    if hasattr(cv2.aruco, "DetectorParameters_create"):
        aruco_params = cv2.aruco.DetectorParameters_create()
    else:
        aruco_params = cv2.aruco.DetectorParameters()

    # 新 API: ArucoDetector
    if hasattr(cv2.aruco, "ArucoDetector"):
        detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
    else:
        detector = None
        
    gray = cv2.cvtColor(warped_frame, cv2.COLOR_BGR2GRAY)

    if detector is not None:
        corners_list, ids, _ = detector.detectMarkers(gray)
    else:
        corners_list, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)

    aruco_info = []

    if ids is not None:
        ids = ids.flatten()
        for i in range(len(ids)):
            # 获取ArUco的四个角点
            corners = corners_list[i].reshape(4, 2)
            
            # 计算ArUco中心点
            center_x_fp = np.mean(corners[:, 0])
            center_y_fp = np.mean(corners[:, 1])
            center_x = int(center_x_fp)
            center_y = int(center_y_fp)

            # 记录坐标到历史数据
            aruco_id = ids[i]
            if aruco_id not in points_history:
                points_history[aruco_id] = {
                    'x_coords': [],
                    'y_coords': []
                }
            
            # 添加当前坐标到历史列表
            points_history[aruco_id]['x_coords'].append(center_x_fp)
            points_history[aruco_id]['y_coords'].append(center_y_fp)

            aruco_info.append({
                'id': aruco_id,
                'corners': corners,
                'selected_corner': (center_x, center_y),
                'corner_name': "中心",
                'center': (center_x, center_y)
            })

    return aruco_info


def calculate_trimmed_mean(coords):
    """
    计算剔除最大10%和最小10%后的均值
    :param coords: 坐标列表
    :return: 修剪后的均值
    """
    if len(coords) < 10:  # 数据量不足时直接返回普通均值
        return np.mean(coords) if coords else 0
    
    # 排序坐标
    sorted_coords = sorted(coords)
    
    # 计算需要剔除的数量（10%）
    trim_count = int(len(sorted_coords) * 0.1)
    
    # 剔除首尾10%，取中间80%
    trimmed_coords = sorted_coords[trim_count:-trim_count]
    
    # 返回均值
    return np.mean(trimmed_coords) if trimmed_coords else 0

# 新增：CSV更新辅助函数
def update_aruco_csv(aruco_id, cv_x, cv_y, csv_path=CSV_FILE_PATH):
    """
    更新ArUco坐标CSV文件，将cv_x、cv_y写入对应id的列
    :param aruco_id: ArUco标记ID
    :param cv_x: X坐标均值
    :param cv_y: Y坐标均值
    :param csv_path: CSV文件路径
    """
    # 定义CSV列头
    csv_headers = ['id', 'robot_x', 'robot_y', 'robot_z', 'cv_x', 'cv_y', 'cv_z']
    # 存储所有CSV数据（字典列表）
    csv_data = []
    
    # 1. 判断CSV文件是否存在
    if os.path.exists(csv_path):
        # 读取现有CSV数据
        with open(csv_path, 'r', newline='', encoding='utf-8') as csv_file:
            csv_reader = csv.DictReader(csv_file)
            for row in csv_reader:
                # 转换id为整数，方便后续比较和排序
                row['id'] = int(row['id'])
                # 转换已有cv_x、cv_y为浮点型（若存在）
                row['cv_x'] = float(row['cv_x']) if row['cv_x'] else 0.0
                row['cv_y'] = float(row['cv_y']) if row['cv_y'] else 0.0
                csv_data.append(row)
    
    # 2. 检查当前id是否已存在于CSV数据中
    id_exists = False
    for row in csv_data:
        if row['id'] == aruco_id:
            # 更新对应id的cv_x、cv_y
            row['cv_x'] = round(cv_x, 2)
            row['cv_y'] = round(cv_y, 2)
            id_exists = True
            break
    
    # 3. 若id不存在，添加新行（其他列暂时填充为空字符串，后续可补充）
    if not id_exists:
        new_row = {
            'id': aruco_id,
            'robot_x': '',
            'robot_y': '',
            'robot_z': '',
            'cv_x': round(cv_x, 2),
            'cv_y': round(cv_y, 2),
            'cv_z': ''
        }
        csv_data.append(new_row)
    
    # 4. 按id从小到大排序
    csv_data.sort(key=lambda x: x['id'])
    
    # 5. 写入CSV文件（覆盖原有内容，保证数据最新且有序）
    with open(csv_path, 'w', newline='', encoding='utf-8') as csv_file:
        csv_writer = csv.DictWriter(csv_file, fieldnames=csv_headers)
        # 写入列头
        csv_writer.writeheader()
        # 写入数据行
        csv_writer.writerows(csv_data)


def print_aruco_averages(points_history, current_frame):
    """输出每个ArUco的修剪后坐标均值，并写入CSV文件"""
    print(f"\n{'='*80}")
    print(f"第 {current_frame} 帧 - ArUco坐标均值（剔除首尾10%）")
    print(f"{'='*80}")
    
    for aruco_id in sorted(points_history.keys()):
        x_coords = points_history[aruco_id]['x_coords']
        y_coords = points_history[aruco_id]['y_coords']
        
        if not x_coords or not y_coords:
            continue
            
        # 计算修剪后的均值
        x_mean = calculate_trimmed_mean(x_coords)
        y_mean = calculate_trimmed_mean(y_coords)
        
        # 输出信息
        print(f"ArUco ID {aruco_id}:")
        print(f"  - 采样数量: {len(x_coords)} 个")
        print(f"  - X坐标均值: {x_mean:.2f} 像素")
        print(f"  - Y坐标均值: {y_mean:.2f} 像素")
        print(f"  - 原始X范围: [{min(x_coords):.2f}, {max(x_coords):.2f}]")
        print(f"  - 原始Y范围: [{min(y_coords):.2f}, {max(y_coords):.2f}]")
        print()
        
        # 新增：调用函数写入/更新CSV文件
        update_aruco_csv(aruco_id, x_mean, y_mean)
    
    print(f"{'='*80}\n")
    # 新增：提示CSV文件更新状态
    print(f"CSV文件已更新：{os.path.abspath(CSV_FILE_PATH)}")


def draw_aruco_markers(frame, src_points):
    """在图像上绘制检测到的ArUco标记和连线"""
    frame_with_markers = frame.copy()

    if src_points is not None:
        # 绘制角点
        for i, point in enumerate(src_points):
            x, y = int(point[0]), int(point[1])
            cv2.circle(frame_with_markers, (x, y), 10, (0, 255, 0), -1)
            cv2.putText(frame_with_markers, f"P{i}", (x + 15, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        # 绘制连线
        points_int = src_points.astype(int)
        cv2.polylines(frame_with_markers, [points_int], True, (255, 0, 0), 3)

    return frame_with_markers


def draw_aruco_on_warped_frame(warped_frame, aruco_info):
    """在warped_frame上绘制检测到的ArUco码和关键点"""
    frame_with_aruco = warped_frame.copy()

    if not aruco_info:
        return frame_with_aruco

    for info in aruco_info:
        id_val = info['id']
        corners = info['corners']
        selected_corner = info['selected_corner']
        corner_name = info['corner_name']
        center = info['center']

        # 1. 绘制ArUco边界框
        corners_int = corners.astype(int)
        cv2.polylines(frame_with_aruco, [corners_int], True, (0, 255, 255), 2)

        # 2. 绘制关键点（红色）
        corner_x, corner_y = int(selected_corner[0]), int(selected_corner[1])
        color = (0, 0, 255)

        cv2.circle(frame_with_aruco, (corner_x, corner_y), 8, color, -1)
        cv2.circle(frame_with_aruco, (corner_x, corner_y), 10, (255, 255, 255), 2)

        # 3. 显示ID和坐标
        cv2.putText(frame_with_aruco, f"ID:{id_val}", (center[0] - 20, center[1]),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        # 在关键点附近显示坐标
        coord_text = f"({corner_x},{corner_y})"
        text_x = corner_x - 100
        text_y = corner_y - 10

        if id_val == 4:
            text_y = corner_y + 30

        cv2.putText(frame_with_aruco, coord_text, (text_x, text_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

    return frame_with_aruco


def getPointAruco():
    """主函数 - 自动透视变换"""
    print("=" * 60)
    print("透视变换演示 - 每100帧输出ArUco坐标均值并写入CSV")
    print("=" * 60)
    print(f"输出尺寸: {OUTPUT_W} x {OUTPUT_H} 像素")
    print(f"实际尺寸: {REAL_W} x {REAL_H} 厘米")
    print(f"将在第 {AUTO_CALIBRATION_FRAME} 帧自动进行透视变换")
    print(f"每 {OUTPUT_INTERVAL_FRAMES} 帧输出一次ArUco坐标均值（剔除首尾10%）并更新CSV")
    print(f"CSV文件将保存为: {os.path.abspath(CSV_FILE_PATH)}")
    print("按 'q' 键退出程序")
    print("=" * 60)

    # 存储每个ArUco的坐标历史
    points_history = {}
    
    # 初始化摄像头
    if IS_REAL_SENSE:
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 1920, 1080, rs.format.bgr8, 30)
        config.enable_device(REAL_SENSE_NO)
        pipeline.start(config)
    else:
        # cap = cv2.VideoCapture(2)
        # if not cap.isOpened():
        #     print("无法打开摄像头")
        #     return
        camera = CameraHandler(camera_id=2)

    # 初始化变量
    perspective_matrix = None
    calibration_done = False
    calibration_failed = False
    frame_idx = 0
    is_warped = False

    print("\n开始采集视频...")

    # 主循环
    try:
        while True:
            frame_idx += 1

            # 读取帧
            if IS_REAL_SENSE:
                frames = pipeline.wait_for_frames()
                frame = frames.get_color_frame()
                if not frame:
                    continue
                frame = np.asanyarray(frame.get_data())
            else:
                frame = camera.get_frame_directly()
                if frame is None:
                    print("无法读取帧")
                    break

            # 在第60帧尝试自动校准
            if frame_idx == AUTO_CALIBRATION_FRAME and not calibration_done and not calibration_failed:
                print(f"\n在第 {frame_idx} 帧尝试自动校准...")
                src_points = detect_aruco_corners(frame)

                if src_points is not None:
                    perspective_matrix = cv2.getPerspectiveTransform(src_points, dst_points)
                    calibration_done = True
                    print("✓ 透视变换校准成功！")
                    # 保存校准帧
                    marked_frame = draw_aruco_markers(frame, src_points)
                    cv2.imwrite("calibration_frame.jpg", marked_frame)
                    print("校准帧已保存为: calibration_frame.jpg")
                else:
                    calibration_failed = True
                    print("✗ 校准失败：未检测到所有四个ArUco标记")

            # 应用透视变换（如果已校准）
            if calibration_done and perspective_matrix is not None:
                is_warped = True
                warped_frame = cv2.warpPerspective(frame, perspective_matrix, (OUTPUT_W, OUTPUT_H))

                # 在warped_frame中检测ArUco码并记录坐标
                aruco_info = detect_aruco_in_warped_frame(warped_frame, points_history)

                # 在warped_frame上绘制ArUco标记和关键点
                warped_frame_with_aruco = draw_aruco_on_warped_frame(warped_frame, aruco_info)
                display_frame = warped_frame_with_aruco
                
                # 每100帧输出一次坐标均值并更新CSV
                if frame_idx % OUTPUT_INTERVAL_FRAMES == 0 and frame_idx > AUTO_CALIBRATION_FRAME:
                    print_aruco_averages(points_history, frame_idx)
                    
            else:
                is_warped = False
                display_frame = frame

            # 状态文本设置
            if calibration_failed:
                status_text = "0"
                status_color = (0, 0, 255)
            elif frame_idx >= AUTO_CALIBRATION_FRAME and not calibration_done:
                status_text = "1"
                status_color = (255, 165, 0)
            else:
                status_text = f"{AUTO_CALIBRATION_FRAME}帧"
                status_color = (255, 165, 0)

            # 在画面上显示信息
            # 状态信息
            # cv2.putText(display_frame, status_text, (20, 40),
            #             cv2.FONT_HERSHEY_SIMPLEX, 0.8, status_color, 2)

            # 帧编号
            cv2.putText(display_frame, f"{frame_idx}", (20, 80),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

            # 输出尺寸信息（仅在透视变换激活时显示）
            if calibration_done:
                cv2.putText(display_frame, f" {OUTPUT_W}x{OUTPUT_H}px", (20, 120),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
                cv2.putText(display_frame, f" {REAL_W}x{REAL_H}cm", (20, 150),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

            # 显示控制说明
            cv2.putText(display_frame, "'q' exit", (20, display_frame.shape[0] - 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)

            # 如果校准失败，显示提示信息
            if calibration_failed:
                cv2.putText(display_frame, "校准失败: 未检测到所有ArUco标记",
                            (20, display_frame.shape[0] - 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

            # 显示窗口
            window_name = "Warped Frame with ArUco Detection" if calibration_done else "Original Frame"
            if is_warped:
                display_frame = cv2.resize(display_frame, (OUTPUT_W // 2, OUTPUT_H // 2))
            cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
            cv2.imshow(window_name, display_frame)
            cv2.resizeWindow(window_name, 800, 600)

            # 退出条件
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("\n用户退出程序")
                # 程序退出时输出最后一次统计结果并更新CSV
                if points_history and calibration_done:
                    print_aruco_averages(points_history, frame_idx)
                break

    except KeyboardInterrupt:
        print("\n程序被中断")
        # 程序中断时输出最后一次统计结果并更新CSV
        if points_history and calibration_done:
            print_aruco_averages(points_history, frame_idx)
    except Exception as e:
        print(f"\n程序出错: {e}")
    finally:
        # 清理资源
        print("\n清理资源...")
        cv2.destroyAllWindows()

        # 显示最终状态
        print("=" * 60)
        print("程序结束")
        if calibration_done:
            print("状态: 透视变换校准成功")
            print("结果: 输出俯视图，CSV文件已更新")
        elif calibration_failed:
            print("状态: 透视变换校准失败")
            print("原因: 未检测到所有四个ArUco标记")
        else:
            print("状态: 未进行校准")
        print("=" * 60)


if __name__ == "__main__":
    getPointAruco()
