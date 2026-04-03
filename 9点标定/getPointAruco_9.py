import cv2
import numpy as np
import pyrealsense2 as rs

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
# TODO
ARUCO_IDS_CORNER = {
    'top_left': 0,
    'top_right': 8,
    'bottom_right': 24,
    'bottom_left': 16
}

# 输出矩形尺寸（像素）
# TODO
REAL_W, REAL_H = 64, 92.05   # cm
RATIO_WH = 20

H_min = 10000
H_max = 0

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
IS_REAL_SENSE = True

# TODO
REAL_SENSE_NO = '211622062803'

AUTO_CALIBRATION_FRAME = 60  # 在第60帧自动校准


# -------------------------------
# 工具函数
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
        bottom_right_corners = id_to_corners[ARUCO_IDS_CORNER['bottom_right']]  # ID: 16
        bottom_left_corners = id_to_corners[ARUCO_IDS_CORNER['bottom_left']]  # ID: 24

        # TODO 确定使用四个透视变换码的哪个角点
        src_pts = np.float32([
            top_left_corners[2],  # 左上ArUco的右下角点     2
            top_right_corners[3],  # 右上ArUco的左下角点    3
            bottom_right_corners[0],  # 右下ArUco的左上角点    0
            bottom_left_corners[1]  # 左下ArUco的右上角点    1
        ])
        return src_pts
    except KeyError as e:
        print(f"提取角点时出错: {e}")
        return None


def detect_aruco_in_warped_frame(warped_frame, points):
    """在warped_frame中检测所有ArUco码，返回角点信息"""

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

            # ArUco角点顺序：[左上, 右上, 右下, 左下]
            # 索引 0: 左上角, 1: 右上角, 2: 右下角, 3: 左下角

            # 对于ID为4的标记，使用右上角点（索引1），其他使用右下角点（索引2）


            # selected_corner = corners[2]  # 右下角点
            corner_name = "中心"

            # 计算ArUco中心点
            center_x = int(np.mean(corners[:, 0]))
            center_y = int(np.mean(corners[:, 1]))

            selected_corner = (center_x, center_y)

            center_x_fp = np.mean(corners[:, 0])
            center_y_fp = np.mean(corners[:, 1])


            aruco_info.append({
                'id': ids[i],
                'corners': corners,
                'selected_corner': selected_corner,  # 选择的关键点
                'corner_name': corner_name,  # 关键点名称
                'center': (center_x, center_y)
            })

            if ids[i] not in points:
                points[ids[i]] = {}
                points[ids[i]]['max_x'] = center_x_fp
                points[ids[i]]['max_y'] = center_y_fp
                points[ids[i]]['min_x'] = center_x_fp
                points[ids[i]]['min_y'] = center_y_fp
            else:
                if center_x_fp > points[ids[i]]['max_x']:
                    points[ids[i]]['max_x'] = center_x_fp
                if center_x_fp < points[ids[i]]['min_x']:
                    points[ids[i]]['min_x'] = center_x_fp
                if center_y_fp > points[ids[i]]['max_y']:
                    points[ids[i]]['max_y'] = center_y_fp
                if center_y_fp < points[ids[i]]['min_y']:
                    points[ids[i]]['min_y'] = center_y_fp


            # if ids[i] == 80:
            if True:
                # 打印坐标信息
                print(f"ArUco ID {ids[i]}: {corner_name}坐标 ({center_x_fp}, {center_y_fp})")
                print(f"x_max-x_min:{points[ids[i]]['max_x'] - points[ids[i]]['min_x']}, y_max-y_min:{points[ids[i]]['max_y'] - points[ids[i]]['min_y']}")
                global H_min, H_max
                if center_y_fp < H_min:
                    H_min = center_y_fp
                if center_y_fp > H_max:
                    H_max = center_y_fp

                print(f"H_min:{H_min},H_max:{H_max}")



    return aruco_info


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

        # 2. 绘制关键点（ID4为蓝色，其他为红色）
        corner_x, corner_y = int(selected_corner[0]), int(selected_corner[1])
        color = (0, 0, 255)  # 红色

        cv2.circle(frame_with_aruco, (corner_x, corner_y), 8, color, -1)
        cv2.circle(frame_with_aruco, (corner_x, corner_y), 10, (255, 255, 255), 2)

        # 3. 显示ID和坐标
        # 在ArUco中心显示ID
        cv2.putText(frame_with_aruco, f"ID:{id_val}", (center[0] - 20, center[1]),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        # 在关键点附近显示坐标和角点类型
        coord_text = f"({corner_x},{corner_y})"
        text_x = corner_x - 100
        text_y = corner_y - 10

        # 根据ID调整文本位置
        if id_val == 4:
            text_y = corner_y + 30  # 向下移动一点，避免重叠

        cv2.putText(frame_with_aruco, coord_text, (text_x, text_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

    return frame_with_aruco


def getPointAruco():
    """主函数 - 自动透视变换"""
    print("=" * 60)
    print("透视变换演示")
    print("=" * 60)
    print(f"输出尺寸: {OUTPUT_W} x {OUTPUT_H} 像素")
    print(f"实际尺寸: {REAL_W} x {REAL_H} 厘米")
    print(f"将在第 {AUTO_CALIBRATION_FRAME} 帧自动进行透视变换")
    print("按 'q' 键退出程序")
    print("=" * 60)


    points = {}
    # 'max_x' 'max_y' 'min_x' 'min_y'

    # 初始化摄像头
    # TODO   读帧初始化
    if IS_REAL_SENSE:
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 1920, 1080, rs.format.bgr8, 30)
        config.enable_device(REAL_SENSE_NO)
        pipeline.start(config)
    else:
        cap = cv2.VideoCapture(3)
        if not cap.isOpened():
            print("无法打开摄像头")
            return

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
            # TODO   读帧
            if IS_REAL_SENSE:
                frames = pipeline.wait_for_frames()
                frame = frames.get_color_frame()
                if not frame:
                    continue
                frame = np.asanyarray(frame.get_data())
            else:
                ret, frame = cap.read()
                if not ret:
                    print("无法读取帧")
                    break

            # 在第60帧尝试自动校准
            if frame_idx == AUTO_CALIBRATION_FRAME and not calibration_done and not calibration_failed:
                print(f"\n在第 {frame_idx} 帧尝试自动校准...")
                calibration_frame = frame.copy()
                src_points = detect_aruco_corners(frame)

                if src_points is not None:
                    perspective_matrix = cv2.getPerspectiveTransform(src_points, dst_points)
                    calibration_done = True
                    print("✓ 透视变换校准成功！")
                    print(f"使用的ArUco角点:")
                    print(f"  左上ArUco(ID:{ARUCO_IDS_CORNER['top_left']})的右下角点: {src_points[0]}")
                    print(f"  右上ArUco(ID:{ARUCO_IDS_CORNER['top_right']})的左下角点: {src_points[1]}")
                    print(f"  右下ArUco(ID:{ARUCO_IDS_CORNER['bottom_right']})的左上角点: {src_points[2]}")
                    print(f"  左下ArUco(ID:{ARUCO_IDS_CORNER['bottom_left']})的右上角点: {src_points[3]}")

                    # 保存校准帧
                    if src_points is not None:
                        marked_frame = draw_aruco_markers(frame, src_points)
                        cv2.imwrite("calibration_frame.jpg", marked_frame)
                        print("校准帧已保存为: calibration_frame_frame.jpg")
                else:
                    calibration_failed = True
                    print("✗ 校准失败：未检测到所有四个ArUco标记")

            # 应用透视变换（如果已校准）
            if calibration_done and perspective_matrix is not None:
                is_warped = True
                warped_frame = cv2.warpPerspective(frame, perspective_matrix, (OUTPUT_W, OUTPUT_H))

                # 在warped_frame中检测ArUco码
                print(f"\n--- 帧 {frame_idx} ---")
                aruco_info = detect_aruco_in_warped_frame(warped_frame, points)

                # 在warped_frame上绘制ArUco标记和关键点
                warped_frame_with_aruco = draw_aruco_on_warped_frame(warped_frame, aruco_info)

                display_frame = warped_frame_with_aruco
                status_text = "-1"
                status_color = (0, 255, 0)
            else:
                is_warped = False
                display_frame = frame
                if calibration_failed:
                    status_text = "0"
                    status_color = (0, 0, 255)
                elif frame_idx >= AUTO_CALIBRATION_FRAME:
                    status_text = "1"
                    status_color = (255, 165, 0)
                else:
                    status_text = f"{AUTO_CALIBRATION_FRAME}帧"
                    status_color = (255, 165, 0)

            # 在画面上显示信息
            # 状态信息
            cv2.putText(display_frame, status_text, (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, status_color, 2)

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
                # display_frame = display_frame[OUTPUT_H // 2:, :, :]
                display_frame = cv2.resize(display_frame, (OUTPUT_W // 2, OUTPUT_H // 2))
            cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
            cv2.imshow(window_name, display_frame)
            cv2.resizeWindow(window_name, 800, 600)

            # 退出条件
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("\n用户退出程序")
                break

    except KeyboardInterrupt:
        print("\n程序被中断")
    except Exception as e:
        print(f"\n程序出错: {e}")
    finally:
        # 清理资源
        print("\n清理资源...")
        if IS_REAL_SENSE:
            pipeline.stop()
        else:
            cap.release()
        cv2.destroyAllWindows()

        # 显示最终状态
        print("=" * 60)
        print("程序结束")
        if calibration_done:
            print("状态: 透视变换校准成功")
            print("结果: 输出俯视图")
        elif calibration_failed:
            print("状态: 透视变换校准失败")
            print("原因: 未检测到所有四个ArUco标记")
        else:
            print("状态: 未进行校准")
        print("=" * 60)


if __name__ == "__main__":
    getPointAruco()