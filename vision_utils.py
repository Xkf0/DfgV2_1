# vision_utils.py
import cv2
import numpy as np
import random
from get_points import get_points
import math
import time
# 移除 from config import *

def random_color():
    return [random.randint(50, 255) for _ in range(3)]

def get_centroid(contour):
    M = cv2.moments(contour)
    if M['m00'] == 0:
        return None
    cx = int(M['m10'] / M['m00'])
    cy = int(M['m01'] / M['m00'])
    return (cx, cy)

# 修改：增加 real_w_cm, real_h_cm 参数，移除默认值
def pixel_area_to_cm2(pixel_area, output_w, output_h, real_w_cm, real_h_cm):
    scale_x = real_w_cm / output_w
    scale_y = real_h_cm / output_h
    return pixel_area * scale_x * scale_y

# 修改：增加 real_w_cm, real_h_cm 参数
def pixel_distance_to_cm(pixel_distance, output_w, output_h, real_w_cm, real_h_cm):
    scale_x = real_w_cm / output_w
    scale_y = real_h_cm / output_h
    scale = (scale_x + scale_y) / 2
    return pixel_distance * scale

def get_short_edge_centers_and_angle(contour, mask=None):
    rect = cv2.minAreaRect(contour)
    center, (width, height), angle = rect

    if width > height:
        long_edge = width
        short_edge_angle = angle + 90
    else:
        long_edge = height
        short_edge_angle = angle

    short_edge_angle = short_edge_angle % 180
    box = cv2.boxPoints(rect)
    box = np.int32(box)

    if width > height:
        short_edge1_center = ((box[0][0] + box[1][0]) / 2, (box[0][1] + box[1][1]) / 2)
        short_edge2_center = ((box[2][0] + box[3][0]) / 2, (box[2][1] + box[3][1]) / 2)
        inward_vec = np.array([box[2][0] - box[1][0], box[2][1] - box[1][1]])
    else:
        short_edge1_center = ((box[1][0] + box[2][0]) / 2, (box[1][1] + box[2][1]) / 2)
        short_edge2_center = ((box[3][0] + box[0][0]) / 2, (box[3][1] + box[0][1]) / 2)
        inward_vec = np.array([box[1][0] - box[0][0], box[1][1] - box[0][1]])

    inward_vec = inward_vec / (np.linalg.norm(inward_vec) + 1e-6)
    shift_dist = 0.1 * long_edge
    offset = inward_vec * shift_dist

    center1 = np.array(short_edge1_center, dtype=np.float32) - offset
    center2 = np.array(short_edge2_center, dtype=np.float32) + offset

    center1 = (int(round(center1[0])), int(round(center1[1])))
    center2 = (int(round(center2[0])), int(round(center2[1])))

    if mask is not None:
        h, w = mask.shape
        def move_until_inside(pt, vec, max_iter=20):
            p = np.array(pt, dtype=np.float32)
            for _ in range(max_iter):
                x, y = int(round(p[0])), int(round(p[1]))
                if 0 <= x < w and 0 <= y < h and mask[y, x] > 0:
                    return (x, y)
                p += vec * (0.05 * long_edge)
            return (int(round(pt[0])), int(round(pt[1])))
        center1 = move_until_inside(center1, inward_vec)
        center2 = move_until_inside(center2, -inward_vec)

    return center1, center2, 90 - short_edge_angle, rect, long_edge

# 修改：增加 corner_ids_dict 参数 (接收 cfg.aruco_ids_corner)
def detect_aruco_corners(frame, detector, aruco_dict, aruco_params, corner_ids_dict):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    if detector is not None:
        corners_list, ids, rejected = detector.detectMarkers(gray)
    else:
        corners_list, ids, rejected = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
    
    id_to_corners = {}
    if ids is not None:
        ids = ids.flatten()
        for corner, id_val in zip(corners_list, ids):
            c = corner.reshape(4, 2)
            id_to_corners[id_val] = c

    try:
        # 使用传入的字典而不是全局变量
        top_left = id_to_corners[corner_ids_dict['top_left']]
        top_right = id_to_corners[corner_ids_dict['top_right']]
        bottom_right = id_to_corners[corner_ids_dict['bottom_right']]
        bottom_left = id_to_corners[corner_ids_dict['bottom_left']]

        src_pts = np.float32([
            top_left[3], top_right[2], bottom_right[1], bottom_left[0]
        ])
        return src_pts
    except KeyError:
        return None

def get_rotated_rect_crop(image, contour):
    rect = cv2.minAreaRect(contour)
    box = cv2.boxPoints(rect)
    box = np.int32(box)
    rect_width = int(rect[1][0])
    rect_height = int(rect[1][1])

    if rect_width == 0 or rect_height == 0:
        return None

    # if rect_width >= rect_height:
    #     final_width, final_height = rect_width, rect_height
    #     src_pts = box.astype("float32")
    # else:
    #     final_width, final_height = rect_height, rect_width
    #     src_pts = box.astype("float32")
    final_width, final_height = rect_width, rect_height
    src_pts = box.astype("float32")

    dst_pts = np.array([
        [0, final_height - 1], [0, 0],
        [final_width - 1, 0], [final_width - 1, final_height - 1]
    ], dtype="float32")

    M = cv2.getPerspectiveTransform(src_pts, dst_pts)
    warped = cv2.warpPerspective(image, M, (final_width, final_height))

    # if warped.shape[0] > warped.shape[1]:
    #     warped = cv2.rotate(warped, cv2.ROTATE_90_CLOCKWISE)
    return warped

def compute_affine_transform(cfg_1, ransac_thresh=3.0, confidence=0.99):
    robot_points, real_points = get_points(cfg_1.points_file_path)

    real_pts = np.asarray(real_points, dtype=np.float32)
    robot_pts = np.asarray(robot_points, dtype=np.float32)
    
    min_len = min(len(real_pts), len(robot_pts))
    real_pts = real_pts[:min_len]
    robot_pts = robot_pts[:min_len]

    if min_len < 3:
        raise ValueError("至少需要3个点来计算仿射变换")

    affine_matrix, inliers = cv2.estimateAffine2D(
        real_pts, robot_pts,
        method=cv2.RANSAC,
        ransacReprojThreshold=ransac_thresh,
        confidence=confidence,
        refineIters=10
    )
    if affine_matrix is None:
        raise RuntimeError("RANSAC 仿射估计失败")
    print(f"RANSAC 内点比例: {inliers.sum() / len(inliers):.2%}")
    return affine_matrix

def transform_point(affine_matrix, point):
    x, y = point
    vec = np.array([x, y, 1])
    x_robot, y_robot = affine_matrix @ vec
    return x_robot, y_robot

def correct_color(frame):
    lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)
    avg_a, avg_b = np.mean(a), np.mean(b)
    a = a - (avg_a - 128) * (l / 255.0) * 1.1
    b = b - (avg_b - 128) * (l / 255.0) * 1.1
    lab = cv2.merge([l, a, b])
    balanced = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)
    hsv = cv2.cvtColor(balanced, cv2.COLOR_BGR2HSV).astype(np.float32)
    hsv[..., 1] *= 1.1
    hsv[..., 2] *= 1.05
    hsv = np.clip(hsv, 0, 255).astype(np.uint8)
    return cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)

def collisionDetect1(length_half_lead_screw_cm, cfg, motion_dict, center, mid, length_half_conveyor_belt_cm):
    delta_y_arm1 = length_half_lead_screw_cm * cfg.ratio_wh * math.sin(math.radians(motion_dict[mid]['angle'])) # 51:半个丝杆长度，单位cm
    vision_y_far_edge =  center[1] - abs(delta_y_arm1) 
    if vision_y_far_edge < length_half_conveyor_belt_cm * cfg.ratio_wh: # 46：传送带一半长度，单位cm
        return True
    else:
        return False
    
def collisionDetect2(length_half_lead_screw_cm, cfg, motion_dict, center, mid, length_half_conveyor_belt_cm):
    delta_y_arm1 = length_half_lead_screw_cm * cfg.ratio_wh * math.sin(math.radians(motion_dict[mid]['angle'])) # 51:半个丝杆长度，单位cm
    vision_y_far_edge =  center[1] + abs(delta_y_arm1) 
    if vision_y_far_edge > length_half_conveyor_belt_cm * cfg.ratio_wh: # 46：传送带一半长度，单位cm
        return True
    else:
        return False
    
def get_mapped_robot_x_y(AFFINE_MATRIX, center, DIRECTION, get_speed_now, get_time_pre_now):
    mapped_robot_x, mapped_robot_y = transform_point(AFFINE_MATRIX, (center[0] , center[1]))
    mapped_robot_x = mapped_robot_x + get_speed_now * get_time_pre_now * 10 * DIRECTION[0]
    mapped_robot_y = mapped_robot_y + get_speed_now * get_time_pre_now * 10 * DIRECTION[1] 
    return mapped_robot_x, mapped_robot_y

def get_target_robot_coord(cfg, rz_use, AFFINE_MATRIX, center, DIRECTION, get_speed_now, get_time_pre_now):
    mapped_robot_x, mapped_robot_y = get_mapped_robot_x_y(AFFINE_MATRIX, center, DIRECTION, get_speed_now, get_time_pre_now)
    target_safe_pos = cfg.safe_pos_init.copy() 
    target_safe_pos[0] = mapped_robot_x
    target_safe_pos[1] = mapped_robot_y
    target_safe_pos[5] = rz_use
    robot_coord = [mapped_robot_x, mapped_robot_y, cfg.stand_z, cfg.stand_rx, cfg.stand_ry, rz_use]
    return robot_coord, target_safe_pos

def ChangeTaskDict1(cfg_1, motion_dict, mid, openCollisionDetect, length_lead_screw_cm, center, to_del, AFFINE_MATRIX_1, DIRECTION_1, get_speed_now, get_time_pre_now, task_lock_1, task_queue_1):
    rz_use = cfg_1.stand_rz + motion_dict[mid]['angle']
    if rz_use > 180: rz_use -= 360
    if rz_use < -180: rz_use += 360

    if openCollisionDetect:
        if collisionDetect1(length_lead_screw_cm / 2, cfg_1, motion_dict, center, mid, cfg_1.real_h / 2):
            print("已越界, 会发生碰撞, 跳过1号臂抓取任务")
            to_del.append(mid)
            return False
                        
    robot_coord, target_safe_pos = get_target_robot_coord(cfg_1, rz_use, AFFINE_MATRIX_1, center, DIRECTION_1, get_speed_now, get_time_pre_now)
                            
    # 分配给1号机械臂，使用1号的初始安全位
    with task_lock_1:
        task_queue_1.put((motion_dict, mid, robot_coord, target_safe_pos)) # robot_coord（跟随运动出发位）和target_safe_pos（传送带静止拦截位）只有rx、ry不同

    print(f"任务id-{mid} 分配至1号机械臂 ")

    motion_dict[mid]['status'] = 3
    return True

def ChangeTaskDict2(cfg_2, motion_dict, mid, openCollisionDetect, length_lead_screw_cm, center, to_del, AFFINE_MATRIX_2, DIRECTION_2, get_speed_now, get_time_pre_now, task_lock_2, task_queue_2):
    rz_use = cfg_2.stand_rz - motion_dict[mid]['angle']
    if rz_use > 180: rz_use -= 360
    if rz_use < -180: rz_use += 360

    if openCollisionDetect:
        if collisionDetect2(length_lead_screw_cm / 2, cfg_2, motion_dict, center, mid, cfg_2.real_h / 2):
            print("已越界, 会发生碰撞, 跳过2号臂抓取任务")
            to_del.append(mid)
            return False

    robot_coord, target_safe_pos = get_target_robot_coord(cfg_2, rz_use, AFFINE_MATRIX_2, center, DIRECTION_2, get_speed_now, get_time_pre_now)
    
    # 分配给2号机械臂，使用2号的初始安全位
    with task_lock_2:
        task_queue_2.put((motion_dict, mid, robot_coord, target_safe_pos))

    print(f"任务id-{mid} 分配至2号机械臂")

    motion_dict[mid]['status'] = 3
    return True

def ChangeTaskDictAll(edge_1_in, edge_1_out, edge_2_in, edge_2_out, sum_detect, cfg_1, cfg_2, motion_dict, mid, openCollisionDetect, length_lead_screw_cm, center, to_del, AFFINE_MATRIX_1, AFFINE_MATRIX_2, DIRECTION_1, DIRECTION_2, get_speed_now, get_time_pre_now, task_lock_1, task_lock_2, task_queue_1, task_queue_2):
    if edge_1_in <= center[1]<=edge_1_out: # G 当前（左侧机械臂）
        if ChangeTaskDict1(cfg_1, motion_dict, mid, openCollisionDetect, length_lead_screw_cm, center, to_del, AFFINE_MATRIX_1, DIRECTION_1, get_speed_now, get_time_pre_now, task_lock_1, task_queue_1):
            sum_detect[0]+=1
            print(f"id-{mid} 加入抓取队列")
            print("总计加入队列数量：",sum_detect[0])
        else:
            return False
    elif edge_2_in<= center[1]<edge_2_out : # G 新增（右侧机械臂）
        if ChangeTaskDict2(cfg_2, motion_dict, mid, openCollisionDetect, length_lead_screw_cm, center, to_del, AFFINE_MATRIX_2, DIRECTION_2, get_speed_now, get_time_pre_now, task_lock_2, task_queue_2):
            sum_detect[0]+=1
            print(f"id-{mid} 加入抓取队列")
            print("总计加入队列数量：",sum_detect[0])
        else:
            return False
    else:
        motion_dict[mid]['status'] = 3
        # be_ignored+=1
        print("超出传送带范围")
    return True

def cleanTimeOutTrack(mid, tracked_objects, motion_dict, to_del):
    if mid not in tracked_objects:
        if time.perf_counter() - motion_dict[mid].get('last_seen', time.perf_counter()) > 60:
            to_del.append(mid)
        motion_dict[mid]['last_seen'] = time.perf_counter()

def cycleTaskHandle(tracked_objects, edge_1_in, edge_1_out, edge_2_in, edge_2_out, sum_detect, cfg_1, cfg_2, motion_dict, openCollisionDetect, length_lead_screw_cm, to_del, AFFINE_MATRIX_1, AFFINE_MATRIX_2, DIRECTION_1, DIRECTION_2, get_speed_now, get_time_pre_now, task_lock_1, task_lock_2, task_queue_1, task_queue_2):
    for mid in motion_dict:
        status = motion_dict[mid]['status']
        if (status == 2 
        and mid not in [t[1] for t in list(task_queue_1.queue) if t[1] is not None] 
        and mid not in [t[1] for t in list(task_queue_2.queue) if t[1] is not None]): # G
            center = motion_dict[mid]['motion_center']
            
            print(f"============{mid}=============")
            # G根据双臂进行修改
            if ChangeTaskDictAll(edge_1_in, edge_1_out, edge_2_in, edge_2_out, sum_detect, cfg_1, cfg_2, motion_dict, mid, openCollisionDetect, length_lead_screw_cm, center, to_del, AFFINE_MATRIX_1, AFFINE_MATRIX_2, DIRECTION_1, DIRECTION_2, get_speed_now, get_time_pre_now, task_lock_1, task_lock_2, task_queue_1, task_queue_2) is False:
                continue

        cleanTimeOutTrack(mid, tracked_objects, motion_dict, to_del)

    for mid in to_del:
        motion_dict.pop(mid, None)