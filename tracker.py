# tracker.py
import numpy as np
import cv2
from collections import deque
# 移除 from config import *
# 引用路径根据你的实际项目结构调整，这里假设 vision_utils 同级或已正确导入
import vision_utils  # 整体导入

from logger import LOG_DEBUG, LOG_INFO, LOG_WARN, LOG_ERROR, LOG_CRITICAL
import os

class StableGrabCenter:
    def __init__(self, alpha_center=0.85, alpha_angle=0.75, max_history=15):
        self.alpha_center = alpha_center
        self.alpha_angle = alpha_angle
        self.max_history = max_history
        self.stable_center = None
        self.stable_angle = None
        self.center_history = deque(maxlen=max_history)
        self.angle_history = deque(maxlen=max_history)
        self.MAX_DEVIATION_PX = 40
        

    def _robust_pca_direction(self, mask, centroid):
        points = np.column_stack(np.where(mask > 0))
        if len(points) < 10:
            return np.array([1, 0]), np.array(centroid), 0.0
        points = points[:, ::-1] # (y,x) -> (x,y)
        centroid_np = np.array(centroid)
        try:
            centered = points - centroid_np
            cov = np.cov(centered.T)
            eigvals, eigvecs = np.linalg.eigh(cov)
            idx = np.argmax(eigvals)
            direction = eigvecs[:, idx]
            if np.linalg.norm(direction) < 1e-6:
                raise ValueError("Zero direction vector")
            direction = direction / np.linalg.norm(direction)
            
            projs = centered @ direction
            p1 = centroid_np + direction * projs.min()
            p2 = centroid_np + direction * projs.max()
            pca_center = (p1 + p2) * 0.5
            angle = np.arctan2(direction[1], direction[0]) * 180 / np.pi
            return direction, pca_center, angle
        except Exception:
            return np.array([1, 0]), centroid_np, 0.0

    def get_stable_grab_center_from_mask(self, mask, centroid, motion_front_center=None, extension_ratio=0.45):
        centroid = np.array(centroid, dtype=float)
        direction, pca_center, raw_angle = self._robust_pca_direction(mask, centroid)

        if motion_front_center is not None:
            vec_to_front = np.array(motion_front_center) - centroid
            if np.linalg.norm(vec_to_front) > 10:
                front_dir = vec_to_front / np.linalg.norm(vec_to_front)
                if np.dot(direction, front_dir) < 0:
                    direction = -direction
                    raw_angle += 180
        
        raw_angle %= 360
        vec_from_centroid = pca_center - centroid
        extend_length = np.linalg.norm(vec_from_centroid) * extension_ratio
        candidate_center = centroid + direction * extend_length

        if np.linalg.norm(candidate_center - centroid) > self.MAX_DEVIATION_PX:
            candidate_center = centroid.copy()
            raw_angle = self.stable_angle if self.stable_angle is not None else raw_angle

        if self.stable_center is None:
            self.stable_center = candidate_center.copy()
            self.stable_angle = raw_angle
        else:
            self.stable_center = self.alpha_center * candidate_center + (1 - self.alpha_center) * self.stable_center
            self.stable_angle = (self.alpha_angle * raw_angle + (1 - self.alpha_angle) * self.stable_angle) % 360
        
        self.center_history.append(self.stable_center.copy())
        self.angle_history.append(self.stable_angle)
        return tuple(self.stable_center.astype(int)), self.stable_angle

# 修改：接收 cfg 对象
def calculate_speed(position_history, time_history, current_time, cfg):
    if len(position_history) < 2:
        return 0.0
    speeds = []
    # 使用 cfg.speed_smooth_frames
    for i in range(1, min(cfg.speed_smooth_frames + 1, len(position_history))):
        if i >= len(position_history): break
        pos1, pos2 = position_history[-i - 1], position_history[-i]
        t1, t2 = time_history[-i - 1], time_history[-i]
        if t2 - t1 > 0:
            pixel_dist = np.linalg.norm(np.array(pos2) - np.array(pos1))
            # 传递 cfg 中的参数
            cm_dist = vision_utils.pixel_distance_to_cm(pixel_dist, cfg.output_w, cfg.output_h, cfg.real_w, cfg.real_h)
            speeds.append(cm_dist / (t2 - t1))
    return np.mean(speeds) if speeds else 0.0

# 修改：接收 cfg 对象
def update_motion_status(img_filename, longedge, single_mask, motion_dict, object_id, centroid, motion_front_center, current_time, angle, long_side_length, affine_matrix, cfg, permitGrasp, isStatic):
    line1_x = cfg.output_w * 2 // 5
    line2_x = cfg.output_w * 1 // 2
    x_pos = motion_front_center[0]

    if object_id not in motion_dict:
        LOG_INFO("motion dict don't have %did", object_id)
        motion_dict[object_id] = {
            'centroid': centroid,
            'motion_center': motion_front_center,
            'status': 0,
            'time_use': None, 'time': None, 'time_pre': None,
            'line1_time': None, 'line2_time': None,
            'line1_pos': None, 'line2_pos': None,
            'angle': angle, 'speed': None, 'direction': None,
            'long_side_length': None
        }
    
    obj_info = motion_dict[object_id]

    from datetime import datetime
    foldname = f"/home/xf/imgs/{img_filename}/{object_id}"
    os.makedirs(foldname, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S.%f")[:-3]
    long_side_length = round(long_side_length, 3)
    longedge = round(longedge, 3)
    filename = f"{foldname}/{timestamp}_status{obj_info['status']}_{long_side_length}_{longedge}.png"
    cv2.imwrite(filename, single_mask)

    if obj_info['status'] <= 1:
        obj_info['motion_center'] = motion_front_center
        obj_info['angle'] = angle

    if obj_info['status'] == 0 and x_pos <= line2_x - 100:
        obj_info['status'] = 1
        obj_info['line1_time'] = current_time
        obj_info['line1_pos'] = centroid
        LOG_INFO("对象 %d: 越过1/2基线前面5cm的位置, 状态更新为1", object_id)
        print(f"面积: {np.count_nonzero(single_mask)/400} cm2, id: {object_id}, 长度: {longedge/20}")
        LOG_INFO("面积: %fcm2, id: %d, 长度: %f", np.count_nonzero(single_mask)/400, object_id, longedge/20)

    # elif obj_info['status'] == 1 and x_pos >= line2_x and permitGrasp[object_id] is True:
    elif obj_info['status'] == 1 and x_pos >= line2_x and isStatic[object_id] is True:
        LOG_INFO("对象 %d: 越过1/2基线，状态更新为2", object_id)
        print(f"================2面积: {np.count_nonzero(single_mask)/400} cm2, id: {object_id}, 长度: {longedge/20}")
        LOG_INFO("================2面积: %fcm2, id: %d, 长度: %f", np.count_nonzero(single_mask)/400, object_id, longedge/20)
        obj_info['status'] = 2
        obj_info['line2_time'] = current_time
        obj_info['line2_pos'] = centroid
        obj_info['time'] = current_time
        obj_info['long_side_length'] = long_side_length
        
        if obj_info['line1_time'] is not None and obj_info['line1_pos'] is not None:
            obj_info['time_use'] = current_time - obj_info['line1_time']
            obj_info['time_pre'] = 16 # 此处原逻辑为固定值，如果 cfg 中有配置也可替换
            
            # 使用 cfg 参数
            scale_x = cfg.real_w / cfg.output_w
            scale_y = cfg.real_h / cfg.output_h
            
            point1_real = np.array([obj_info['line1_pos'][0] * scale_x * 3, obj_info['line1_pos'][1] * scale_y])
            point2_real = np.array([obj_info['line2_pos'][0] * scale_x * 3, obj_info['line2_pos'][1] * scale_y])
            
            vec1 = np.array([point1_real[0], point1_real[1], 1])
            vec2 = np.array([point2_real[0], point2_real[1], 1])
            
            robot_point1 = affine_matrix @ vec1
            robot_point2 = affine_matrix @ vec2
            direction_robot = robot_point2 - robot_point1
            
            norm = np.linalg.norm(direction_robot)
            if norm > 1e-6:
                obj_info['direction'] = direction_robot / norm
                print(f"对象 {object_id} 状态2确认. Robot Dir: {obj_info['direction']}")

# 修改：接收 cfg 对象
def draw_motion_info(image, motion_dict, tracked_objects, cfg):
    line1_x = cfg.output_w // 3
    line2_x = cfg.output_w * 1 // 2
    cv2.line(image, (line2_x, 0), (line2_x, cfg.output_h), (255, 255, 0), 10)
    cv2.putText(image, "1/3", (line1_x + 5, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
    cv2.putText(image, "1/2", (line2_x + 5, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

    for object_id in motion_dict:
        if object_id in tracked_objects:
            obj_info = motion_dict[object_id]
            center = obj_info['motion_center']
            cv2.putText(image, f"S:{obj_info['status']}", (center[0] + 15, center[1] - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    status_counts = {0: 0, 1: 0, 2: 0, 3: 0}
    for obj_info in motion_dict.values():
        if obj_info['status'] in status_counts:
            status_counts[obj_info['status']] += 1
    
    cv2.putText(image, f"Motion Status - S0:{status_counts[0]} S1:{status_counts[1]} S2:{status_counts[2]}",
                (10, 140), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)