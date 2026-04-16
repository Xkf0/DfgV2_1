# lerobot/iros/DfgV1_1/object_detector.py

import cv2
import numpy as np
import time
import math
from collections import deque
import vision_utils as vu
# from reid_module import FeatureExtractor, filter_objects_by_similarity
from tracker import StableGrabCenter, calculate_speed, update_motion_status, draw_motion_info

from sam2_use.SAM2_motion_detector import SAM2MotionDetector
import os
from global_state import AppState
from logger import LOG_DEBUG, LOG_INFO, LOG_WARN, LOG_ERROR, LOG_CRITICAL
from queue import Queue
import time
import matplotlib.pyplot as plt
from scipy import ndimage
area_ratio =1


class ObjectDetector:
    """
    将视觉处理逻辑封装成一个类。
    """

    def __init__(self, config, is_use_sam=False):
        """
        初始化对象检测器。
        :param config: Configer 实例，包含所有配置参数。
        """
        self.cfg = config

        self.is_use_sam = is_use_sam

        self.detector_sam2 = None
        if self.is_use_sam :
            self.detector_sam2 = SAM2MotionDetector(
                model_cfg="configs/sam2.1/sam2.1_hiera_t.yaml",
                checkpoint="sam2_use/models/sam2.1_hiera_tiny.pt"
            )

        self.background = None
        self.perspective_matrix = None
        self.is_background_use = False
        
        # ArUco 检测器
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        # 状态跟踪变量
        self.tracked_objects = {}
        self.motion_dict = {}
        self.pixel_area_dict = []
        self.area_dict = []
        self.area_grasp = {}
        self.grab_calculators = {}
        self.grab_history = {}
        self.next_id = 0
        self.frame_idx = 0
        self.last_speed_update_time = time.perf_counter()
        self.last_area_update_time = time.perf_counter()

        if AppState.CONFIG["testStaticFabricLength"] or AppState.CONFIG["testDynamicFabricLength"]:        
            self.x_max = 0
            self.y_max = 0
            self.x_min = 1000000
            self.y_min = 1000000
            self.t_max = -1000000
            self.t_min = 1000000
            self.last_print_time = time.time()
            self.interval = 0.5

        from datetime import datetime
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S.%f")[:-3]
        self.img_filename = timestamp
        foldname = f"/home/xf/imgs/{timestamp}"
        os.makedirs(foldname, exist_ok=True)
        self.centroidLastestFive = []
        self.lastAreaAvg = 0.0
        self.id_now = []
        self.lastPrintTime = time.time()
        self.printLog = False
        self.saveLastFrame = 5 # 用于漂移保留
        self.nowAreaAvg = []
        self.warnId = []
        self.templateArea = 0.0
        self.permitGrasp = []
        self.isStatic = []
        self.frameTh = 0
        self.firstFrame = np.load('frame_data.npy')
        self.threshold_value = 25
        self.start_split = False
        self.numCentroids = 5

        # ReID 特征提取器
        # self.feature_extractor = FeatureExtractor()
    
    def frame_to_black(self, warped_frame, balck_rate=0.04):
        h, w, _ = warped_frame.shape

        # 计算上下10%高度
        top_height = int(balck_rate * h)
        bottom_height = int(balck_rate * h)

        # 用黑色覆盖上下区域
        warped_frame[0:top_height, :] = 0  # 上10%
        warped_frame[h - bottom_height:h, :] = 0  # 下10%

        # warped_frame[:, 0:150] = 0  # 上10%
        # warped_frame[:, 1600:w] = 0  # 下10%

        return warped_frame


    def initialize_background(self, frame):
        """
        在第30帧初始化背景和透视变换矩阵。
        :param frame: 原始输入帧。
        :return: bool, 是否成功初始化。
        """
        if not self.is_background_use and self.frame_idx >= 30:
            src_points_new = vu.detect_aruco_corners(frame, self.detector, self.aruco_dict, self.aruco_params, self.cfg.aruco_ids_corner)


            if src_points_new is not None:
                self.perspective_matrix = cv2.getPerspectiveTransform(src_points_new, self.cfg.dst_points)
                warped_frame = cv2.warpPerspective(frame, self.perspective_matrix, (self.cfg.output_w, self.cfg.output_h))
                warped_frame = self.frame_to_black(warped_frame)
                self.background = warped_frame.copy()
                self.is_background_use = True
                return True
        return False

    def update_area_statistics(self, tid: int):
        """
        更新某个 tid 的面积均值统计。
        """
        if tid >= len(self.area_dict):
            return
        if tid >= len(self.nowAreaAvg):
            return

        if len(self.area_dict[tid]) == 5:
            self.area_dict[tid].sort()
            self.nowAreaAvg[tid] = self.area_dict[tid][2]  # 中值

        elif len(self.area_dict[tid]) > 5:
            self.nowAreaAvg[tid] = (
                self.nowAreaAvg[tid] * (len(self.area_dict[tid]) - 5)
                + self.area_dict[tid][-1]
            ) / (len(self.area_dict[tid]) - 4)


        # @staticmethod
    def is_abnormal_cloth( self ,   info: dict):
        """
        异常布料判定接口

        视觉侧可写入：
        - jam_flag=True               -> 卡料
        - curl_flag=True              -> 卷曲
        - abnormal_type="jam"/"curl" -> 异常类型
        """
        if info.get("jam_flag", False) is True:
            return True, "卡料"

        if info.get("curl_flag", False) is True:
            return True, "卷曲"

        abnormal_type = str(info.get("abnormal_type", "")).strip().lower()
        if abnormal_type in ("jam", "stuck", "block"):
            return True, "卡料"
        if abnormal_type in ("curl", "curled", "wrinkle"):
            return True, "卷曲"

        return False, ""


    def build_abnormal_info_for_tid(self , tid :int )->dict :
        global area_ratio
        abnormal_info = {
                            "jam_flag": False ,
                            "curl_flag" : False ,
                            "tid":tid ,
                            "area_ratio" :area_ratio,
                            "template_area" : self.templateArea,
                            "current_area" :self .nowAreaAvg[tid],
                            "reason" : "卷曲或者换布料", 
                        }
        if (
            tid <len (self.area_dict)
            and tid < len (self.nowAreaAvg)
            and len (self.area_dict[tid])>20
            and self.templateArea not in (0 ,None)
        ):
            if self.templateArea in (0,None):
                return abnormal_info
            if self.nowAreaAvg[tid] in (-1.0 , None):
                 return abnormal_info
            
        if self.templateArea == 0:
            return abnormal_info
        area_ratio = self.nowAreaAvg[tid] / self.templateArea
        abnormal_info["area_ratio"] = area_ratio
        abnormal_info["template_area"] = self.templateArea
        abnormal_info["current_area"] = self.nowAreaAvg[tid]
        if area_ratio < 0.97:
            abnormal_info["curl_flag"] = True
            abnormal_info["abnormal_type"] = "curl"
            abnormal_info["reason"] = f"面积比例过小，疑似卷曲或换布，ratio={area_ratio:.4f}"
        return abnormal_info
        #    面积更新计算
    
   
    def apply_abnormal_result(self, tid: int, track_info: dict):
        """
        对单个 tid 执行异常检测，并把结果写回 track_info。
        """
        abnormal_info = self.build_abnormal_info_for_tid(tid)
        abnormal, abnormal_name = self.is_abnormal_cloth(abnormal_info)

        track_info["abnormal"] = abnormal
        track_info["abnormal_name"] = abnormal_name
        track_info["abnormal_info"] = abnormal_info

        if abnormal:
            if tid < len(self.warnId):
                self.warnId[tid] = True
            if tid < len(self.permitGrasp):
                self.permitGrasp[tid] = False

            LOG_WARN(
                "异常检测报警: tid=%d, type=%s, template_area=%s, current_area=%s, area_ratio=%s, reason=%s",
                tid,
                abnormal_name,
                abnormal_info.get("template_area"),
                abnormal_info.get("current_area"),
                abnormal_info.get("area_ratio"),
                abnormal_info.get("reason"),
            )

    def split_draw(self, frame, MOG2points, last_centroid):
        """主处理函数"""
        if self.firstFrame is None:
            print("❌ 请先设置模板帧")
            return None, None
            
        # --- 1. 差分计算 ---
        frame_f = frame.astype(np.float32)
        diff = np.abs(frame_f - self.firstFrame)
        diff_gray = np.max(diff, axis=2)
        
        # --- 2. 统计diff信息---
        vu.debug_diff(diff_gray)
        
        # --- 3. 去噪与阈值（可调）---
        diff_blur = cv2.GaussianBlur(diff_gray, (11, 11), 0)
        _, mask = cv2.threshold(diff_blur, self.threshold_value, 255, cv2.THRESH_BINARY)
        mask = (mask > 0).astype(np.uint8)
        
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        # --- 4. 质心计算（过滤小区域）---
        labels, num = ndimage.label(mask)

        centroids = []

        for i in range(1, num + 1):
            # 提取单个连通区域
            component_mask = (labels == i).astype(np.uint8) * 255
            # 找轮廓
            contours, _ = cv2.findContours(
                component_mask, 
                cv2.RETR_EXTERNAL, 
                cv2.CHAIN_APPROX_SIMPLE
            )
            if not contours:
                continue
            # 取最大轮廓（通常只有一个）
            cnt = max(contours, key=cv2.contourArea)
            # 计算凸包
            hull = cv2.convexHull(cnt)
            # ✅ 凸包面积（这才是“内接凸区域面积”）
            hull_area = cv2.contourArea(hull)
            if hull_area >= 20000:
                # 用原始 mask 计算质心（更准确）
                cy, cx = ndimage.center_of_mass(mask, labels, i)
                centroids.append((int(cx), int(cy), hull_area))

        # centroids 现在是过滤后的结果
        
        # --- 5. 可视化 ---
        display_img = self._draw_display(frame, diff_blur, mask, centroids, MOG2points, last_centroid)
        
        return centroids, display_img
    
    def _draw_display(self, current_frame, diff_blur, mask, centroids, MOG2points_ori, last_centroid_ori):
        """绘制：左=模板图，右=热力图，质心在右"""
        h, w = self.firstFrame.shape[:2]
        
        # === 左侧：静态模板图 ===
        left_panel = self.firstFrame.astype(np.uint8).copy()
        cv2.putText(left_panel, "Template Frame", (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        # === 右侧：热力图（不使用归一化）===
        # 将diff_blur转换为uint8（注意：可能超出0-255范围，需要截断）
        diff_uint8 = np.clip(diff_blur, 0, 255).astype(np.uint8)
        
        # 应用颜色映射
        heatmap_color = cv2.applyColorMap(diff_uint8, cv2.COLORMAP_JET)
        
        # 将阈值以下的部分设为白色
        # 创建掩码：diff_blur <= threshold_value 的区域
        below_threshold_mask = diff_blur <= self.threshold_value
        
        # 将热力图中对应位置设为白色
        heatmap_color[below_threshold_mask] = [255, 255, 255]  # BGR白色
        
        # 在热力图上绘制质心
        for (x, y, area) in centroids:
            cv2.drawMarker(heatmap_color, (x, y), (255, 0, 255), 
                          cv2.MARKER_SQUARE, 30, 2)
            cv2.putText(heatmap_color, f"({x},{y})-{area}", (x+10, y-10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 2)
        
        if MOG2points_ori is not None:
            MOG2points = [(int(round(x)), int(round(y))) for x, y in MOG2points_ori]
            for (x, y) in MOG2points:
                cv2.drawMarker(heatmap_color, (x, y), (128, 0, 128), 
                            cv2.MARKER_STAR, 30, 2)
                
        if last_centroid_ori is not None:
            last_centroid = centroids = [(int(round(x)), int(round(y))) for x, y, _ in last_centroid_ori]
            for (x, y) in last_centroid:
                cv2.drawMarker(heatmap_color, (x, y), (0, 128, 0), 
                            cv2.MARKER_TRIANGLE_UP, 30, 2)
        
        # 右侧热力图
        right_panel = np.hstack([heatmap_color])
        
        # 在右侧顶部添加信息
        info_text = f"Threshold: {self.threshold_value} | Objects: {len(centroids)}"
        cv2.putText(right_panel, info_text, (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2)
        
        # 最终组合：左模板 + 右热力图
        combined = np.hstack([left_panel, right_panel])
        
        return combined
    

    def detect_and_track(self, frame, affine_matrix, current_time, get_speed_now):
        """
        执行前景提取、轮廓检测、特征匹配、对象追踪、面积计算等核心逻辑。
        :param frame: 原始输入帧。
        :param affine_matrix: 用于坐标转换的仿射矩阵。
        :param current_time: 当前帧的时间戳。
        :return: tuple (warped_frame, mask, combined_display_image)
                 - warped_frame: 透视校正后的图像。
                 - mask: 前景掩码。
                 - combined_display_image: 用于显示的合成图像。
        """
        self.printLog = False
        current_time0 = time.time()
        if current_time0 - self.lastPrintTime > 0.5:
            self.lastPrintTime = current_time0
            self.printLog = True

        # 图像预处理
        if self.perspective_matrix is not None:
            warped_frame = cv2.warpPerspective(frame, self.perspective_matrix, (self.cfg.output_w, self.cfg.output_h))
            warped_frame = self.frame_to_black(warped_frame)
        else:
            warped_frame = frame.copy()
        if self.frameTh > -1:
            self.frameTh += 1
        if self.frameTh == 30:
            # self.firstFrame = warped_frame
            if AppState.CONFIG["save_first_frame"] is True:
                np.save('frame_data.npy', warped_frame)
            # # 计算每个通道的平均值
            # channel_means = np.mean(self.firstFrame, axis=(0, 1))  # 形状: (3,)

        #     # # 将每个通道的所有值替换为该通道的平均值
        #     # self.firstFrame = np.full_like(self.firstFrame, channel_means)  # 形状: (1080, 1920, 3)
        #     self.frameTh = -1
            self.start_split = True

        foldname = f"/home/xf/imgs/{self.img_filename}/original_pic"
        os.makedirs(foldname, exist_ok=True)
        from datetime import datetime
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S.%f")[:-3]
        filename = f"{foldname}/{timestamp}.png"
        cv2.imwrite(filename, warped_frame)
        centroids = None

        mask = None
        combined_display_image = warped_frame.copy() # 默认返回原始warped图

        # 核心处理逻辑 (仅在背景初始化后)
        if self.background is not None and self.perspective_matrix is not None:
            
            if self.is_use_sam:
                # if centroids is not None:
                #     mask = self.detector_sam2.process_frame_xiefan(warped_frame, self.img_filename, centroids)
                # else:
                #     mask = self.detector_sam2.process_frame_wanqi(warped_frame, self.img_filename)

                mask, movingPointExist, MOG2points = self.detector_sam2.process_frame_jiaqin(warped_frame, self.img_filename, is_static=False)
                last_centroid = None

                if AppState.centroid != []:
                    last_centroid = [[AppState.centroid[AppState.max_tid][0], AppState.centroid[AppState.max_tid][1], 0]]

                if self.start_split is True:
                    LOG_INFO("enter split_draw")
                    centroids, display_img = self.split_draw(warped_frame, MOG2points, last_centroid)
                    window_name = 'Cloth Detection (Left: Video | Center: Heatmap | Right: Colorbar)'
                    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
                    if display_img is not None:
                        cv2.imshow(window_name, display_img)
                    key = cv2.waitKey(1) & 0xFF

                if centroids is not None and movingPointExist is False:
                    mask = self.detector_sam2.process_frame_xiefan(warped_frame, self.img_filename, centroids)

                if centroids is None and movingPointExist is False:
                    mask = self.detector_sam2.process_frame_xiefan(warped_frame, self.img_filename, last_centroid)
                
            else:
                # 1. 前景提取
                diff = cv2.absdiff(warped_frame, self.background)
                gray = cv2.cvtColor(diff, cv2.COLOR_BGR2GRAY)
                gray_blur = cv2.GaussianBlur(gray, (5, 5), 0)
                kernel_sharpen = np.array([[0, -1, 0], [-1, 5, -1], [0, -1, 0]])
                gray_sharp = cv2.filter2D(gray_blur, -1, kernel_sharpen)
                _, mask = cv2.threshold(gray_sharp, self.cfg.diff_threshold, 255, cv2.THRESH_BINARY)
                # print(f"masks_cv:\n{mask}")

            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, self.cfg.morph_kernel_size)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=self.cfg.morph_open_iterations)
            mask = cv2.morphologyEx(mask, cv2.MORPH_DILATE, kernel, iterations=self.cfg.morph_dilate_iterations)

            # 2. 轮廓检测
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            current_objects = []
            for cnt in contours:
                if cv2.contourArea(cnt) < 400 * self.cfg.min_contour_area or cv2.contourArea(cnt) > 400 * self.cfg.max_contour_area:
                    continue

                # 假设 cnt 是当前处理的轮廓（类型为 numpy.ndarray）
                x_coords = cnt[:, :, 0]  # 提取所有点的 x 坐标
                x_min = np.min(x_coords)  # 计算 x 坐标的最小值
                x_max = np.max(x_coords)  # 计算 x 坐标的最大值

                if (x_min < 5 or x_max > AppState.cfg_1.output_w - 5) and ((x_max - x_min) < AppState.cfg_1.output_w * 1 / 2):
                    continue

                # all_enter = (x_min > 10)
                all_enter = True
                
                LOG_INFO("x_min: %f, x_max: %f, w_img: %f", x_min, x_max, AppState.cfg_1.output_w)

                centroid = vu.get_centroid(cnt)
                if centroid and centroid[0] <= self.cfg.output_w * 4 // 5:  # 区域过滤
                    current_objects.append({'contour': cnt, 'centroid': centroid, 'all_enter': all_enter})

            # # 3. 特征匹配 (ReID)
            # current_objects = filter_objects_by_similarity(
            #     current_objects, warped_frame, True, None, self.feature_extractor, self.cfg
            # )

            area_px = 0.0
            area_cm = 0.0
            # 4. 对象追踪与ID分配
            new_tracked_objects = {}
            for obj in current_objects:
                c = obj['centroid']
                assigned = False

                mask_temp = np.zeros_like(mask)
                cv2.drawContours(mask_temp, [obj['contour']], -1, 255, -1)
                area_px = cv2.countNonZero(mask_temp)
                area_cm = vu.pixel_area_to_cm2(area_px, self.cfg.output_w, self.cfg.output_h, self.cfg.real_w, self.cfg.real_h)

                if not self.tracked_objects:
                    LOG_INFO("self.tracked_objects为空")

                for tid, info in self.tracked_objects.items():
                    # 比如self.next_id=9，意味着一共已经有了0-8一共9个已经有的id，我们只需要看后面五个，因此为tid为45678，即9-4=5,9-5=4,全部小于6
                    LOG_INFO("self.next_id: %d, tid: %d", self.next_id, tid)
                    AppState.centroid[tid] = c
                    if self.next_id - tid < 6:
                        dist = np.linalg.norm(np.array(c) - np.array(info['centroid']))
                        LOG_INFO("tid: %d, now centroid x: %f, y: %f, history centroid x: %f, y: %f", tid, c[0], c[1], info['centroid'][0], info['centroid'][1])
                        
                        flag = False
                        if len(self.centroidLastestFive) > 0:
                            if self.centroidLastestFive[tid].empty() is False:
                                length = self.centroidLastestFive[tid].qsize()
                                Xmin = 10000
                                Xmax = -10000
                                for index in range(length):
                                    temp = self.centroidLastestFive[tid].get()
                                    deltaT = time.time() - temp[0]
                                    deltaX = deltaT * get_speed_now * AppState.cfg_1.ratio_wh
                                    predictX = temp[1][0] + deltaX
                                    nowX = c[0]
                                    LOG_INFO("tid: %d, predictX - nowX: %f", tid, predictX - nowX)
                                    if abs(predictX - nowX) < 20 * self.cfg.max_distance:
                                        flag = True
                                    self.centroidLastestFive[tid].put(temp)

                                    Xmin = min(Xmin, temp[1][0])
                                    Xmax = min(Xmax, temp[1][0])
                                if abs(Xmax - Xmin) < 10:
                                    self.isStatic[tid] = True
                        if flag is True:
                            if self.nowAreaAvg:
                                if self.nowAreaAvg[tid]:
                                    LOG_INFO("self.nowAreaAvg[%d]: %f", tid, self.nowAreaAvg[tid])
                            # if len(self.area_dict[tid]) > 1:
                            #     flag = False
                        LOG_INFO("tid: %d, flag: %d", tid, flag)
                        if flag is True:
                            # 继承历史信息
                            info['position_history'].append(c)
                            info['time_history'].append(current_time)
                            if current_time - self.last_speed_update_time > self.cfg.speed_update_interval:
                                info['speed'] = calculate_speed(info['position_history'], info['time_history'], current_time, self.cfg)
                            
                            new_tracked_objects[tid] = {
                                **info,
                                'centroid': c,
                                'contour': obj['contour']
                            }
                            assigned = True
                            self.pixel_area_dict[tid].append(area_px)
                            if obj["all_enter"]:
                                self.area_dict[tid].append(area_cm)
                            LOG_INFO("tid: %d, 增加新面积: %f", tid, area_cm)

                            queue = Queue()
                            timenow = time.time()
                            struct = [timenow, c]
                            self.centroidLastestFive[tid].put(struct)
                            if self.centroidLastestFive[tid].qsize() > self.numCentroids:
                                pop = self.centroidLastestFive[tid].get()
                            
                            if len(self.centroidLastestFive) > 0:
                                if self.centroidLastestFive[tid].empty() is False:
                                    length = self.centroidLastestFive[tid].qsize()
                                    for index in range(length):
                                        temp = self.centroidLastestFive[tid].get()
                                        LOG_INFO("tid: %d, length: %d, queue[%d][%d] x: %f, y: %f", tid, length, tid, index, temp[1][0], temp[1][1])
                                        self.centroidLastestFive[tid].put(temp)
                            flag = False
                            if len(self.id_now) > 0:
                                for index in range(len(self.id_now)):
                                    LOG_INFO("tid: %d, self.id_now[%d][0] = %d, self.id_now[%d][1] = %d", tid, index, self.id_now[index][0], index, self.id_now[index][1])
                                    if(self.id_now[index][0] == tid):
                                        flag = True
                                        self.id_now[index][1] = self.numCentroids
                                    else:
                                        if self.id_now[index][1] >= 0:
                                            self.id_now[index][1] -= 1
                                        if(self.id_now[index][1] < 0):
                                            if len(self.area_dict[index]) > 0:
                                                areaTemp = median_trimmed_mean(self.area_dict[index])
                                                # areaTemp = sum(self.area_dict[index]) / len(self.area_dict[index])
                                                if areaTemp > 0:
                                                    self.lastAreaAvg = areaTemp
                                                    LOG_INFO("last Areaavg: %f", self.lastAreaAvg)
                                                    if tid == 1:
                                                        self.templateArea = areaTemp
                                                self.area_dict[index] = [-1.0]
                                                # LOG_INFO("tid: %d删除id为%d的面积, 被删除的面积为%f", tid, index, areaTemp)
                            if flag is False:
                                count = self.numCentroids
                                struct = [tid, count]
                                self.id_now.append(struct)

                            # 比如现在的len(self.area_dict[tid])=5，意味着里面一共有五个历史面积数据，现在是第五个，前面四个已经求了均值
                            # 若len=10，意味着一共有1+4个历史数据，现在是第六个，前面五个已经求了均值
                            if len(self.area_dict) > 0:
                                # self.update_area_statistics(tid)

                                # if len(self.area_dict[tid]) == 5:
                                #     self.area_dict[tid].sort()
                                #     self.nowAreaAvg[tid] = self.area_dict[tid][2] # 第三个值
                                # if len(self.area_dict[tid]) > 5:
                                #     self.nowAreaAvg[tid] = (self.nowAreaAvg[tid] * (len(self.area_dict[tid]) - 5) + 
                                #                             self.area_dict[tid][len(self.area_dict[tid]) - 1]) / (len(self.area_dict[tid]) - 4)
                                if (len(self.area_dict[tid]) > 10 or len(self.area_dict[tid]) < 20) and self.warnId[tid] == False:
                                    self.nowAreaAvg[tid] = median_trimmed_mean(self.area_dict[tid])
                                    LOG_INFO("self.templateArea: %f", self.templateArea)
                                    if self.templateArea != 0:
                                        area_ratio = self.nowAreaAvg[tid] / self.templateArea
                                        if area_ratio < 0.97:
                                            LOG_WARN("异常检测报警，为卷曲或者更换布料, 第一张布料面积为%f, 当前布料面积为%f, 面积比例为%f", self.templateArea, self.nowAreaAvg[tid], self.nowAreaAvg[tid] / self.templateArea)
                                            self.warnId[tid] = True
                                            self.permitGrasp[tid] = False

                                #             abnormal_info={
                                #                 "jam_flag": False ,
                                #                 "curl_flag" : True ,
                                #                 "tid":tid ,
                                #                 "area_ratio" :area_ratio,
                                #                 "template_area" : self.templateArea,
                                #                 "current_area" :self .nowAreaAvg[tid],
                                #                 "reason" : "卷曲或者换布料", 
                                #             }
                                #             abnormal , abnormal_name = self.is_abnormal_cloth (abnormal_info)
                                #         if abnormal:
                                #                 info["abnormal"] = True 
                                #                 info["abnormal_name"] = abnormal_name
                                #                 info["abnormal_info"] = abnormal_info

                            
                            # break
                
                if not assigned:
                    self.warnId.append(False)
                    self.permitGrasp.append(True)
                    new_tracked_objects[self.next_id] = {
                        'centroid': c,
                        'color': vu.random_color(),
                        'contour': obj['contour'],
                        'position_history': deque([c], maxlen=self.cfg.max_speed_history),
                        'time_history': deque([current_time], maxlen=self.cfg.max_speed_history),
                        'speed': 0.0
                    }
                    self.pixel_area_dict.append([area_px])
                    if obj["all_enter"]:
                        self.area_dict.append([area_cm])
                    LOG_INFO("增加新面积: %f", area_cm)
                    # if self.lastAreaAvg != 0:
                    #     if area_cm / self.lastAreaAvg > 1.02 or area_cm / self.lastAreaAvg < 0.98:
                    #         LOG_WARN("异常检测报警，为卷曲或者更换布料")
                    queue = Queue()
                    timenow = time.time()
                    struct = [timenow, c]
                    queue.put(struct)
                    self.centroidLastestFive.append(queue)

                    if len(self.centroidLastestFive) > 0:
                        if self.centroidLastestFive[self.next_id].empty() is False:
                            length = self.centroidLastestFive[self.next_id].qsize()
                            for index in range(length):
                                temp = self.centroidLastestFive[self.next_id].get()
                                LOG_INFO("length: %d, queue[%d][%d] x: %f, y: %f", length, self.next_id, index, temp[1][0], temp[1][1])
                                self.centroidLastestFive[self.next_id].put(temp)

                    flag = False
                    if len(self.id_now) > 0:
                        for index in range(len(self.id_now)):
                            if(self.id_now[index][0] == self.next_id):
                                flag = True
                                self.id_now[index][1] = self.numCentroids
                            else:
                                if self.id_now[index][1] >= 0:
                                    self.id_now[index][1] -= 1
                                if(self.id_now[index][1] < 0):
                                    if len(self.area_dict[index]) > index:
                                        areaTemp = sum(self.area_dict[index]) / len(self.area_dict[index])
                                        if areaTemp > 0 and len(self.area_dict[index]) > 20:
                                            # LOG_INFO("areaTemp = %f, self.next_id = %d", areaTemp, self.next_id)
                                            self.lastAreaAvg = areaTemp
                                            if self.next_id == 1:
                                                self.templateArea = areaTemp
                                        self.area_dict[index] = [-1.0]
                                        LOG_INFO("删除id为%d的面积, 被删除的面积为%f", index, areaTemp)
                    if flag is False:
                        count = self.numCentroids
                        struct = [self.next_id, count]
                        self.id_now.append(struct)

                    if len(self.area_dict) > 0:
                        self.nowAreaAvg.append(-1.0)
                    
                    self.isStatic.append(False)
                    AppState.centroid.append(c)
                    AppState.max_tid = self.next_id

                    self.next_id += 1
            
            # if self.printLog:
            #     LOG_INFO("last Areaavg: %f", self.lastAreaAvg)

            copy_last_frame = False
            if not new_tracked_objects:
                self.saveLastFrame -= 1
                LOG_INFO("copy last frame")
                copy_last_frame = True
                if self.saveLastFrame < 0:
                    self.tracked_objects = new_tracked_objects
                    LOG_INFO("copy last frame 5th")
            else:
                self.saveLastFrame = 5
                self.tracked_objects = new_tracked_objects

            # 5. 更新面积与运动状态
            # if current_time - self.last_area_update_time > self.cfg.area_update_interval:
            # for tid, info in self.tracked_objects.items():
            #     self.pixel_area_dict[tid] = area_px
            #     self.area_dict[tid] = area_cm
                
                # self.last_area_update_time = current_time

            line2_x = self.cfg.output_w * 1 // 2
            for tid, info in self.tracked_objects.items():
                # 计算短边中心
                single_mask = np.zeros_like(mask)
                cv2.drawContours(single_mask, [info['contour']], -1, 255, -1)
                sc1, sc2, angle, rect, longEdge = vu.get_short_edge_centers_and_angle(info['contour'], single_mask)
                info['short_center1'] = sc1
                info['short_center2'] = sc2
                info['rect'] = rect
                info['angle'] = angle

                # 计算长度
                long_side_px = math.hypot(sc1[0] - sc2[0], sc1[1] - sc2[1])

                if AppState.CONFIG["testStaticFabricLength"] or AppState.CONFIG["testDynamicFabricLength"]:
                    current_time = time.time()
                    if current_time - self.last_print_time >= self.interval:
                        self.x_min = min(info['centroid'][0], self.x_min)
                        self.y_min = min(info['centroid'][1], self.y_min)
                        self.x_max = max(info['centroid'][0], self.x_max)
                        self.y_max = max(info['centroid'][1], self.y_max)
                        self.t_min = min(angle, self.t_min)
                        self.t_max = max(angle, self.t_max)
                        print(f"id: {tid}, x_min: {self.x_min}, y_min: {self.y_min}, x_max: {self.x_max}, y_max: {self.y_max}, t_min: {self.t_min}, t_max: {self.t_max}")
                        self.last_print_time = current_time
                
                # area = 0
                # if tid < len(self.area_dict):
                #     area = self.area_dict[tid]
                
                LOG_INFO("id: %d, 质心x: %f, 质心y: %f, t: %f", tid, info['centroid'][0], info['centroid'][1], angle)
                if(len(self.area_dict) > tid):
                    areaTid = self.area_dict[tid]
                    if(len(areaTid) > 0):
                        x = min(self.numCentroids, len(areaTid))
                        for index in range(x):
                            LOG_INFO("%dth area %d: %f", tid, index, areaTid[index])


                info['long_side_length'] = long_side_px / self.cfg.ratio_wh * 10
                #异常检测调用
                # self.apply_abnormal_result(tid ,info)

                # 抓取点计算
                if tid not in self.grab_calculators:
                    self.grab_calculators[tid] = StableGrabCenter()
                    self.grab_history[tid] = deque(maxlen=50)
                
                if info['centroid'][0] >= line2_x:
                    gb_center, gb_angle = self.grab_calculators[tid].get_stable_grab_center_from_mask(
                        single_mask, info['centroid'], extension_ratio=0.48
                    )
                    self.grab_history[tid].append(gb_center if gb_center else info['centroid'])
                else:
                    self.grab_history[tid].append(info['centroid'])

                LOG_INFO("global centroid: id: %d, x: %f, y: %f", tid, AppState.centroid[tid][0], AppState.centroid[tid][1])

                # 更新运动状态
                update_motion_status(self.img_filename, longEdge, single_mask, self.motion_dict, tid, info['centroid'], info['centroid'], current_time, info['angle'], info['long_side_length'], affine_matrix, self.cfg, self.permitGrasp, self.isStatic, copy_last_frame)

            # 6. 绘制与显示 (传入 cfg)
            mask_colored = np.zeros_like(warped_frame)
            for tid, info in self.tracked_objects.items():
                cv2.drawContours(mask_colored, [info['contour']], -1, info['color'], -1)
                if info['rect'] is not None:
                    box = np.int32(cv2.boxPoints(info['rect']))
                    cv2.drawContours(mask_colored, [box], 0, (255, 0, 0), 5)
            
            combined_display_image = cv2.addWeighted(warped_frame, self.cfg.blend_alpha, mask_colored, self.cfg.blend_alpha, 0)
            draw_motion_info(combined_display_image, self.motion_dict, self.tracked_objects, self.cfg)


        return warped_frame, mask, combined_display_image

    def get_motion_dict(self):
        """获取当前的运动字典"""
        return self.motion_dict
    
    def get_tracked_objects(self):
        """获取当前追踪的对象"""
        return self.tracked_objects
    
    def reset_frame_counter(self):
        """重置帧计数器（如果需要）"""
        self.frame_idx = 0

    def increment_frame_counter(self):
        """增加帧计数器"""
        self.frame_idx += 1

    def get_motionDict_trackedObjects_DisplayImg(self, frame, AFFINE_MATRIX_1, current_time, get_speed_now):
        # 让检测器递增帧计数器
        self.increment_frame_counter()

        # 尝试初始化背景
        self.initialize_background(frame)
        warped_frame, mask, display_img = self.detect_and_track(frame, AFFINE_MATRIX_1, current_time, get_speed_now)# G 这里AFFINE_MATRIX是用来计算机械臂的移动方向（当前并未使用，使用的是写死的值）
        
        # 获取检测结果
        motion_dict = self.get_motion_dict()
        tracked_objects = self.get_tracked_objects()
        return motion_dict, tracked_objects, display_img
    
def median_trimmed_mean(data, k=1): # 中位数滤波
    median = np.median(data)
    mad = np.median([abs(x - median) for x in data])
    filtered = [x for x in data if abs(x - median) <= k * mad]
    return np.mean(filtered)