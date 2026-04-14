# config_loader.py
import numpy as np

class Configer:
    def __init__(self, 
        real_w, real_h, 
        ratio_wh,
        aruco_ids_corner, 
        speed_smooth_frames=5,
        similarity_threshold=0.85,
        diff_threshold=25,
        morph_kernel_size=(3,3),
        morph_open_iterations=1,
        morph_dilate_iterations=2,
        min_contour_area=500,
        max_contour_area=1000000,
        max_distance=5,
        
        max_speed_history=10,
        speed_update_interval=0.1,
        area_update_interval=0.2,
        stand_rz=0.0,
        speed=10.0,
        WAIT_DISTANCE=90,
        safe_pos_init=None,
        stand_z=300.0,
        stand_rx=180.0,
        stand_ry=0.0,
        blend_alpha=0.5,
        is_real_sense=False,
        camera_no=0,
        points_file_path="points.txt",
        down_height=50.0,
        safe_pos_1=None,
        safe_pos_2=None,
        safe_pos_1_5=None,
        detect_pose=None,
        detect_none_pose=None,
        testStaticFabricLength=False,
        testDynamicFabricLength=False,
        test_x_cm=0,
        test_y_cm=0,
        robot_ip1=None,
        robot_ip2=None,
        length_lead_screw_cm=0.0,
        openCollisionDetect=False,
        speed_mode=1,
        speed_port="",
        id_wcs=2,
        safe_x=0,
        safe_y=0,
        safe_z=0,
        detect_x=0,
        detect_y=0,
        save_first_frame=False,
        ):

        self.real_w = real_w
        self.real_h = real_h
        self.aruco_ids_corner = aruco_ids_corner
        self.speed_smooth_frames = speed_smooth_frames
        self.similarity_threshold = similarity_threshold
        self.diff_threshold = diff_threshold
        self.morph_kernel_size = morph_kernel_size
        self.morph_open_iterations = morph_open_iterations
        self.morph_dilate_iterations = morph_dilate_iterations
        self.min_contour_area = min_contour_area
        self.max_contour_area = max_contour_area
        self.max_speed_history = max_speed_history
        self.speed_update_interval = speed_update_interval
        self.area_update_interval = area_update_interval
        self.ratio_wh = ratio_wh
        self.stand_rz = stand_rz
        self.speed = speed
        self.safe_pos_init = safe_pos_init
        self.stand_z = stand_z
        self.stand_rx = stand_rx
        self.stand_ry = stand_ry
        self.blend_alpha = blend_alpha
        self.is_real_sense = is_real_sense
        self.camera_no = camera_no
        self.points_file_path = points_file_path
        self.down_height = down_height
        self.safe_pos_1 = safe_pos_1 if safe_pos_1 is not None else []
        self.safe_pos_2 = safe_pos_2 if safe_pos_2 is not None else []
        self.safe_pos_1_5 = safe_pos_1_5 if safe_pos_1_5 is not None else []

        self.detect_pose = detect_pose if detect_pose is not None else []
        self.detect_none_pose = detect_none_pose if detect_none_pose is not None else []

        self.output_w = int(real_w * ratio_wh)
        self.output_h = int(real_h * ratio_wh)

        # self.max_contour_area = self.output_w * self.output_h / 3

        self.max_distance = max_distance
        
        self.WAIT_DISTANCE = WAIT_DISTANCE

        self.dst_points = np.float32([
            [0, 0],
            [self.output_w - 1, 0],
            [self.output_w - 1, self.output_h - 1],
            [0, self.output_h - 1]
        ])

        # self.time_pre = 90 / self.speed
        self.time_pre = self.WAIT_DISTANCE / self.speed

        self.safe_pos_init[2] = self.stand_z

        self.testStaticFabricLength = testStaticFabricLength
        self.testDynamicFabricLength = testDynamicFabricLength

        self.test_x_cm = test_x_cm
        self.test_y_cm = test_y_cm
        self.robot_ip1 = robot_ip1
        self.robot_ip2 = robot_ip2
        self.length_lead_screw_cm = length_lead_screw_cm
        self.openCollisionDetect = openCollisionDetect
        self.speed_mode = speed_mode
        self.speed_port = speed_port
        self.id_wcs = id_wcs
        self.safe_x = safe_x
        self.safe_y = safe_y
        self.safe_z = safe_z
        self.detect_x = detect_x
        self.detect_y = detect_y
        self.save_first_frame = save_first_frame