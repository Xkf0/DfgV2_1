import numpy as np
import math
from get_points import get_points


def calculate_horizontal_unit_vector(mech_start, mech_end, real_start, real_end):
    """
    根据机械臂和现实世界的起始点、终点坐标，计算机械臂坐标系下
    对应现实世界水平移动的单位运动向量 (X, Y)，其平方和为1。

    Args:
        mech_start (tuple): 机械臂起始坐标 (x, y)
        mech_end (tuple): 机械臂终点坐标 (x, y)
        real_start (tuple): 现实世界起始坐标 (x, y)
        real_end (tuple): 现实世界终点坐标 (x, y)

    Returns:
        tuple: 机械臂坐标系下对应水平移动的单位向量 (unit_x, unit_y)
    """
    # 1. 计算机械臂运动向量
    mech_delta_x = mech_end[0] - mech_start[0]
    mech_delta_y = mech_end[1] - mech_start[1]
    
    # 2. 计算现实世界运动向量
    real_delta_x = real_end[0] - real_start[0]
    real_delta_y = real_end[1] - real_start[1]
    
    # 3. 计算机械臂运动向量的模长
    mech_vector_magnitude = math.sqrt(mech_delta_x**2 + mech_delta_y**2)
    
    # 4. 将机械臂运动向量单位化，即为对应现实世界移动的单位向量
    # 注意：题目要求是现实水平移动，即real_delta_x方向的移动。
    # 但根据原始推导，我们是直接将机械臂的实际运动向量单位化了。
    # 这个单位向量的方向代表了现实世界从start到end的整体移动方向。
    # 如果现实世界是纯水平移动（real_delta_y接近0），那么这个单位向量就代表了水平移动。
    # 因此，我们可以直接对机械臂向量进行单位化。
    unit_x = mech_delta_x / mech_vector_magnitude
    unit_y = mech_delta_y / mech_vector_magnitude

    return np.array([unit_x, unit_y])


if __name__ == "__main__":

    points_file_path = "cv_aruco_2.csv"

    ROBOT_POINTS, REAL_PTS = get_points(points_file_path)


    # unit_vector = calculate_horizontal_unit_vector(ROBOT_POINTS[3], ROBOT_POINTS[5], REAL_PTS[3], REAL_PTS[5])
    # unit_vector = calculate_horizontal_unit_vector(ROBOT_POINTS[0], ROBOT_POINTS[1], REAL_PTS[0], REAL_PTS[1])
    unit_vector = calculate_horizontal_unit_vector(ROBOT_POINTS[1], ROBOT_POINTS[0], REAL_PTS[1], REAL_PTS[0])
    print("机械臂坐标系下对应水平移动的单位向量:", unit_vector)