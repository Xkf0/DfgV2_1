import numpy as np
import cv2
from getPointCsv import getPointCsv

# 计算变换矩阵
def compute_affine_transform(real_points, robot_points,
                                    ransac_thresh=3.0,
                                    confidence=0.99):
    """
    使用 RANSAC 估计 2D 仿射变换（更鲁棒）
    real_points  : (N,2) 视觉/现实坐标
    robot_points : (N,2) 机械臂坐标
    """

    real_pts = np.asarray(real_points, dtype=np.float32)
    robot_pts = np.asarray(robot_points, dtype=np.float32)

    assert real_pts.shape[0] >= 3

    affine_matrix, inliers = cv2.estimateAffine2D(
        real_pts,
        robot_pts,
        method=cv2.RANSAC,
        ransacReprojThreshold=ransac_thresh,
        confidence=confidence,
        refineIters=10
    )

    if affine_matrix is None:
        raise RuntimeError("RANSAC 仿射估计失败，请检查点分布")

    inlier_ratio = inliers.sum() / len(inliers)

    print(f"RANSAC 内点比例: {inlier_ratio:.2%}")

    return affine_matrix

# 视觉坐标转为机械臂坐标
def transform_point(affine_matrix, point):
    """
    使用仿射矩阵将现实坐标映射为机械臂坐标
    """

    x, y = point
    vec = np.array([x, y, 1])
    # print("affine_matrix @ vec:",affine_matrix @ vec)
    x_robot, y_robot = affine_matrix @ vec
    return x_robot, y_robot

def trans_points(testPoint, points_file_path='cv_aruco_1.csv'):

    # 获取视觉坐标、机械臂坐标
    ROBOT_POINTS, REAL_PTS = getPointCsv(points_file_path)

    # 计算变换矩阵
    AFFINE_MATRIX = compute_affine_transform(REAL_PTS, ROBOT_POINTS)

    # 测试所用视觉坐标

    # testPoint = (985.75, 1291.5)
    # 4：-458.604,235.434     9：-458.607,235.227
    # 视觉坐标转为机械臂坐标
    robot_point_x, robot_point_y = transform_point(AFFINE_MATRIX, testPoint)

    print(f"转换后的机械臂坐标为：{robot_point_x:.3f},{robot_point_y:.3f}")

def main():
    trans_points((523.03, 619.40))

if __name__ == '__main__':
    main()

