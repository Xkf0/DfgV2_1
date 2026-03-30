import csv

def get_points(points_file_path):
    # 初始化数组
    ROBOT_POINTS = []
    REAL_PTS = []

    # 读取CSV文件
    with open(points_file_path, 'r', encoding='utf-8') as csvfile:
        reader = csv.DictReader(csvfile)
        
        for row in reader:
            # 提取机器人坐标并转换为浮点数
            robot_x = float(row['robot_x'])
            robot_y = float(row['robot_y'])
            # robot_z = float(row['robot_z'])
            ROBOT_POINTS.append([robot_x, robot_y])
            
            # 提取CV坐标并转换为整数
            cv_x = float(row['cv_x'])
            cv_y = float(row['cv_y'])
            REAL_PTS.append([cv_x, cv_y])

    # 转换为numpy数组（可选，如果需要数学运算）
    import numpy as np
    ROBOT_POINTS = np.array(ROBOT_POINTS)
    REAL_PTS = np.array(REAL_PTS)

    # 打印结果验证
    print('-------------------------')
    print('ROBOT_POINTS:')
    print(ROBOT_POINTS)
    print(f"Shape: {ROBOT_POINTS.shape}")
    print('-------------------------')
    print('REAL_PTS:')
    print(REAL_PTS)
    print(f"Shape: {REAL_PTS.shape}")
    print('-------------------------')

    return ROBOT_POINTS, REAL_PTS

if __name__ == '__main__':
    get_points('cv_aruco.csv')