import csv
import math
import os

def read_coordinates_from_csv(file_path):
    """
    从CSV文件读取指定ID的坐标数据（兼容多分隔符，增加调试）
    返回格式: {id: {'robot_x': x, 'robot_y': y, 'cv_x': x, 'cv_y': y}}
    """
    coordinates = {}
    
    # 先检查文件是否存在
    if not os.path.exists(file_path):
        print(f"错误：文件 {file_path} 不存在！")
        return coordinates
    
    # 读取文件，先尝试自动检测分隔符，兼容制表符/逗号/空格
    with open(file_path, 'r', encoding='utf-8-sig') as csvfile:
        # 读取第一行（表头），清理空格和特殊字符
        header = csvfile.readline().strip()
        print(f"【调试】CSV表头原始内容: {header}")
        
        # 自动判断分隔符（优先制表符，其次逗号，最后空格）
        if '\t' in header:
            delimiter = '\t'
        elif ',' in header:
            delimiter = ','
        else:
            delimiter = ' '  # 空格分隔
        print(f"【调试】自动识别的分隔符: {repr(delimiter)}")
        
        # 重置文件指针到开头，重新读取
        csvfile.seek(0)
        
        # 创建CSV读取器，忽略空行，清理列名空格
        reader = csv.DictReader(csvfile, delimiter=delimiter)
        # 清理列名（去除首尾空格、特殊字符）
        reader.fieldnames = [f.strip() for f in reader.fieldnames] if reader.fieldnames else []
        print(f"【调试】清理后的列名: {reader.fieldnames}")
        
        for row_num, row in enumerate(reader, start=2):  # 行号从2开始（表头是1）
            try:
                # 打印原始行数据（调试用）
                print(f"【调试】第{row_num}行原始数据: {row}")
                
                # 提取ID（确保列名正确）
                if 'id' not in row:
                    print(f"【警告】第{row_num}行无'id'列，跳过")
                    continue
                current_id_str = row['id'].strip()
                if not current_id_str:
                    print(f"【警告】第{row_num}行ID为空，跳过")
                    continue
                current_id = int(current_id_str)
                
                # 提取并清理坐标值（处理逗号/空格，兼容空值）
                def get_float_value(row, key):
                    """安全获取浮点数值"""
                    val = row.get(key, '').strip().replace(',', '.')
                    return float(val) if val else 0.0
                
                robot_x = get_float_value(row, 'robot_x')
                robot_y = get_float_value(row, 'robot_y')
                cv_x = get_float_value(row, 'cv_x')
                cv_y = get_float_value(row, 'cv_y')
                
                # 存储到字典
                coordinates[current_id] = {
                    'robot_x': robot_x,
                    'robot_y': robot_y,
                    'cv_x': cv_x,
                    'cv_y': cv_y
                }
                print(f"【调试】成功读取ID {current_id}: {coordinates[current_id]}")
                
            except ValueError as e:
                print(f"【错误】第{row_num}行数据格式错误: {e}")
                continue
            except Exception as e:
                print(f"【错误】第{row_num}行处理异常: {e}")
                continue
    return coordinates

def calculate_unit_vector(start_point, end_point):
    """
    计算从起点到终点的单位向量（x、y分量）
    start_point/end_point格式: (x, y)
    返回: (unit_x, unit_y)，分量平方和=1
    """
    dx = end_point[0] - start_point[0]
    dy = end_point[1] - start_point[1]
    
    magnitude = math.sqrt(dx**2 + dy**2)
    if magnitude == 0:
        return (0.0, 0.0)
    
    unit_x = dx / magnitude
    unit_y = dy / magnitude
    return (unit_x, unit_y)

def calculate_coordinate_average(coord_data, id_list, coord_type):
    """
    计算指定ID列表的坐标均值（支持robot/cv两种坐标类型）
    参数:
        coord_data: 从read_coordinates_from_csv获取的坐标字典
        id_list: 要计算均值的ID列表，如[80, 83, 87]
        coord_type: 坐标类型，可选'robot'或'cv'
    返回:
        (mean_x, mean_y): 计算得到的x、y均值；若有效ID为0，返回(0.0, 0.0)
    """
    if coord_type not in ['robot', 'cv']:
        print(f"【错误】不支持的坐标类型 {coord_type}，仅支持'robot'或'cv'")
        return (0.0, 0.0)
    
    # 定义对应的x、y键名
    x_key = f"{coord_type}_x"
    y_key = f"{coord_type}_y"
    
    valid_x = []
    valid_y = []
    missing_ids = []
    
    # 遍历ID列表，收集有效坐标
    for target_id in id_list:
        if target_id in coord_data:
            valid_x.append(coord_data[target_id][x_key])
            valid_y.append(coord_data[target_id][y_key])
            print(f"【调试】收集ID {target_id} {coord_type}坐标: ({coord_data[target_id][x_key]:.8f}, {coord_data[target_id][y_key]:.8f})")
        else:
            missing_ids.append(target_id)
    
    # 输出缺失ID警告
    if missing_ids:
        print(f"【警告】未找到以下ID的坐标数据: {', '.join(map(str, missing_ids))}")
    
    # 计算均值（避免除以0）
    if not valid_x or not valid_y:
        print(f"【错误】无有效坐标数据，无法计算{coord_type}坐标均值")
        return (0.0, 0.0)
    
    mean_x = sum(valid_x) / len(valid_x)
    mean_y = sum(valid_y) / len(valid_y)
    
    print(f"【调试】计算完成，{coord_type}坐标均值: ({mean_x:.8f}, {mean_y:.8f})（基于{len(valid_x)}个有效ID）")
    return (mean_x, mean_y)

if __name__ == "__main__":
    # -------------------------- 配置参数 --------------------------
    csv_file_path = "cv_aruco_0.csv"  # 【必须修改】替换为你的CSV文件实际路径
    start_id_list = [80, 83, 87]  # 起点ID列表
    end_id_list = [82, 86, 91]    # 终点ID列表
    # -------------------------------------------------------------
    
    # 1. 读取CSV数据
    print("=== 开始读取CSV文件 ===")
    coord_data = read_coordinates_from_csv(csv_file_path)
    print(f"=== 读取完成，有效ID列表: {sorted(list(coord_data.keys()))} ===")
    
    # 2. 计算斜线1（robot坐标）的起点和终点均值
    print("\n=== 计算斜线1（robot坐标）的起点/终点均值 ===")
    line1_start = calculate_coordinate_average(coord_data, start_id_list, 'robot')
    line1_end = calculate_coordinate_average(coord_data, end_id_list, 'robot')
    
    # 验证斜线1坐标是否有效
    if line1_start == (0.0, 0.0) or line1_end == (0.0, 0.0):
        print("\n【错误】斜线1坐标计算失败，程序退出")
        exit(1)
    print(f"\n斜线1 起点（{', '.join(map(str, start_id_list))}均值）: {line1_start}")
    print(f"斜线1 终点（{', '.join(map(str, end_id_list))}均值）: {line1_end}")
    
    # 3. 计算斜线2（cv坐标）的起点和终点均值
    print("\n=== 计算斜线2（cv坐标）的起点/终点均值 ===")
    line2_start = calculate_coordinate_average(coord_data, start_id_list, 'cv')
    line2_end = calculate_coordinate_average(coord_data, end_id_list, 'cv')
    
    # 验证斜线2坐标是否有效
    if line2_start == (0.0, 0.0) or line2_end == (0.0, 0.0):
        print("\n【错误】斜线2坐标计算失败，程序退出")
        exit(1)
    print(f"\n斜线2 起点（{', '.join(map(str, start_id_list))}均值）: {line2_start}")
    print(f"斜线2 终点（{', '.join(map(str, end_id_list))}均值）: {line2_end}")
    
    # 4. 计算斜线2的单位运动分量
    line2_start = (line2_start[0]-line1_start[0],line2_start[1]-line1_start[1])
    line2_end = (line2_end[0]-line1_end[0],line2_end[1]-line1_end[1])
    # line2_start = (line1_start[0]-line2_start[0],line1_start[1]-line2_start[1])
    # line2_end = (line1_end[0]-line2_end[0],line1_end[1]-line2_end[1])
    line2_unit_x, line2_unit_y = calculate_unit_vector(line2_start, line2_end)
    
    # 5. 输出结果并验证
    print("\n=== 斜线2的单位运动分量 ===")
    print(f"x分量: {line2_unit_x:.8f}")
    print(f"y分量: {line2_unit_y:.8f}")
    print(f"分量平方和验证: {line2_unit_x**2 + line2_unit_y**2:.8f} (应接近1)")
