# step1 获取视觉坐标
    0.5 x-y-平面标定法标工件坐标系
    1.0 板子放好以后，看是否最远点能达到
    1.05 机械臂挪远
    1.1 运行'getPointAruco_9_mean.py'
    1.2 待点稳定（通常在'采样数量'为500以上）
    1.3 将对应ArUco_ID的'X坐标均值'与'Y坐标均值'填入'cv_aruco_0.csv'的'cv_x'与'cv_y'
        cv_aruco_0.csv中*为机械臂编号

# step2 获取机械臂坐标
    2.1 更换标定结构件，运动至每一个aruco点的中心点
        通常上位机中的RX为-180，RY为0，RZ无固定值
    2.2 记录X、Y、Z，填入'cv_aruco_0.csv'的'robot_x'、'robot_y'与'robot_z'
        Z目前无需标定，先记录待后续使用，cv_z暂为空
    2.3 对应Aruco图形与id见'aruco_9.png'

# step3 验证标定效果
    3.1 将标定板随机置放至检测区域
    3.2 运行'getPointAruco_9_mean.py'
    3.3 待点稳定（通常在'采样数量'为500以上）
    3.4 取任意aruco点，并填入'TransPoint.py'中'testPoint'
    3.5 修改'TransPoint.py'中'points_file_path'为所维护'cv_aruco_0.csv'
    3.6 运行'TransPoint.py'
        读取输出'转换后的机械臂坐标'
        将其输入上位机中的X、Y,Z为尽量高的值（防止碰撞），RX为-180，RY为0，RZ无固定值
    3.7 微调Z值，直至标定件触碰所选Aruco码中心点，若击中，则标定成功
    3.8 用此方法处理剩余五个点，并且微调到对的位置，然后放到csv厘米

# step4 标定文件复制至Dfg工程
    4.1 将'cv_aruco_0.csv' （*需改为机械臂编号，通常为1或2）移动至Dfg工程根目录（'main.py'同目录下）
    4.2 测量传送带方向向量，取两个点，算单位向量
    4.3 找出四个位置参数，safe_pos_2+detect_pose+detect_none_pose是放置位置上面的等待位，"stand_z": 99, "stand_rx":-179.486, "stand_ry": -1.859, "stand_rz": 133.442,是传送带上面等待夹取的位置，safe_pos_init的后面三个也是水平位姿，place_pos_arm_1是放置布料的位置

# 附录 
## 夹爪控制
    grip_open：夹爪充气打开到最大
    grip_clamp：夹爪闭合（吸气）
    grip_release：夹爪从松开气夹=》张到最大=》常规态

