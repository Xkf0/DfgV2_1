import xmlrpc.client
import os
import socket
import hashlib
import time
from datetime import datetime
import logging
from functools import wraps
from logging.handlers import RotatingFileHandler
from queue import Queue
import threading
import struct
import sys
import ctypes
from ctypes import *


class RobotStatePkg(Structure):
    _pack_ = 1
    _fields_ = [
        ("frame_head", c_uint16),  # 帧头 0x5A5A
        ("frame_cnt", c_byte),  # 帧计数
        ("data_len", c_uint16),  # 数据长度
        ("program_state", c_byte),  # 程序运行状态，1-停止；2-运行；3-暂停
        ("robot_state", c_byte),  # 机器人运动状态，1-停止；2-运行；3-暂停；4-拖动
        ("main_code", c_int),  # 主故障码
        ("sub_code", c_int),  # 子故障码
        ("robot_mode", c_byte),  # 机器人模式，0-自动模式；1-手动模式
        ("jt_cur_pos", c_double * 6),  # 机器人当前关节位置，假设有6个关节
        ("tl_cur_pos", ctypes.c_double * 6),  # 工具当前位姿
        ("flange_cur_pos", ctypes.c_double * 6),  # 末端法兰当前位姿
        ("actual_qd", ctypes.c_double * 6),  # 机器人当前关节速度
        ("actual_qdd", ctypes.c_double * 6),  # 机器人当前关节加速度
        ("target_TCP_CmpSpeed", ctypes.c_double * 2),  # 机器人TCP合成指令速度
        ("target_TCP_Speed", ctypes.c_double * 6),  # 机器人TCP指令速度
        ("actual_TCP_CmpSpeed", ctypes.c_double * 2),  # 机器人TCP合成实际速度
        ("actual_TCP_Speed", ctypes.c_double * 6),  # 机器人TCP实际速度
        ("jt_cur_tor", ctypes.c_double * 6),  # 当前扭矩
        ("tool", ctypes.c_int),  # 工具号
        ("user", ctypes.c_int),  # 工件号
        ("cl_dgt_output_h", ctypes.c_byte),  # 数字输出15-8
        ("cl_dgt_output_l", ctypes.c_byte),  # 数字输出7-0
        ("tl_dgt_output_l", ctypes.c_byte),  # 工具数字输出7-0(仅bit0-bit1有效)
        ("cl_dgt_input_h", ctypes.c_byte),  # 数字输入15-8
        ("cl_dgt_input_l", ctypes.c_byte),  # 数字输入7-0
        ("tl_dgt_input_l", ctypes.c_byte),  # 工具数字输入7-0(仅bit0-bit1有效)
        ("cl_analog_input", ctypes.c_uint16 * 2),  # 控制箱模拟量输入
        ("tl_anglog_input", ctypes.c_uint16),  # 工具模拟量输入
        ("ft_sensor_raw_data", ctypes.c_double * 6),  # 力/扭矩传感器原始数据
        ("ft_sensor_data", ctypes.c_double * 6),  # 力/扭矩传感器数据
        ("ft_sensor_active", ctypes.c_byte),  # 力/扭矩传感器激活状态， 0-复位，1-激活
        ("EmergencyStop", ctypes.c_byte),  # 急停标志
        ("motion_done", ctypes.c_int),  # 到位信号
        ("gripper_motiondone", ctypes.c_byte),  # 夹爪运动完成信号
        ("mc_queue_len", ctypes.c_int),  # 运动队列长度
        ("collisionState", ctypes.c_byte),  # 碰撞检测，1-碰撞；0-无碰撞
        ("trajectory_pnum", ctypes.c_int),  # 轨迹点编号
        ("safety_stop0_state", ctypes.c_byte),  # 安全停止信号SI0
        ("safety_stop1_state", ctypes.c_byte),  # 安全停止信号SI1
        ("gripper_fault_id", ctypes.c_byte),  # 错误夹爪号
        ("gripper_fault", ctypes.c_uint16),  # 夹爪故障
        ("gripper_active", ctypes.c_uint16),  # 夹爪激活状态
        ("gripper_position", ctypes.c_byte),  # 夹爪位置
        ("gripper_speed", ctypes.c_byte),  # 夹爪速度
        ("gripper_current", ctypes.c_byte),  # 夹爪电流
        ("gripper_tmp", ctypes.c_int),  # 夹爪温度
        ("gripper_voltage", ctypes.c_int),  # 夹爪电压
        ("auxState", c_byte * 16),  # 485扩展轴状态
        ("extAxisStatus", c_byte * 64),  # UDP扩展轴状态
        ("extDIState", c_uint16 * 8),  # 扩展DI输入
        ("extDOState", c_uint16 * 8),  # 扩展DO输出
        ("extAIState", c_uint16 * 4),  # 扩展AI输入
        ("extAOState", c_uint16 * 4),  # 扩展AO输出
        ("rbtEnableState", ctypes.c_int),  # 机器人使能状态
        ("jointDriverTorque", ctypes.c_double * 6),  # 关节驱动器当前扭矩
        ("jointDriverTemperature", ctypes.c_double * 6),  # 关节驱动器当前温度
        ("year", ctypes.c_uint16),  # 年
        ("mouth", ctypes.c_uint8),  # 月
        ("day", ctypes.c_uint8),  # 日
        ("hour", ctypes.c_uint8),  # 小时
        ("minute", ctypes.c_uint8),  # 分
        ("second", ctypes.c_uint8),  # 秒
        ("millisecond", ctypes.c_uint16),  # 毫秒
        ("softwareUpgradeState", ctypes.c_int),  # 机器人软件升级状态
        ("endLuaErrCode", ctypes.c_uint16),  # 末端LUA运行状态
        ("check_sum", c_ushort)]  # 校验和


class RPC():
    ip_address = "192.168.58.2"
    log_output_model = -1
    queue = Queue(maxsize=10000 * 1024)
    is_conect = True
    ROBOT_REALTIME_PORT = 20004
    BUFFER_SIZE = 1024 * 8

    def __init__(self, ip="192.168.58.2"):
        self.ip_address = ip
        link = 'http://' + self.ip_address + ":20003"
        self.robot = xmlrpc.client.ServerProxy(link)  # xmlrpc连接机器人20003端口，用于发送机器人指令数据帧

        self.sock_cli_state = None
        self.robot_realstate_exit = False
        self.robot_state_pkg = RobotStatePkg  # 机器人状态数据

        self.stop_event = threading.Event()  # 停止事件
        thread = threading.Thread(target=self.robot_state_routine_thread)  # 创建线程循环接收机器人状态数据
        thread.daemon = True
        thread.start()
        time.sleep(1)

        try:
            # 调用 XML-RPC 方法
            socket.setdefaulttimeout(1)
            self.robot.GetControllerIP()
        except socket.timeout:
            print("XML-RPC connection timed out.")
            RPC.is_conect = False
        except socket.error as e:
            print("可能是网络故障，请检查网络连接。")
            RPC.is_conect = False
        except Exception as e:
            print("An error occurred during XML-RPC call:", e)
            RPC.is_conect = False
        finally:
            # 恢复默认超时时间
            self.robot = None
            socket.setdefaulttimeout(None)
            self.robot = xmlrpc.client.ServerProxy(link)

    def connect_to_robot(self):
        """连接到机器人的实时端口"""
        self.sock_cli_state = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # 套接字连接机器人20004端口，用于实时更新机器人状态数据
        try:
            self.sock_cli_state.connect((self.ip_address, self.ROBOT_REALTIME_PORT))
        except Exception as ex:
            print("SDK连接机器人实时端口失败", ex)
            return False
        return True

    def robot_state_routine_thread(self):
        """处理机器人状态数据包的线程例程"""

        while (1):
            recvbuf = bytearray(self.BUFFER_SIZE)
            tmp_recvbuf = bytearray(self.BUFFER_SIZE)
            state_pkg = bytearray(self.BUFFER_SIZE)
            find_head_flag = False
            index = 0
            length = 0
            tmp_len = 0
            if not self.connect_to_robot():
                return

            try:
                # while not self.robot_realstate_exit:
                while not self.robot_realstate_exit and not self.stop_event.is_set():
                    recvbyte = self.sock_cli_state.recv_into(recvbuf)
                    if recvbyte <= 0:
                        self.sock_cli_state.close()
                        print("接收机器人状态字节 -1")
                        return
                    else:
                        if tmp_len > 0:
                            if tmp_len + recvbyte <= self.BUFFER_SIZE:
                                recvbuf = tmp_recvbuf[:tmp_len] + recvbuf[:recvbyte]
                                recvbyte += tmp_len
                                tmp_len = 0
                            else:
                                tmp_len = 0

                        for i in range(recvbyte):
                            if format(recvbuf[i], '02X') == "5A" and not find_head_flag:
                                if i + 4 < recvbyte:
                                    if format(recvbuf[i + 1], '02X') == "5A":
                                        find_head_flag = True
                                        state_pkg[0] = recvbuf[i]
                                        index += 1
                                        length = length | recvbuf[i + 4]
                                        length = length << 8
                                        length = length | recvbuf[i + 3]
                                    else:
                                        continue
                                else:
                                    tmp_recvbuf[:recvbyte - i] = recvbuf[i:recvbyte]
                                    tmp_len = recvbyte - i
                                    break
                            elif find_head_flag and index < length + 5:
                                state_pkg[index] = recvbuf[i]
                                index += 1
                            elif find_head_flag and index >= length + 5:
                                if i + 1 < recvbyte:
                                    checksum = sum(state_pkg[:index])
                                    checkdata = 0
                                    checkdata = checkdata | recvbuf[i + 1]
                                    checkdata = checkdata << 8
                                    checkdata = checkdata | recvbuf[i]

                                    if checksum == checkdata:
                                        self.robot_state_pkg = RobotStatePkg.from_buffer_copy(recvbuf)
                                        find_head_flag = False
                                        index = 0
                                        length = 0
                                        i += 1
                                    else:
                                        find_head_flag = False
                                        index = 0
                                        length = 0
                                        i += 1
                                else:
                                    tmp_recvbuf[:recvbyte - i] = recvbuf[i:recvbyte]
                                    tmp_len = recvbyte - i
                                    break
                            else:
                                continue
            except Exception as ex:
                print("SDK读取机器人实时数据失败", ex)

    """   
    @brief  设置控制箱数字量输出
    @param  [in] 必选参数 id:io 编号，范围 [0~15]
    @param  [in] 必选参数 status:0-关，1-开
    @param  [in] 默认参数 smooth:0-不平滑，1-平滑 默认0
    @param  [in] 默认参数 block:0-阻塞，1-非阻塞 默认0
    @return 错误码 成功-0  失败-错误码
    """

    def SetDO(self, id, status, smooth=0, block=0):
        id = int(id)
        status = int(status)
        smooth = int(smooth)
        block = int(block)
        error = self.robot.SetDO(id, status, smooth, block)
        return error
    
    def GetDI(self, id, block=0):
        id = int(id)
        block = int(block)
        # _error = self.robot.GetDI(id, block)
        # error = _error[0]
        # print(_error)
        # if _error[0] == 0:
        #     di = _error[1]
        #     return error, di
        # else:
        #     return error
        if 0 <= id < 8:
            level = (self.robot_state_pkg.cl_dgt_input_l & (0x01 << id)) >> id
            return 0, level
        elif 8 <= id < 16:
            id -= 8
            level = (self.robot_state_pkg.cl_dgt_input_h & (0x01 << id)) >> id
            return 0, level
        else:
            return -1

def grip_open(robot):
    """
    气夹松开功能
    在松开前需要一直夹住，再调用松开
    DO1控制气夹松开
    """
    robot.SetDO(1, 1)  # DO1=1表示松开 
    print("############################气夹已松开")
    # """
    # 气夹松开功能
    # 在松开前需要一直夹住，再调用松开
    # DO1控制气夹松开
    # """
    # # 然后设置DO1为1（松开）
    # robot.SetDO(0, 0)
    # time.sleep(0.1)
    # robot.SetDO(1, 1)  # DO1=1表示松开
    # time.sleep(0.1)
    # print("############################气夹已松开")


def grip_clamp(robot):

    """
    气夹夹住功能
    DO0控制气夹夹住
    """
    # 设置DO0为1（夹住）
    robot.SetDO(1, 0)
    # time.sleep(0.1)
    robot.SetDO(0, 1)  # DO0=1表示夹住
    robot.SetDO(2, 1)
    print("气夹已夹住")
    # time.sleep(0.1)  # 短暂延时确保信号发送

def photoelectric_sensor(robot):
    err, state = robot.GetDI(4, 0)
    return state
    print("光电检测布料掉落")
    # time.sleep(0.1)     


def grip_release(robot):
    """
    气夹松开功能
    在松开前需要一直夹住，再调用松开
    DO1控制气夹松开
    """
    # 确保DO0为0（夹住状态）一段时间
    robot.SetDO(0, 0)  # 松开气夹
    robot.SetDO(2, 0)
    time.sleep(0.1)

    # 然后设置DO1为1（松开）
    robot.SetDO(1, 1)  # DO1=1表示松开
    print("气夹已松开")
    time.sleep(0.5)

    robot.SetDO(1, 0)

    time.sleep(0.1)

    # """
    # 气夹松开功能
    # 在松开前需要一直夹住，再调用松开
    # DO1控制气夹松开
    # """
    # # 然后设置DO1为1（松开）
    # robot.SetDO(1, 0)
    # time.sleep(0.1)
    # robot.SetDO(0, 0)  # DO1=1表示松开
    # print("############################气夹已夹住")
    # time.sleep(0.1)

def start_suction(robot):
    """
    吸气
    """
    robot.SetDO(3, 1)

def stop_suction(robot):
    """
    不吸气
    """
    robot.SetDO(3, 0)

def main():
    # 创建机器人连接实例
    robot = RPC("192.168.57.4")  # 使用你的机器人IP地址
    # stop_suction(robot)
    # while True:
    #     ret = photoelectric_sensor(robot)
    #     print(f"{ret}")
    #     time.sleep(0.2)

    # for i in range(1):
    #     grip_open(robot)
    #     time.sleep(5)
    #     # grip_clamp(robot)
    #     # time.sleep(5)
    #     grip_release(robot)
    #     time.sleep(5)
    
    grip_release(robot)

    # while True:
    #     grip_open(robot)

    # robot.SetDO(0, 0)
    # robot.SetDO(1, 0)
    # robot.SetDO(2, 0)


if __name__ == "__main__":
    main()



