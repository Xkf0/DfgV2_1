#!/usr/bin/env python3
import json
import time
import cv2
import numpy as np
import threading
import datetime
import sys

from module_speeddect import SpeedMonitor
speed_now = 0
speed_mode = 1
time_pre_now = 0.0  # 定义为全局变量

def process_tasks_speed(monitor):
    """
    处理实时获取速度模式
    """
    global time_pre_now, speed_now
    time.sleep(1)  # 初始等待数据
    while True:
        speed = -monitor.speed_now*100
        if speed > 0:
            speed_now = speed
            time_pre_now = 7 / speed_now
        else:
            speed_now=speed_now
            time_pre_now = 10.0  # 默认值，避免除零
        print(f"当前实时平均速度: {speed:.4f} cm/s")
        print(f"当前实时预抓取时间: {time_pre_now:.4f} s")
        time.sleep(0.1)  # 刷新间隔，调整根据需要

def main():
    global time_pre_now
    time_pre_now = 0.0
    
    if speed_mode == 1:
        monitor = SpeedMonitor(enable_plot=True)  # 根据需要配置
        monitor.start()
        threading.Thread(target=process_tasks_speed, args=(monitor,), daemon=True).start()
    else:
        speed_now = 10
        time_pre_now = 11

    print("开始主循环...")
    sum_detect = 0
    try:
        while True:
            print("aaa")
            if speed_mode == 1:
                print("now_speed:", monitor.speed_now)
            else:
                print("now_speed:", speed_now)
            print("time_pre_now:", time_pre_now)
            time.sleep(1)  # 添加sleep避免CPU占用过高
    except KeyboardInterrupt:
        print("\n收到中断信号")
    finally:
        if speed_mode == 1:
            monitor.stop()
        print("程序退出")

if __name__ == "__main__":
    main()
