#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot Control Center – 精简版
左侧按钮区已删除，全部功能按钮集中到顶部横向菜单栏。
"""
import tkinter.font as tkfont
# from ttkwidgets import HoverButton   # 需 pip install ttkwidgets
import os
import sys
import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext
import json
import subprocess
import time
import ast
import serial
import threading
from PIL import ImageTk, Image

# ==================== 核心配置参数 (来自jolly.py) ====================
class MotorController:
    """RS485 步进电机控制器类"""
    def __init__(self):
        self.ser = None
        self.SERIAL_PORT = "/dev/ttyUSB0"
        self.BAUD_RATE = 9600
        self.TIMEOUT = 0.1
        self.DRIVER_ADDR = 0x01
        self.PULSES_PER_MM = 200.0
        self.MOVE_RANGE_MIN = 1.0
        self.MOVE_RANGE_MAX = 238.0

        # 寄存器地址
        self.REG_SUB_DIV = 0x0011
        self.REG_CTRL_MODE = 0x0026
        self.REG_START_SPEED = 0x0020
        self.REG_ACCEL_TIME = 0x0021
        self.REG_DECEL_TIME = 0x0022
        self.REG_MAX_SPEED = 0x0023
        self.REG_CURRENT = 0x0010
        self.REG_ORIGIN_SIG = 0x0043
        self.REG_HOMING_MODE = 0x0031
        self.REG_HOMING_SPEED = 0x0032
        self.REG_HOMING_CRAWL = 0x0033
        self.REG_HOMING_ACCEL = 0x0034
        self.REG_ABS_POS_H = 0x0024
        self.REG_ABS_POS_L = 0x0025
        self.REG_RUN_CMD = 0x0027
        self.REG_HOMING_CMD = 0x0030
        self.REG_ESTOP_CMD = 0x0028

        self.connection_status = "未连接"
        self.current_position = 0.0

    def crc16_modbus(self, data: bytes) -> bytes:
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                crc = (crc >> 1) ^ 0xA001 if (crc & 0x0001) else crc >> 1
        return bytes([crc & 0xFF, (crc >> 8) & 0xFF])

    def modbus_write_reg(self, reg_addr: int, reg_value: int):
        if not self.ser or not self.ser.is_open:
            log_message("错误：串口未连接")
            return False
        try:
            frame = bytearray()
            frame.append(self.DRIVER_ADDR)
            frame.append(0x06)
            frame.extend(reg_addr.to_bytes(2, byteorder='big'))
            frame.extend(reg_value.to_bytes(2, byteorder='big'))
            frame.extend(self.crc16_modbus(frame))
            self.ser.write(frame)
            self.ser.flush()
            time.sleep(0.015)
            return True
        except Exception as e:
            log_message(f"485通信错误: {e}")
            return False

    def connect(self, port=None):
        if port:
            self.SERIAL_PORT = port
        try:
            self.ser = serial.Serial(port=self.SERIAL_PORT, baudrate=self.BAUD_RATE,
                                     parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
                                     bytesize=serial.EIGHTBITS, timeout=self.TIMEOUT)
            self.connection_status = "已连接"
            log_message(f"串口已连接: {self.SERIAL_PORT}")
            return True
        except Exception as e:
            self.connection_status = "连接失败"
            log_message(f"串口连接失败: {e}")
            return False

    def disconnect(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.connection_status = "未连接"
            log_message("串口已断开")

    def init_driver(self):
        if not self.ser or not self.ser.is_open:
            log_message("错误：请先连接串口")
            return
        log_message("正在初始化驱动器...")
        commands = [
            (self.REG_SUB_DIV, 0x0008), (self.REG_CTRL_MODE, 1),
            (self.REG_START_SPEED, 0x0064), (self.REG_ACCEL_TIME, 0x0064),
            (self.REG_DECEL_TIME, 0x0064), (self.REG_MAX_SPEED, 0x0320),
            (self.REG_CURRENT, 0x0006), (self.REG_ORIGIN_SIG, 0x0001),
            (self.REG_HOMING_MODE, 0x0001), (self.REG_HOMING_SPEED, 0x0320),
            (self.REG_HOMING_CRAWL, 0x001E), (self.REG_HOMING_ACCEL, 0x0064)
        ]
        for reg, value in commands:
            if not self.modbus_write_reg(reg, value):
                log_message("初始化失败")
                return
            time.sleep(0.05)
        log_message("驱动器初始化完成")

    def move_to_distance(self, distance_mm: float):
        if not self.ser or not self.ser.is_open:
            log_message("错误：请先连接串口")
            return
        if not (self.MOVE_RANGE_MIN <= distance_mm <= self.MOVE_RANGE_MAX):
            log_message(f"无效距离！范围: {self.MOVE_RANGE_MIN}-{self.MOVE_RANGE_MAX}mm")
            return
        target_pulses = int(round(distance_mm * self.PULSES_PER_MM))
        log_message(f"移动指令: {distance_mm:.1f}mm (脉冲: {target_pulses})")
        self.modbus_write_reg(self.REG_ABS_POS_H, (target_pulses >> 16) & 0xFFFF)
        time.sleep(0.01)
        self.modbus_write_reg(self.REG_ABS_POS_L, target_pulses & 0xFFFF)
        time.sleep(0.01)
        self.modbus_write_reg(self.REG_RUN_CMD, 5)
        self.current_position = distance_mm

    def homing(self):
        log_message("正在回原点...")
        self.modbus_write_reg(self.REG_HOMING_CMD, 1)
        self.current_position = 0.0

    def estop(self):
        log_message("急停！")
        self.modbus_write_reg(self.REG_ESTOP_CMD, 0x0001)


# ==================== 机器人库占位 ====================
try:
    from fairino2_8 import init_robot
    from control_air_close_open import grip_clamp, grip_open, grip_release
except ImportError:
    def init_robot(): return None
    def grip_clamp(*args): pass
    def grip_open(*args): pass
    def grip_release(*args): pass

# ==================== 全局变量 ====================
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
CONFIG_PATH = os.path.join(SCRIPT_DIR, "CONFIG.json")
current_process = None
robot = None
motor = MotorController()

# ==================== 工具函数 ====================
def load_config():
    try:
        with open(CONFIG_PATH, 'r', encoding='utf-8-sig') as f:
            return json.load(f)
    except FileNotFoundError:
        messagebox.showerror("文件缺失", f"未找到CONFIG.json文件：{CONFIG_PATH}")
        return {}
    except Exception as e:
        messagebox.showerror("加载失败", f"读取配置文件出错：{e}")
        return {}

def save_config(config_data):
    try:
        with open(CONFIG_PATH, 'w', encoding='utf-8') as f:
            json.dump(config_data, f, indent=4)
        log_message("配置已保存")
        return True
    except Exception as e:
        messagebox.showerror("保存失败", f"写入CONFIG.json出错：{e}")
        return False

def log_message(message):
    timestamp = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
    log_text.insert(tk.END, f"[{timestamp}] {message}\n")
    log_text.see(tk.END)
    root.update_idletasks()

# ==================== 进程管理 ====================
def stop_subprocess():
    global current_process
    if current_process and current_process.poll() is None:
        log_message("正在终止后台进程...")
        try:
            current_process.terminate()
            current_process.wait(timeout=2)
        except subprocess.TimeoutExpired:
            current_process.kill()
        log_message("后台进程已结束。")
        current_process = None

def start_main_tmp():
    global current_process
    if current_process and current_process.poll() is None:
        messagebox.showwarning("警告", "程序已经在运行中，请勿重复启动！")
        return
    try:
        main_tmp_path = os.path.join(SCRIPT_DIR, "main.py")
        if not os.path.exists(main_tmp_path):
            messagebox.showerror("文件缺失", f"未找到主程序文件:\n{main_tmp_path}")
            return
        flags = subprocess.CREATE_NO_WINDOW if sys.platform == "win32" else 0
        current_process = subprocess.Popen([sys.executable, main_tmp_path], encoding='utf-8', creationflags=flags)
        log_message(f"成功启动 main.py (PID: {current_process.pid})")
    except Exception as e:
        messagebox.showerror("启动失败", f"无法启动 main.py:\n{e}")
        current_process = None

# ==================== 电机控制 ====================
def update_motor_status():
    status_label.config(text=f"状态: {motor.connection_status}")
    root.after(1000, update_motor_status)

def connect_motor():
    port = port_entry.get().strip() or "/dev/ttyUSB0"
    threading.Thread(target=lambda: motor.connect(port) and connect_btn.config(state=tk.DISABLED) or disconnect_btn.config(state=tk.NORMAL), daemon=True).start()

def disconnect_motor():
    motor.disconnect()
    connect_btn.config(state=tk.NORMAL)
    disconnect_btn.config(state=tk.DISABLED)

def init_motor():
    threading.Thread(target=motor.init_driver, daemon=True).start()

def motor_homing():
    threading.Thread(target=motor.homing, daemon=True).start()

def motor_estop():
    threading.Thread(target=motor.estop, daemon=True).start()

def motor_move():
    try:
        pos = float(position_entry.get())
        threading.Thread(target=motor.move_to_distance, args=(pos,), daemon=True).start()
    except ValueError:
        messagebox.showerror("错误", "请输入有效的数字位置")

# ==================== 参数设置窗口 ====================
def open_param_settings():
    config = load_config()
    if not config:
        return
    top = tk.Toplevel(root)
    top.title("CONFIG.json 参数设置")
    top.geometry("1200x800")
    top.transient(root)
    top.grab_set()
    top.configure(bg='#f0f5f9')
    canvas = tk.Canvas(top, bg='#f0f5f9', highlightthickness=0)
    scroll = ttk.Scrollbar(top, orient="vertical", command=canvas.yview)
    frame = tk.Frame(canvas, bg='#f0f5f9')
    frame.bind("<Configure>", lambda e: canvas.configure(scrollregion=canvas.bbox("all")))
    canvas.create_window((0, 0), window=frame, anchor='nw')
    canvas.configure(yscrollcommand=scroll.set)
    canvas.pack(side="left", fill="both", expand=True)
    scroll.pack(side="right", fill="y")

    widgets = {}
    row = 0

    def build(parent, data, parent_key="", indent=0):
        nonlocal row
        for k, v in sorted(data.items()):
            full = f"{parent_key}.{k}" if parent_key else k
            if isinstance(v, dict):
                tk.Label(parent, text="  "*indent + f"[{k}]", font=('DejaVu Sans', 12, 'bold'), bg='#f0f5f9', fg='#2c3e50').grid(row=row, column=0, columnspan=2, sticky='w', padx=10, pady=(10,5))
                row += 1
                build(parent, v, full, indent+1)
            else:
                tk.Label(parent, text="  "*indent + f"{k}:", font=('DejaVu Sans', 10), bg='#f0f5f9', anchor='w', width=50).grid(row=row, column=0, sticky='w', padx=10, pady=5)
                if isinstance(v, (list, dict, tuple)):
                    t = tk.Text(parent, height=4, width=80, font=('DejaVu Sans Mono', 10))
                    t.insert('1.0', json.dumps(v, indent=2))
                    t.grid(row=row, column=1, padx=10, pady=5, sticky='ew')
                    widgets[full] = (t, type(v), k, parent_key)
                else:
                    e = ttk.Entry(parent, width=80, font=('DejaVu Sans', 10))
                    e.insert(0, str(v))
                    e.grid(row=row, column=1, padx=10, pady=5, sticky='ew')
                    widgets[full] = (e, type(v), k, parent_key)
                row += 1
        parent.columnconfigure(1, weight=1)
    build(frame, config)

    def save():
        try:
            new_config = json.loads(json.dumps(config))
            for full, (w, tp, k, pk) in widgets.items():
                raw = w.get('1.0', tk.END).strip() if isinstance(w, tk.Text) else w.get().strip()
                if not raw:
                    continue
                val = json.loads(raw) if isinstance(w, tk.Text) else (tp(raw) if tp != bool else ast.literal_eval(raw.capitalize()))
                if pk:
                    d = new_config
                    for step in pk.split('.'):
                        d = d[step]
                    d[k] = val
                else:
                    new_config[k] = val
            if save_config(new_config):
                messagebox.showinfo("Success", "CONFIG.json updated! Restart to take effect.")
                top.destroy()
        except Exception as e:
            messagebox.showerror("Save Failed", str(e))
    btn_frame = tk.Frame(top, bg='#f0f5f9')
    btn_frame.pack(side=tk.BOTTOM, pady=20)
    ttk.Button(btn_frame, text="Save Configuration", command=save, style='Accent.TButton').pack(side=tk.LEFT, padx=10)
    ttk.Button(btn_frame, text="Cancel", command=top.destroy).pack(side=tk.LEFT, padx=10)

# ==================== 机器人/夹爪 ====================
def clamp_gripper():
    grip_clamp(robot) if robot else log_message("错误：机器人未初始化")

def open_gripper():
    grip_open(robot) if robot else log_message("错误：机器人未初始化")

def release_gripper():
    grip_release(robot) if robot else log_message("错误：机器人未初始化")

# ==================== 关闭系统 ====================
def close_system():
    if messagebox.askyesno("Confirm Close", "确定要关闭系统吗？\n(将同时终止后台运行的任务)"):
        stop_subprocess()
        motor.disconnect()
        log_message("系统已关闭")
        root.quit()
        root.destroy()

# ==================== 顶部菜单栏 ====================
def create_module_dropdown_panel(parent):
    """返回横向滚动的按钮容器 frame（已 pack 到 parent 顶部）"""
    outer = tk.LabelFrame(parent, text="功能快捷入口", font=('DejaVu Sans', 11, 'bold'),
                          bg='#ffffff', fg='#2c3e50', padx=10, pady=10)
    outer.pack(fill=tk.X, padx=20, pady=(10, 0))

    canvas = tk.Canvas(outer, height=200, bg='#ffffff', highlightthickness=0)
    scroll = ttk.Scrollbar(outer, orient=tk.HORIZONTAL, command=canvas.xview)
    canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
    scroll.pack(side=tk.BOTTOM, fill=tk.X)
    canvas.configure(xscrollcommand=scroll.set)

    btn_frame = tk.Frame(canvas, bg='#ffffff')
    canvas.create_window((0, 0), window=btn_frame, anchor='nw')
    btn_frame.bind('<Configure>', lambda e: canvas.configure(scrollregion=canvas.bbox('all')))
    # 横向滚轮
    canvas.bind_all('<Shift-MouseWheel>', lambda e: canvas.xview_scroll(int(-1*(e.delta/120)), 'units'))
    return btn_frame
# ====================  参数解释按钮  ====================


def open_param_explain_chinese():
    """弹出“参数中文解释”——带搜索、树形分级、一键复制"""
    top = tk.Toplevel(root)
    top.title('参数中文解释')
    top.geometry('800x700')
    top.minsize(700, 500)
    top.configure(bg='#fafafa')
    top.transient(root)
    top.grab_set()

    # ---- 搜索栏 ----
    search_frame = tk.Frame(top, bg='#fafafa')
    search_frame.pack(fill=tk.X, padx=15, pady=10)
    tk.Label(search_frame, text='搜索：', bg='#fafafa').pack(side=tk.LEFT)
    search_var = tk.StringVar()
    search_entry = ttk.Entry(search_frame, textvariable=search_var, width=30)
    search_entry.pack(side=tk.LEFT, padx=5)
    ttk.Button(search_frame, text='清除',
               command=lambda: search_var.set('')).pack(side=tk.LEFT, padx=5)

    # ---- 树形表格 ----
    columns = ('参数', '类型', '范围/示例', '中文说明')
    tree = ttk.Treeview(top, columns=columns, show='tree headings', height=20)
    vsb = ttk.Scrollbar(top, orient=tk.VERTICAL, command=tree.yview)
    hsb = ttk.Scrollbar(top, orient=tk.HORIZONTAL, command=tree.xview)
    tree.configure(yscrollcommand=vsb.set, xscrollcommand=hsb.set)

    # 表头
    for col in columns:
        tree.heading(col, text=col, anchor='w')
        tree.column(col, width=180 if col == '中文说明' else 120, anchor='w')

    # 数据（分级：根 → 模块 → 参数）
    data = {
        '根级别': {
            'robot_mode':                    ('int',  '0/1',        '机器人运行模式（0=仿真，1=真机）'),
            'DIRECTION_1':                   ('list', '[1,0,0]',    '机械臂方向向量1，用于坐标转换'),
            'DIRECTION_2':                   ('list', '[0,1,0]',    '机械臂方向向量2'),
            'place_pos_arm_1':               ('list', '[x,y,z,rx,ry,rz]', '放置目标点1，单位mm/°'),
            'place_pos_arm_2':               ('list', '[x,y,z,rx,ry,rz]', '放置目标点2'),
            'TASK_EXPIRATION_THRESHOLD':     ('int',  '≥1（秒）',   '任务过期阈值'),
            'edge_params':                   ('dict', '见子项',      '边缘检测收缩/外扩比例'),
        },
        'CONFIG_PARAMS_VISION': {
            'real_w':                        ('int',  '≥10（cm）',  '现实场景宽度'),
            'real_h':                        ('int',  '≥10（cm）',  '现实场景高度'),
            'ratio_wh':                      ('float','>0',         '输出图像宽高比'),
            'aruco_ids_corner':              ('dict', '{0:1,1:2...}', 'ArUco角点ID映射'),
            'speed_smooth_frames':           ('int',  '≥1',         '速度平滑采样帧数'),
            'similarity_threshold':          ('float','0~1',        '特征匹配门限'),
            'diff_threshold':                ('int',  '0~255',      '背景差分门限'),
            'morph_kernel':                  ('int',  '奇数≥3',     '形态学核大小'),
            'morph_iter':                    ('int',  '≥1',         '形态学迭代次数'),
            'min_contour_area':              ('int',  '≥0',         '最小有效轮廓面积'),
            'max_distance':                  ('float','≥0（px）',   '目标跟踪最大距离'),
        },
        'CONFIG_PARAMS_ARM': {
            'speed':                         ('float','>0（cm/s）', '机械臂速度'),
            'accel':                         ('int',  '≥0（ms）',   '加减速时间'),
            'safe_pos_list':                 ('list', '[[x,y,z]..]','安全点坐标列表'),
            'stand_z':                       ('float','mm',         '标准高度'),
            'stand_rx':                      ('float','°',          '标准姿态Rx'),
            'stand_ry':                      ('float','°',          '标准姿态Ry'),
            'stand_rz':                      ('float','°',          '标准姿态Rz'),
            'blend_alpha':                   ('float','0~1',        '图像融合透明度'),
            'is_real_sense':                 ('bool', 'true/false', '是否使用RealSense'),
            'camera_no':                     ('int',  '≥0',         '摄像头索引'),
            'points_file_path':              ('str',  '绝对/相对路径', '标定坐标文件'),
            'down_height':                   ('float','≥0（mm）',   '抓取下降高度'),
            'WAIT_DISTANCE':                 ('float','≥0（mm）',   '进入等待区阈值'),
        },
        'edge_params': {
            'edge_left_in_ratio':            ('float','0~1',        '左边缘向内收缩比例'),
            'edge_left_out_ratio':           ('float','0~1',        '左边缘向外扩展比例'),
            'edge_right_in_ratio':           ('float','0~1',        '右边缘向内收缩比例'),
            'edge_right_out_ratio':          ('float','0~1',        '右边缘向外扩展比例'),
            'edge_top_in_ratio':             ('float','0~1',        '上边缘向内收缩比例'),
            'edge_top_out_ratio':            ('float','0~1',        '上边缘向外扩展比例'),
            'edge_bottom_in_ratio':          ('float','0~1',        '下边缘向内收缩比例'),
            'edge_bottom_out_ratio':         ('float','0~1',        '下边缘向外扩展比例'),
        }
    }

    # 插入树形节点
    def insert(parent, key, tp, rng, desc):
        tree.insert(parent, 'end', text=key, values=(key, tp, rng, desc))

    for module, params in data.items():
        mid = tree.insert('', 'end', text=module, values=(module, '', '', ''))
        for k, (tp, rng, desc) in params.items():
            insert(mid, k, tp, rng, desc)

    tree.pack(fill=tk.BOTH, expand=True, padx=15, pady=5)
    vsb.pack(side=tk.RIGHT, fill=tk.Y)
    hsb.pack(side=tk.BOTTOM, fill=tk.X)

    # ---- 搜索高亮逻辑 ----
    def search(*_):
        s = search_var.get().strip().lower()
        for iid in tree.get_children():
            tree.item(iid, open=s in tree.item(iid, 'text').lower())
            for cid in tree.get_children(iid):
                vals = tree.item(cid, 'values')
                hit = any(s in str(v).lower() for v in vals)
                tree.detach(cid) if not hit else tree.reattach(cid, iid, 'end')
    search_var.trace_add('write', search)

    # ---- 右键复制 ----
    def copy_cell(event):
        row = tree.identify_row(event.y)
        col = tree.identify_column(event.x)
        if row and col:
            val = tree.set(row, col)
            top.clipboard_clear()
            top.clipboard_append(val)
            top.update()
    tree.bind('<Button-3>', copy_cell)

    # ---- 底部按钮 ----
    btn_bar = tk.Frame(top, bg='#fafafa')
    btn_bar.pack(side=tk.BOTTOM, fill=tk.X, pady=10)
    ttk.Button(btn_bar, text='展开全部',
               command=lambda: [tree.item(i, open=True) for i in tree.get_children()]).pack(side=tk.LEFT, padx=15)
    ttk.Button(btn_bar, text='收起全部',
               command=lambda: [tree.item(i, open=False) for i in tree.get_children()]).pack(side=tk.LEFT, padx=5)
    ttk.Button(btn_bar, text='关闭', command=top.destroy).pack(side=tk.RIGHT, padx=15)


def populate_module_buttons(master):
    modules = {
        '机械臂': [
            ('参数配置', open_param_settings),
            ('启动主程序', start_main_tmp),
            ('参数解释', open_param_explain_chinese),
            ('关闭系统', close_system),
        ],
        '电机/丝杆': [
            ('连接电机', connect_motor),
            ('初始化驱动器', init_motor),
            ('电机回原点', motor_homing),
            ('电机急停', motor_estop),
        ],
        '夹爪': [
            ('夹紧夹爪', clamp_gripper),
            ('松开夹爪', open_gripper),
            ('释放夹爪', release_gripper),
        ],
    }
    for module, btn_list in modules.items():
        lbl = tk.Label(master, text=module, font=(FONT_FAMILY, 11, 'bold'),
                       bg=COL_CARD, fg=COL_ACCENT, pady=8, padx=15)
        lbl.pack(side=tk.LEFT, fill=tk.Y, padx=(20, 5))
        for txt, cmd in btn_list:
            ttk.Button(master, text=txt, command=cmd, style='Flat.TButton').pack(side=tk.LEFT, padx=4)
        ttk.Separator(master, orient=tk.VERTICAL).pack(side=tk.LEFT, fill=tk.Y, padx=10, pady=8)




# ==================== 主界面 ====================
if __name__ == "__main__":
    try:
        robot = init_robot()
    except Exception as e:
        print(f"机器人初始化失败: {e}")
        robot = None
    

        # ---------- Ubuntu 字体映射 ----------
    # ---------- 配色 ----------
    COL_BG     = '#F7F7F7'        # 窗口背景
    COL_CARD   = '#FFFFFF'        # 卡片背景
    COL_ACCENT = '#E95420'        # Ubuntu 橙
    COL_GRAY   = '#77216F'        # Ubuntu 紫灰（用于文字）
    COL_BORDER = '#DFDFDF'        # 边框淡灰


    root = tk.Tk()
    root.title('Robot Control Center')
    root.geometry('1200x800')
    root.minsize(1100, 700)
    root.configure(bg=COL_BG)
    root.resizable(True, True)
    FONT_FAMILY = 'Ubuntu' if 'Ubuntu' in tkfont.families() else 'Noto Sans'
    FONT_MONO   = 'Ubuntu Mono' if 'Ubuntu Mono' in tkfont.families() else 'Noto Sans Mono'


    # ---------- ttk 主题 ----------
    style = ttk.Style()
    style.theme_use('clam')
    # 全局统一字体
    style.configure('.', font=(FONT_FAMILY, 11), background=COL_BG, foreground=COL_GRAY)
    # 按钮扁平 + 圆角 + 悬停
    style.layout('Flat.TButton',
                [('Button.button', {'sticky': 'nswe', 'border': '1', 'children':
                    [('Button.padding', {'sticky': 'nswe', 'children':
                        [('Button.label', {'sticky': 'nswe'})]})]})])
    style.configure('Flat.TButton', relief='flat', borderwidth=1, focusthickness=0,
                    background=COL_CARD, foreground=COL_GRAY, padding=(10, 6))
    style.map('Flat.TButton',
            background=[('hover', COL_ACCENT), ('active', COL_ACCENT)],
            foreground=[('hover', 'white'), ('active', 'white')])
    # 日志区
    style.configure('Log.TFrame', background=COL_CARD, relief='flat', borderwidth=0)
    # 树形图（参数解释窗口）
    style.configure('Treeview', rowheight=28, font=(FONT_FAMILY, 10))
    style.configure('Treeview.Heading', font=(FONT_FAMILY, 10, 'bold'))

    # 主题/字体
    style = ttk.Style()
    style.theme_use('clam')
    style.configure('TButton', font=('DejaVu Sans', 11), padding=6)
    style.configure('Accent.TButton', font=('DejaVu Sans', 11, 'bold'), padding=6)
    style.configure('TLabel', font=('DejaVu Sans', 11), foreground='#2c3e50', background='#f0f5f9')
    style.configure('TEntry', font=('DejaVu Sans', 10))
    style.configure('TFrame', background='#f0f5f9')
    style.configure('TLabelframe', background='#ffffff', foreground='#2c3e50')
    style.configure('TLabelframe.Label', background='#ffffff', foreground='#2c3e50', font=('DejaVu Sans', 11, 'bold'))

    # —————— ① 顶部菜单栏 ——————
    top_menu = create_module_dropdown_panel(root)
    populate_module_buttons(top_menu)

    # —————— ② 下方日志区占满剩余空间 ——————
    log_frame = tk.Frame(root, bg='#ffffff', relief=tk.RAISED, bd=1)
    log_frame.pack(fill=tk.BOTH, expand=True, padx=20, pady=(10, 20))

    tk.Label(log_frame, text="System Log", font=('DejaVu Sans', 12, 'bold'),
             bg='#ffffff', fg='#2c3e50').pack(pady=15, padx=20, anchor="w")

    log_text = scrolledtext.ScrolledText(log_frame, font=('DejaVu Sans Mono', 10), bg='#fafafa',
                                         relief=tk.SOLID, bd=1, wrap=tk.WORD, height=20)
    log_text.pack(fill=tk.BOTH, expand=True, padx=20, pady=(0, 20))

    # 状态栏（底部）
    status_label = tk.Label(root, text="状态: 未连接", font=('DejaVu Sans', 10), fg='#e74c3c', bg='#f0f5f9')
    status_label.pack(side=tk.BOTTOM, anchor='w', padx=20, pady=5)

    # 端口输入（隐藏，供电机连接函数使用）
    port_entry = tk.Entry(root)
    port_entry.insert(0, "/dev/ttyUSB0")
    port_entry.pack_forget()

    # 连接/断开/初始化按钮（隐藏，供回调使用）
    connect_btn = ttk.Button(root)
    disconnect_btn = ttk.Button(root)
    init_motor_btn = ttk.Button(root)
    position_entry = tk.Entry(root)
    position_entry.insert(0, "50.0")

    # 关闭事件
    root.protocol("WM_DELETE_WINDOW", close_system)

    # 启动状态刷新
    update_motor_status()

    log_message("System startup complete, waiting for commands...")
    log_message("Hint: Please connect serial port and initialize driver first")

    root.mainloop()