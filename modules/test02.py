# -*- coding: utf-8 -*-
"""
测速 + 自适应滤波 + 滤波后速度的滑动平均 + 实时绘图 + 数值显示
"""

import time
from collections import deque

import serial
import matplotlib.pyplot as plt


# -----------------------------
# Modbus RTU helpers (CRC / build req / robust read)
# -----------------------------
def modbus_crc16(data: bytes) -> bytes:
    """Modbus RTU CRC16, return 2 bytes little-endian (low, high)."""
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if (crc & 1) else (crc >> 1)
    return crc.to_bytes(2, "little", signed=False)


def build_read_holding_registers_req(slave_id: int, start_addr: int, reg_count: int) -> bytes:
    pdu = bytes([
        slave_id & 0xFF,
        0x03,
        (start_addr >> 8) & 0xFF,
        start_addr & 0xFF,
        (reg_count >> 8) & 0xFF,
        reg_count & 0xFF,
    ])
    return pdu + modbus_crc16(pdu)


def read_modbus_int32(
    ser: serial.Serial,
    slave_id: int,
    start_addr: int,
    word_count: int = 2,
    timeout_s: float = 0.25,
    signed: bool = True,
    word_swap: bool = False,
    tolerate_no_crc_7bytes: bool = True,
) -> int | None:
    """
    Read 2 holding registers (4 bytes) and parse as int32.
    Standard response: 9 bytes: [id][03][04][data4][crc2]
    Some devices/bridges may return 7 bytes without CRC (tolerate if enabled).
    """
    req = build_read_holding_registers_req(slave_id, start_addr, word_count)

    # 不要每轮 reset_input_buffer()，否则可能把刚到的响应清掉
    ser.write(req)
    ser.flush()

    deadline = time.monotonic() + timeout_s
    buf = bytearray()

    while time.monotonic() < deadline:
        n = ser.in_waiting
        if n:
            buf.extend(ser.read(n))
            if len(buf) >= 9:
                break
            if tolerate_no_crc_7bytes and len(buf) >= 7:
                # 等一小会，看会不会补上 CRC
                time.sleep(0.01)
        else:
            time.sleep(0.002)

    if len(buf) >= 9:
        frame = bytes(buf[:9])
        rid, func, bc = frame[0], frame[1], frame[2]
        if rid != (slave_id & 0xFF) or func != 0x03 or bc != 0x04:
            return None
        data = frame[3:7]
        crc_recv = frame[7:9]
        if modbus_crc16(frame[:7]) != crc_recv:
            return None
    elif tolerate_no_crc_7bytes and len(buf) >= 7:
        frame = bytes(buf[:7])
        rid, func, bc = frame[0], frame[1], frame[2]
        if rid != (slave_id & 0xFF) or func != 0x03 or bc != 0x04:
            return None
        data = frame[3:7]
    else:
        return None

    if word_swap:
        data = data[2:4] + data[0:2]

    return int.from_bytes(data, byteorder="big", signed=signed)


# -----------------------------
# Speed computing & filtering
# -----------------------------
def calculate_linear_speed(encoder_value: int, resolution: int, sample_time_ms: float, perimeter_mm: float) -> float:
    sample_time_s = sample_time_ms / 1000.0
    if sample_time_s <= 0:
        return 0.0
    rps = (encoder_value / float(resolution)) / sample_time_s
    perimeter_m = perimeter_mm / 1000.0
    return rps * perimeter_m


class AdaptiveEMAFilter:
    """
    自适应指数滑动平均：
    - 正常波动：base_alpha 平滑
    - 突变：fast_alpha 快速跟随（或 hard_jump 直接跳变）
    """
    def __init__(self, base_alpha=0.15, fast_alpha=0.75, jump_threshold=0.02, hard_jump=False):
        self.base_alpha = float(base_alpha)
        self.fast_alpha = float(fast_alpha)
        self.jump_threshold = float(jump_threshold)
        self.hard_jump = bool(hard_jump)
        self.y = None

    def update(self, x: float) -> float:
        if self.y is None:
            self.y = x
            return self.y
        delta = abs(x - self.y)
        if delta >= self.jump_threshold:
            if self.hard_jump:
                self.y = x
            else:
                a = self.fast_alpha
                self.y = self.y + a * (x - self.y)
        else:
            a = self.base_alpha
            self.y = self.y + a * (x - self.y)
        return self.y


# -----------------------------
# Plotting (with rolling mean + numeric text)
# -----------------------------
class RealtimePlot:
    def __init__(
        self,
        window_seconds: float = 15.0,
        mean_mode: str = "points",      # "points" or "seconds"
        mean_points: int = 50,          # 最近 N 点
        mean_seconds: float = 2.0,      # 最近 T 秒（mean_mode="seconds"时生效）
    ):
        self.window_seconds = float(window_seconds)
        self.mean_mode = mean_mode
        self.mean_points = int(mean_points)
        self.mean_seconds = float(mean_seconds)

        self.t_buf = deque()
        self.raw_buf = deque()
        self.filt_buf = deque()

        # 为 rolling mean 单独维护一个缓冲（更快）
        self.mean_t_buf = deque()
        self.mean_f_buf = deque()

        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.line_raw, = self.ax.plot([], [], label="raw")
        self.line_filt, = self.ax.plot([], [], label="filtered")
        self.line_mean, = self.ax.plot([], [], label="filtered_mean")

        self.ax.set_xlabel("time (s)")
        self.ax.set_ylabel("speed (m/s)")
        self.ax.grid(True)
        self.ax.legend(loc="upper right")

        # 数值显示（右上角/左上角都行）
        self.text_box = self.ax.text(
            0.02, 0.98, "",
            transform=self.ax.transAxes,
            va="top", ha="left",
            fontsize=10,
            bbox=dict(boxstyle="round", alpha=0.2)
        )

        self.t0 = time.monotonic()
        self.last_raw = 0.0
        self.last_filt = 0.0
        self.last_mean = 0.0

    def _update_mean_buffer(self, t: float, filt: float):
        self.mean_t_buf.append(t)
        self.mean_f_buf.append(filt)

        if self.mean_mode == "points":
            while len(self.mean_f_buf) > self.mean_points:
                self.mean_t_buf.popleft()
                self.mean_f_buf.popleft()
        else:
            # seconds
            while self.mean_t_buf and (self.mean_t_buf[-1] - self.mean_t_buf[0] > self.mean_seconds):
                self.mean_t_buf.popleft()
                self.mean_f_buf.popleft()

    def _current_mean(self) -> float:
        if not self.mean_f_buf:
            return self.last_mean
        return sum(self.mean_f_buf) / len(self.mean_f_buf)

    def push(self, t_now: float, raw: float, filt: float):
        t = t_now - self.t0
        self.t_buf.append(t)
        self.raw_buf.append(raw)
        self.filt_buf.append(filt)

        self.last_raw = raw
        self.last_filt = filt

        # 更新 rolling mean
        self._update_mean_buffer(t, filt)
        self.last_mean = self._current_mean()

        # 滑动窗口裁剪（显示用）
        while self.t_buf and (self.t_buf[-1] - self.t_buf[0] > self.window_seconds):
            self.t_buf.popleft()
            self.raw_buf.popleft()
            self.filt_buf.popleft()

    def refresh(self):
        if not self.t_buf:
            return

        t = list(self.t_buf)
        raw = list(self.raw_buf)
        filt = list(self.filt_buf)

        # rolling mean 画成“水平线”或者“曲线”
        # 这里做成随时间更新的“均值曲线”（每个点对应当前均值）
        mean_curve = [self.last_mean] * len(t)

        self.line_raw.set_data(t, raw)
        self.line_filt.set_data(t, filt)
        self.line_mean.set_data(t, mean_curve)

        # x 轴范围
        self.ax.set_xlim(t[0], max(t[-1], t[0] + 1e-3))

        # y 轴范围
        y_min = min(min(raw), min(filt), min(mean_curve))
        y_max = max(max(raw), max(filt), max(mean_curve))
        if y_max - y_min < 1e-6:
            y_max = y_min + 1e-3
        pad = 0.12 * (y_max - y_min)
        self.ax.set_ylim(y_min - pad, y_max + pad)

        # 更新数值显示
        mode_txt = f"N={self.mean_points}" if self.mean_mode == "points" else f"T={self.mean_seconds:.1f}s"
        self.text_box.set_text(
            f"raw:     {self.last_raw:.4f} m/s\n"
            f"filtered:{self.last_filt:.4f} m/s\n"
            f"mean({mode_txt}): {self.last_mean:.4f} m/s"
        )

        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()


# -----------------------------
# Main
# -----------------------------
def main():
    SERIAL_PORT = "/dev/ttyUSB1"
    BAUD_RATE = 9600

    # 常见Modbus设备是 8E1（偶校验），如果你确定是 8N1 再改成 NONE
    PARITY = serial.PARITY_NONE  # serial.PARITY_EVEN / serial.PARITY_NONE

    SLAVE_ID = 0x01
    START_ADDR = 0x0020
    WORD_COUNT = 2

    RESOLUTION = 131072
    SAMPLE_TIME_MS = 100.0
    PERIMETER_MM = 200.0

    # 建议采样周期不要小于设备速度计算窗口（你这里是100ms）
    READ_PERIOD_S = 0.10

    # 绘图刷新周期（不要每次采样都刷新）
    PLOT_REFRESH_PERIOD_S = 0.10

    # 滤波参数：阈值别太小，否则会频繁判“突变”
    SPEED_JUMP_THRESHOLD = 0.02
    filt = AdaptiveEMAFilter(base_alpha=0.15, fast_alpha=0.75, jump_threshold=SPEED_JUMP_THRESHOLD, hard_jump=False)

    # 平均值：默认按最近 N 点
    plotter = RealtimePlot(
        window_seconds=15.0,
        mean_mode="points",   # "points" or "seconds"
        mean_points=50,       # 最近50个滤波点的平均值
        mean_seconds=2.0,
    )

    last_plot_t = 0.0
    print("Starting... Ctrl+C to stop.")

    try:
        with serial.Serial(
            SERIAL_PORT,
            BAUD_RATE,
            bytesize=8,
            parity=PARITY,
            stopbits=1,
            timeout=0.2,
            write_timeout=0.2,
        ) as ser:
            time.sleep(0.05)

            while True:
                t0 = time.monotonic()

                encoder_value = read_modbus_int32(
                    ser,
                    slave_id=SLAVE_ID,
                    start_addr=START_ADDR,
                    word_count=WORD_COUNT,
                    timeout_s=0.25,
                    signed=True,
                    word_swap=False,
                    tolerate_no_crc_7bytes=True,
                )

                if encoder_value is not None:
                    raw_speed = calculate_linear_speed(encoder_value, RESOLUTION, SAMPLE_TIME_MS, PERIMETER_MM)
                    filtered_speed = filt.update(raw_speed)

                    plotter.push(t0, raw_speed, filtered_speed)
                    print(
                        f"enc={encoder_value:>12d} | raw={raw_speed:>8.4f} m/s "
                        f"| filt={filtered_speed:>8.4f} m/s | mean={plotter.last_mean:>8.4f} m/s"
                    )

                    if (t0 - last_plot_t) >= PLOT_REFRESH_PERIOD_S:
                        plotter.refresh()
                        last_plot_t = t0
                else:
                    print("未读取到有效响应")

                elapsed = time.monotonic() - t0
                sleep_s = READ_PERIOD_S - elapsed
                if sleep_s > 0:
                    time.sleep(sleep_s)

    except KeyboardInterrupt:
        print("\nStopped by user.")
    except serial.SerialException as e:
        print(f"串口通信错误: {e}")
    except Exception as e:
        print(f"发生错误: {e}")


if __name__ == "__main__":
    main()

