from __future__ import annotations

import argparse
import sys
import time
from typing import Callable

from host_ui.actuator_client import LinearActuatorClient


DEFAULT_PORT = "/dev/ttyUSB0"


def run_self_test(use_mock: bool = True) -> int:
    client = LinearActuatorClient.mock() if use_mock else LinearActuatorClient.from_serial(DEFAULT_PORT)
    client.connect()
    client.init_driver(inter_command_delay_s=0.0)
    client.home()
    pulses = client.move_to(120.5)
    client.estop()
    print(f"[self-test] ok, move_to pulses={pulses}, connected={client.is_connected()}")
    client.disconnect()
    return 0


def _build_client(use_mock: bool, port: str) -> LinearActuatorClient:
    if use_mock:
        return LinearActuatorClient.mock()
    return LinearActuatorClient.from_serial(port=port)


def run_qt_app(use_mock: bool, port: str, quit_after_ms: int | None = None) -> int:
    try:
        from PyQt6.QtCore import QTimer
        from PyQt6.QtWidgets import (
            QApplication,
            QCheckBox,
            QDoubleSpinBox,
            QFormLayout,
            QGridLayout,
            QGroupBox,
            QHBoxLayout,
            QLabel,
            QLineEdit,
            QMainWindow,
            QMessageBox,
            QPushButton,
            QPlainTextEdit,
            QVBoxLayout,
            QWidget,
        )
    except Exception as exc:
        print(f"PyQt6 import failed: {exc}")
        return 2

    class MainWindow(QMainWindow):
        def __init__(self) -> None:
            super().__init__()
            self.setWindowTitle("上位机控制程序 (PyQt6) - 丝杠/通信抽象/协议分层")
            self._client_mode_mock = use_mock
            self._client_port = port
            self.client = _build_client(use_mock=use_mock, port=port)
            self._setup_ui()
            self._log(f"启动模式: {'Mock' if use_mock else 'Serial'}")
            self._log(f"默认串口: {port}")

        def _setup_ui(self) -> None:
            root = QWidget(self)
            layout = QVBoxLayout(root)

            conn_box = QGroupBox("连接")
            conn_form = QFormLayout(conn_box)
            self.port_edit = QLineEdit(port)
            self.mock_check = QCheckBox("使用 Mock（无硬件）")
            self.mock_check.setChecked(use_mock)
            conn_form.addRow("串口", self.port_edit)
            conn_form.addRow("", self.mock_check)

            conn_btns = QHBoxLayout()
            self.btn_connect = QPushButton("连接")
            self.btn_disconnect = QPushButton("断开")
            conn_btns.addWidget(self.btn_connect)
            conn_btns.addWidget(self.btn_disconnect)
            conn_form.addRow(conn_btns)

            motion_box = QGroupBox("丝杠控制")
            motion_layout = QGridLayout(motion_box)
            self.distance_spin = QDoubleSpinBox()
            self.distance_spin.setRange(1.0, 210.0)
            self.distance_spin.setDecimals(1)
            self.distance_spin.setSingleStep(1.0)
            self.distance_spin.setValue(120.0)

            self.btn_init = QPushButton("初始化驱动")
            self.btn_home = QPushButton("回原点")
            self.btn_move = QPushButton("移动到")
            self.btn_estop = QPushButton("急停")
            self.status_label = QLabel("状态: 未连接")

            motion_layout.addWidget(QLabel("目标位置(mm)"), 0, 0)
            motion_layout.addWidget(self.distance_spin, 0, 1)
            motion_layout.addWidget(self.btn_init, 1, 0)
            motion_layout.addWidget(self.btn_home, 1, 1)
            motion_layout.addWidget(self.btn_move, 2, 0)
            motion_layout.addWidget(self.btn_estop, 2, 1)
            motion_layout.addWidget(self.status_label, 3, 0, 1, 2)

            self.log_view = QPlainTextEdit()
            self.log_view.setReadOnly(True)
            self.log_view.setPlaceholderText("运行日志...")

            layout.addWidget(conn_box)
            layout.addWidget(motion_box)
            layout.addWidget(self.log_view)
            self.setCentralWidget(root)
            self.resize(700, 520)

            self.btn_connect.clicked.connect(self.on_connect)
            self.btn_disconnect.clicked.connect(self.on_disconnect)
            self.btn_init.clicked.connect(self.on_init_driver)
            self.btn_home.clicked.connect(self.on_home)
            self.btn_move.clicked.connect(self.on_move)
            self.btn_estop.clicked.connect(self.on_estop)

        def _refresh_client_if_mode_changed(self) -> None:
            desired_mock = self.mock_check.isChecked()
            desired_port = self.port_edit.text().strip() or DEFAULT_PORT
            if desired_mock == self._client_mode_mock and desired_port == self._client_port:
                return
            if self.client.is_connected():
                self.client.disconnect()
            self.client = _build_client(use_mock=desired_mock, port=desired_port)
            self._client_mode_mock = desired_mock
            self._client_port = desired_port
            self._log(f"通信层已切换: {'Mock' if desired_mock else 'Serial'}")

        def _set_status(self, text: str) -> None:
            self.status_label.setText(f"状态: {text}")

        def _log(self, message: str) -> None:
            ts = time.strftime("%H:%M:%S")
            self.log_view.appendPlainText(f"[{ts}] {message}")

        def _run_action(self, name: str, fn: Callable[[], object]) -> None:
            try:
                self._refresh_client_if_mode_changed()
                result = fn()
                self._set_status(name)
                if result is not None:
                    self._log(f"{name} 完成: {result}")
                else:
                    self._log(f"{name} 完成")
            except Exception as exc:
                self._set_status(f"{name}失败")
                self._log(f"{name} 失败: {exc}")
                QMessageBox.critical(self, "操作失败", f"{name} 失败\n{exc}")

        def on_connect(self) -> None:
            def _connect() -> None:
                self.client.connect()
            self._run_action("连接", _connect)

        def on_disconnect(self) -> None:
            try:
                self.client.disconnect()
                self._set_status("已断开")
                self._log("断开完成")
            except Exception as exc:
                self._log(f"断开失败: {exc}")

        def on_init_driver(self) -> None:
            self._run_action("初始化驱动", self.client.init_driver)

        def on_home(self) -> None:
            self._run_action("回原点", self.client.home)

        def on_move(self) -> None:
            mm = float(self.distance_spin.value())
            self._run_action(f"移动到 {mm:.1f}mm", lambda: self.client.move_to(mm))

        def on_estop(self) -> None:
            self._run_action("急停", self.client.estop)

    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()

    if quit_after_ms is not None and quit_after_ms >= 0:
        QTimer.singleShot(quit_after_ms, app.quit)

    return app.exec()


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="PyQt6 host UI for linear actuator")
    parser.add_argument("--mock", action="store_true", help="use mock transport (no hardware)")
    parser.add_argument("--port", default=DEFAULT_PORT, help="serial port for SerialTransport")
    parser.add_argument("--self-test", action="store_true", help="run protocol/transport/client smoke test and exit")
    parser.add_argument("--quit-after-ms", type=int, default=None, help="auto quit GUI after N ms")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    if args.self_test:
        return run_self_test(use_mock=True)
    return run_qt_app(use_mock=args.mock, port=args.port, quit_after_ms=args.quit_after_ms)


if __name__ == "__main__":
    raise SystemExit(main())
