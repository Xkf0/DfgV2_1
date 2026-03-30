# from PyQt6.QtWidgets import QApplication, QLabel
# import sys


# app = QApplication(sys.argv)
# w = QLabel("Hello PyQt6")
# w.resize(240, 80)
# w.show()
# sys.exit(app.exec())


from logger import LOG_DEBUG, LOG_INFO, LOG_WARN, LOG_ERROR, LOG_CRITICAL
x = 1.0
LOG_INFO("This is a debug message, %f", x) # 自动记录调用位置
from datetime import datetime
timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
filename = f"imgs/single_mask_{timestamp}.png"
LOG_INFO("save picture ++++++++++++++++++, %s", timestamp)