import logging
import os
import inspect
from datetime import datetime
import sys
import time

class MillisFormatter(logging.Formatter):
    def formatTime(self, record, datefmt=None):
        ct = self.converter(record.created)
        if datefmt:
            s = time.strftime(datefmt, ct)
        else:
            dt = datetime.fromtimestamp(record.created)
            millis = int(dt.microsecond // 1000)
            s = dt.strftime("%Y-%m-%d %H:%M:%S")
            s += f".{millis:03d}"
        return s

class CustomLogger:
    _instance = None

    def __new__(cls, *args, **kwargs):
        if not cls._instance:
            cls._instance = super().__new__(cls)
            cls._instance._init_logger(*args, **kwargs)
        return cls._instance

    def _init_logger(self, project_name):
        self.project_name = project_name
        self.log_dir = "logs"
        os.makedirs(self.log_dir, exist_ok=True)
        self._setup_logger()

    def _setup_logger(self):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_filename = f"{timestamp}_{self.project_name}.log"
        log_path = os.path.join(self.log_dir, log_filename)

        file_handler = logging.FileHandler(log_path)
        file_handler.setLevel(logging.DEBUG)
        
        formatter = MillisFormatter(
            fmt='%(asctime)s] %(filename)-20s %(lineno)-6s %(levelname)-8s: %(message)s',
            datefmt=None
        )
        file_handler.setFormatter(formatter)

        self.logger = logging.getLogger(self.project_name)
        self.logger.setLevel(logging.DEBUG)
        self.logger.addHandler(file_handler)

        self._bind_module_functions()

    def _bind_module_functions(self):
        sys.modules[__name__].LOG_DEBUG = self.logger.debug
        sys.modules[__name__].LOG_INFO = self.logger.info
        sys.modules[__name__].LOG_WARN = self.logger.warning
        sys.modules[__name__].LOG_ERROR = self.logger.error
        sys.modules[__name__].LOG_CRITICAL = self.logger.critical

# 初始化日志系统
logger = CustomLogger(project_name="CFG")