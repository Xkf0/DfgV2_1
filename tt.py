# 导入正确的 loguru 模块
from loguru import logger

# 简单的使用示例，验证导入是否成功
if __name__ == "__main__":
    # 输出普通日志
    logger.info("这是一条信息日志")
    # 输出警告日志
    logger.warning("这是一条警告日志")
    # 输出错误日志
    logger.error("这是一条错误日志")



