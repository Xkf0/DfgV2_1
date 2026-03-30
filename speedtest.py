# speedtest.py (示例修改)
# import config # 移除旧导入
from config_loader import Configer
import time
# 假设其它依赖...

if __name__ == "__main__":
    # 1. 创建配置实例
    my_config = Configer(output_w=1280, output_h=720, real_w=30.0) # 示例：使用不同配置

    # 2. 在需要时使用 my_config.属性名
    start_time = time.time()
    # ... some operation ...
    # 假设某个函数需要 OUTPUT_W
    some_result = some_function_requiring_width(my_config.OUTPUT_W)
    end_time = time.time()
    print(f"Operation took {end_time - start_time} seconds with width {my_config.OUTPUT_W}")