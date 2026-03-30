#!/bin/bash

# 激活conda环境（Dfg），适配不同的conda安装路径
# 若conda未加入系统环境，手动指定conda路径（常见路径二选一）
# 方式1：conda安装在用户目录（推荐）
source ~/anaconda3/etc/profile.d/conda.sh
# 方式2：conda安装在anaconda目录
# source ~/anaconda3/etc/profile.d/conda.sh

# 激活Dfg环境
conda activate Dfg

# 进入代码所在目录（替换为你的实际代码路径！）
cd /home/test/projects/Dfg/DfgV2_1

# 启动UI.py程序
python3 UI.py

# 若程序退出后需要保留终端，取消下面注释
# read -p "按任意键退出..."

