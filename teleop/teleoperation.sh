#!/bin/zsh
echo "start Franka"
# 切换到 Franka/frankapy 目录
cd ~/Franka/frankapy || { echo "目录不存在！"; exit 1; }

# 执行 start_control_pc.sh 脚本
bash ./bash_scripts/start_control_pc.sh -i localhost

# 等待 10 秒
echo "等待 10 秒..."
sleep 10

# 在新终端中运行命令 spacemouse.py
if [ -d "$HOME/teleoperation/franka" ]; then
    gnome-terminal -- bash -c "cd $HOME/teleoperation/franka && python teleop/spacemouse.py; exec bash"
else
    echo "目录 teleoperation/franka 不存在！"
    exit 1
fi

# 再次打开一个新的 zsh 终端并执行 run.py
echo "开启新的 zsh 终端执行 run.py..."
if [ -d "$HOME/teleoperation/franka" ]; then
    gnome-terminal -- bash -c "cd $HOME/teleoperation/franka && python teleop/run.py; exec bash"
echo "启动3D SpaceMouse teleoperation"
else
    echo "目录 teleoperation/franka 不存在！"
    exit 1
fi
