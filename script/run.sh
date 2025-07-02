#!/bin/bash

# 获取脚本的目录
SCRIPT_DIR="$(cd "$(dirname "$0")"; pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/../"; pwd)"
echo $SCRIPT_DIR $ROOT_DIR
cd $SCRIPT_DIR/../../../
# 转到catkin工作空间

# 执行catkin_make并检查返回值
if catkin_make -j2 -DCATKIN_WHITELIST_PACKAGES="hikrobot_camera"; then
    echo "编译成功，继续执行..."
    source devel/setup.bash

    # 获取第一个参数
    PARAM=$1
    # 根据参数的值执行不同的操作
    if [ "$PARAM" = "-n" ]; then
        echo "不启动程序..."
    else
        echo "直接运行..."
        roslaunch hikrobot_camera hikrobot_camera_indoor_rviz.launch
    fi
else
    echo "编译失败，停止执行!"
    exit 1
fi