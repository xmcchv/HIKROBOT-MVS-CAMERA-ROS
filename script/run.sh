#!/bin/bash

# 获取脚本的目录
SCRIPT_DIR="$(cd "$(dirname "$0")"; pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/../"; pwd)"
echo $SCRIPT_DIR $ROOT_DIR
cd $SCRIPT_DIR/../../../
# 转到catkin工作空间

export MVCAM_SDK_PATH=/opt/MVS
export MVCAM_COMMON_RUNENV=/opt/MVS/lib
export MVCAM_GENICAM_CLPROTOCOL=/opt/MVS/lib/CLProtocol
export ALLUSERSPROFILE=/opt/MVS/MVFG

# 检查系统x86 64位还是aarch64
ARCH=$(arch)
if [ "$ARCH" = "x86_64" ]; then
    echo "系统架构: x86_64"
    export LD_LIBRARY_PATH=/opt/MVS/lib/64:/opt/MVS/lib/32:$LD_LIBRARY_PATH
else
    echo "系统架构: aarch64"
    export LD_LIBRARY_PATH=/opt/MVS/lib/aarch64:$LD_LIBRARY_PATH
fi



# 执行catkin_make并检查返回值
if catkin_make -j2 -DCATKIN_WHITELIST_PACKAGES="hikmvs_ros_driver"; then
    echo "编译成功，继续执行..."
    source devel/setup.bash

    # 获取第一个参数
    PARAM=$1
    # 根据参数的值执行不同的操作
    if [ "$PARAM" = "-n" ]; then
        echo "不启动程序..."
    elif [ "$PARAM" = "-t" ]; then
        echo "启动程序..."
        roslaunch hikmvs_ros_driver test_two_camera_rviz.launch
    else
        echo "直接运行..."
        roslaunch hikmvs_ros_driver hikrobot_camera_indoor_rviz.launch
    fi
else
    echo "编译失败，停止执行!"
    exit 1
fi