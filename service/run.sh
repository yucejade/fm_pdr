#!/bin/bash

# 定义环境变量（修复原脚本的变量替换语法错误）
export PACKAGE_PATH="/home/kevin/Dev/yuce/fm_pdr/build/package"

# 检查pdr_test.service服务状态
check_service_status() {
    # 使用systemctl is-active检查服务是否运行（--quiet仅返回退出码，0=活跃，非0=未活跃）
    if sudo systemctl is-active --quiet pdr_test.service; then
        return 0  # 服务已启动
    else
        return 1  # 服务未启动
    fi
}

# 监听键盘'q'按键并停止服务
monitor_keyboard() {
    echo -e "\n按'q'键停止服务..."
    while true; do
        read -n1 -s input  # -n1:读取1个字符；-s:不回显输入
        if [[ "$input" == "q" ]]; then
            echo -e "\n正在停止服务..."
            sudo systemctl stop pdr_test.service
            
            # 等待服务完全停止（可选，确保文件操作不冲突）
            sleep 1
            
            # 处理输出文件
            mv -f "${PACKAGE_PATH}/bin/Trajectory.csv" "${PACKAGE_PATH}/bin/Trajectory_bak.csv"
            "${PACKAGE_PATH}/bin/PDRTest" -d "${PACKAGE_PATH}/bin/output_sensor_data" -o "${PACKAGE_PATH}/bin/Trajectory.csv" -e
            
            echo "服务已停止，文件处理完成！"
            exit 0
        fi
    done
}

# 主逻辑
if check_service_status; then
    echo "检测到pdr_test.service已在运行中"
    monitor_keyboard  # 直接监听键盘
else
    echo "开始PDR倒计时:3"
    sleep 1
    echo "开始PDR倒计时:2"
    sleep 1
    echo "开始PDR倒计时:1"
    sleep 1
    echo "开始PDR!!!"

    # 清理旧文件（-f强制删除，避免不存在时报错）
    rm -fr "${PACKAGE_PATH}/bin/output_sensor_data" \
           "${PACKAGE_PATH}/bin/Trajectory.csv" \
           "${PACKAGE_PATH}/bin/Trajectory_bak.csv"

    # 启动服务
    sudo systemctl start pdr_test.service
    
    # 验证服务是否启动成功（可选增强）
    if ! check_service_status; then
        echo "错误：pdr_test.service启动失败！"
        exit 1
    fi
    
    monitor_keyboard  # 启动后监听键盘
fi