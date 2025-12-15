#! /bin/bash

# 配置目标WiFi信息
TARGET_SSID="Redmi K50 Pro"
WIFI_PASSWORD="87010054"
CHECK_INTERVAL=5  # 未连接时的扫描间隔（秒）
CONNECTED_WAIT=5  # 已连接时的等待间隔（秒）

echo -e "=== WiFi自动切换脚本 ==="
echo "监控目标：$TARGET_SSID"
echo "按 'q' 键随时退出脚本"
echo "------------------------"

while true; do
    # 1. 检查是否按'q'键退出
    read -t 1 -n 1 input  # 1秒超时监听，无需回车
    if [[ $input == "q" ]]; then
        echo -e "\n用户终止脚本，退出..."
        exit 0
    fi

    # 2. 检查当前是否已连接目标WiFi
    CURRENT_SSID=$(iwgetid -r)  # 获取当前连接的SSID
    if [[ "$CURRENT_SSID" == "$TARGET_SSID" ]]; then
        echo -e "[$(date +'%H:%M:%S')] 已连接到$TARGET_SSID，$CONNECTED_WAIT秒后重新检查..."
        sleep $CONNECTED_WAIT
        continue  # 跳过后续逻辑，直接进入下一次循环
    fi

    # 3. 未连接时扫描WiFi列表
    echo -n "[$(date +'%H:%M:%S')] 扫描WiFi中..."
    WIFI_AVAILABLE=$(sudo nmcli device wifi list | grep -i "$TARGET_SSID")
    
    if [[ -n $WIFI_AVAILABLE ]]; then
        echo -e "\n发现目标WiFi：$TARGET_SSID！正在切换..."
        # 执行WiFi切换
        sudo nmcli device wifi connect "$TARGET_SSID" password "$WIFI_PASSWORD"
        
        # 检查连接结果
        if [[ $? -eq 0 ]]; then
            echo "✅ WiFi切换成功！继续监控连接状态..."
        else
            echo "❌ WiFi连接失败，$CHECK_INTERVAL秒后重试..."
        fi
    else
        echo -e "\r[$(date +'%H:%M:%S')] 未发现目标WiFi，$CHECK_INTERVAL秒后重试..."
    fi

    # 4. 等待指定间隔后再次扫描
    sleep $CHECK_INTERVAL
done