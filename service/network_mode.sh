#!/bin/bash
# network_mode.sh - 网络模式切换脚本
# 用法: sudo ./network_mode.sh <0|1|2>
#   0 - 热点模式: 关闭WiFi, 开启热点
#   1 - WiFi模式: 关闭热点, 开启WiFi
#   2 - 并发模式: WiFi和热点同时开启（STA+AP，需同一信道）

HOTSPOT_IP="192.168.8.8/24"
HOTSPOT_IFACE="ap0"
WIFI_IFACE="wlan0"
WIFI_SSID="fm-shenyang-office"
PORTAL_PID="/tmp/hotspot_portal.pid"
PORTAL_CONF="/etc/dnsmasq.d/hotspot-portal.conf"
HOSTAPD_CONF="/etc/hostapd/hostapd.conf"
HOSTAPD_CONF_5G="/etc/hostapd/hostapd.conf.5g"

# 必须以root运行
if [ "$EUID" -ne 0 ]; then
    echo "错误: 请使用 sudo 运行此脚本"
    echo "用法: sudo $0 <0|1|2>"
    exit 1
fi

# 停止所有热点相关服务，清理环境
stop_all() {
    echo "  停止热点相关服务..."
    systemctl stop pi-hotspot 2>/dev/null
    systemctl stop hostapd 2>/dev/null
    killall hostapd 2>/dev/null
    stop_portal
    sleep 1
    # 删除旧的ap0虚拟接口（如果存在）
    ip link show ap0 &>/dev/null && iw dev ap0 del 2>/dev/null
    # 恢复wlan0为受管模式
    nmcli device set "$WIFI_IFACE" managed yes
    nmcli device wifi rescan 2>/dev/null
    sleep 3
    nmcli connection up "$WIFI_SSID" ifname "$WIFI_IFACE" 2>/dev/null && sleep 2 2>/dev/null
}

# 启动Captive Portal假响应服务
start_portal() {
    # 先杀掉旧的portal进程
    stop_portal
    # 等待端口释放
    sleep 1

    cat > "$PORTAL_CONF" <<EOF
address=/connectivitycheck.gstatic.com/192.168.8.8
address=/clients3.google.com/192.168.8.8
address=/captive.apple.com/192.168.8.8
address=/www.msftconnecttest.com/192.168.8.8
address=/www.apple.com/192.168.8.8
EOF
    systemctl restart dnsmasq

    python3 -c "
from http.server import HTTPServer, BaseHTTPRequestHandler
class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        self.send_response(204)
        self.end_headers()
    def do_HEAD(self):
        self.send_response(204)
        self.end_headers()
    def do_POST(self):
        self.send_response(204)
        self.end_headers()
    def log_message(self, *args):
        pass
HTTPServer(('0.0.0.0', 80), Handler).serve_forever()
" &
    echo $! > "$PORTAL_PID"
}

stop_portal() {
    if [ -f "$PORTAL_PID" ]; then
        kill $(cat "$PORTAL_PID") 2>/dev/null
        rm -f "$PORTAL_PID"
    fi
    # 确保所有残留的portal进程都被清理
    pkill -f "HTTPServer.*80.*Handler" 2>/dev/null
    rm -f "$PORTAL_CONF"
    systemctl restart dnsmasq 2>/dev/null
}

# 获取WiFi当前连接的信道号
get_wifi_channel() {
    local freq
    freq=$(iw dev "$WIFI_IFACE" link 2>/dev/null | grep "freq:" | awk '{print $2}')
    if [ -n "$freq" ]; then
        if [ "$freq" -ge 2412 ] && [ "$freq" -le 2484 ]; then
            echo $(( (freq - 2407) / 5 ))
        elif [ "$freq" -ge 5170 ] && [ "$freq" -le 5885 ]; then
            echo $(( (freq - 5000) / 5 ))
        fi
    fi
}

mode_hotspot() {
    echo "[模式0] 切换到热点模式..."

    # 清理环境
    stop_all

    echo "  断开WiFi连接..."
    nmcli device disconnect "$WIFI_IFACE" 2>/dev/null
    nmcli device set "$WIFI_IFACE" managed no

    echo "  等待信道释放..."
    sleep 2

    echo "  切换到5GHz hostapd配置..."
    if [ -f "$HOSTAPD_CONF_5G" ]; then
        cp "$HOSTAPD_CONF_5G" "$HOSTAPD_CONF"
    fi

    echo "  创建AP虚拟接口..."
    iw dev "$WIFI_IFACE" interface add "$HOTSPOT_IFACE" type __ap
    sleep 1

    echo "  启动热点服务..."
    systemctl unmask hostapd
    systemctl start hostapd

    echo "  配置热点IP地址..."
    ip addr add "$HOTSPOT_IP" dev "$HOTSPOT_IFACE" 2>/dev/null
    ip link set "$HOTSPOT_IFACE" up

    echo "  启动网络检测服务..."
    start_portal

    echo "[热点模式已启动] SSID: PiHotspot, IP: 192.168.8.8"
}

mode_wifi() {
    echo "[模式1] 切换到WiFi模式..."

    # 清理环境（停止pi-hotspot和hostapd，删除ap0）
    stop_all

    echo "  连接WiFi: $WIFI_SSID..."
    nmcli connection up "$WIFI_SSID" ifname "$WIFI_IFACE" 2>/dev/null || \
        nmcli device wifi connect "$WIFI_SSID" ifname "$WIFI_IFACE"

    echo "[WiFi模式已启动] SSID: $WIFI_SSID"
}

mode_concurrent() {
    echo "[模式2] 切换到并发模式（WiFi + 热点）..."

    # 先清理旧的ap0（pi-hotspot启动时会重新创建）
    systemctl stop pi-hotspot 2>/dev/null
    killall hostapd 2>/dev/null
    stop_portal
    ip link show ap0 &>/dev/null && iw dev ap0 del 2>/dev/null

    # 确保WiFi已连接
    echo "  确保WiFi已连接..."
    nmcli device set "$WIFI_IFACE" managed yes
    nmcli device wifi rescan 2>/dev/null
    sleep 3
    nmcli connection up "$WIFI_SSID" ifname "$WIFI_IFACE" 2>/dev/null && sleep 2

    local is_connected
    is_connected=$(iw dev "$WIFI_IFACE" link 2>/dev/null | grep "Not connected")
    if [ -n "$is_connected" ]; then
        echo "  连接WiFi: $WIFI_SSID..."
        nmcli device wifi connect "$WIFI_SSID" ifname "$WIFI_IFACE"
        sleep 3
    fi

    # 获取WiFi当前信道
    local channel
    channel=$(get_wifi_channel)
    if [ -z "$channel" ]; then
        echo "  错误: WiFi未连接，无法确定信道"
        exit 1
    fi
    echo "  WiFi当前信道: $channel"

    # 生成与WiFi同信道的hostapd配置
    echo "  生成并发模式hostapd配置（信道 $channel）..."
    local hw_mode
    if [ "$channel" -le 14 ]; then
        hw_mode="g"
    else
        hw_mode="a"
    fi

    cat > "$HOSTAPD_CONF" <<EOF
interface=$HOTSPOT_IFACE
driver=nl80211
ssid=PiHotspot
hw_mode=$hw_mode
channel=$channel
ieee80211n=1
wmm_enabled=1
auth_algs=1
wpa=2
wpa_passphrase=87010054
wpa_key_mgmt=WPA-PSK
rsn_pairwise=CCMP
ap_max_inactivity=3600
disassoc_low_ack=0
skip_inactivity_poll=1
wpa_group_rekey=3600
EOF

    # 使用pi-hotspot服务启动并发模式
    echo "  启动并发模式服务..."
    systemctl start pi-hotspot
    sleep 2

    echo "  启动网络检测服务..."
    start_portal

    echo "[并发模式已启动] WiFi: $WIFI_SSID (信道$channel), 热点: PiHotspot, IP: 192.168.8.8"
}

if [ $# -ne 1 ]; then
    echo "用法: sudo $0 <0|1|2>"
    echo "  0 - 热点模式: 关闭WiFi, 开启热点"
    echo "  1 - WiFi模式: 关闭热点, 开启WiFi"
    echo "  2 - 并发模式: WiFi和热点同时开启"
    exit 1
fi

case "$1" in
    0) mode_hotspot ;;
    1) mode_wifi ;;
    2) mode_concurrent ;;
    *) echo "错误: 参数必须是 0、1 或 2"; exit 1 ;;
esac
