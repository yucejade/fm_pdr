#!/bin/bash
# network_mode.sh - 网络模式切换脚本
# 用法: sudo ./network_mode.sh <0|1>
#   0 - 热点模式: 关闭WiFi, 开启热点
#   1 - WiFi模式: 关闭热点, 开启WiFi

HOTSPOT_IP="192.168.8.8/24"
HOTSPOT_IFACE="ap0"
WIFI_IFACE="wlan0"
WIFI_SSID="fm-shenyang-office"
PORTAL_PID="/tmp/hotspot_portal.pid"
PORTAL_CONF="/etc/dnsmasq.d/hotspot-portal.conf"

# 必须以root运行
if [ "$EUID" -ne 0 ]; then
    echo "错误: 请使用 sudo 运行此脚本"
    echo "用法: sudo $0 <0|1>"
    exit 1
fi

# 启动Captive Portal假响应服务
start_portal() {
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
    rm -f "$PORTAL_CONF"
    systemctl restart dnsmasq
}

mode_hotspot() {
    echo "[模式0] 切换到热点模式..."

    echo "  断开WiFi连接..."
    nmcli device disconnect "$WIFI_IFACE" 2>/dev/null

    echo "  等待信道释放..."
    sleep 2

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

    echo "  关闭网络检测服务..."
    stop_portal

    echo "  关闭热点服务..."
    systemctl stop hostapd
    systemctl mask hostapd

    echo "  连接WiFi: $WIFI_SSID..."
    nmcli device wifi connect "$WIFI_SSID" ifname "$WIFI_IFACE"

    echo "[WiFi模式已启动] SSID: $WIFI_SSID"
}

if [ $# -ne 1 ]; then
    echo "用法: sudo $0 <0|1>"
    echo "  0 - 热点模式: 关闭WiFi, 开启热点"
    echo "  1 - WiFi模式: 关闭热点, 开启WiFi"
    exit 1
fi

case "$1" in
    0) mode_hotspot ;;
    1) mode_wifi ;;
    *) echo "错误: 参数必须是 0 或 1"; exit 1 ;;
esac
