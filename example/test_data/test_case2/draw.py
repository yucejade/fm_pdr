import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.basemap import Basemap
import matplotlib as mpl
import os

# 解决中文显示问题
plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei']
plt.rcParams['axes.unicode_minus'] = False

# 1. 读取Location_output.csv文件（惯导轨迹）
file_path = 'Location_output.csv'
data = pd.read_csv(file_path, delimiter=',', skiprows=1, header=None, 
                   names=['Time', 'Latitude', 'Longitude', 'Height', 
                          'Velocity', 'Direction', 'Horizontal_Accuracy', 
                          'Vertical_Accuracy'])

print(f"成功读取惯导轨迹数据点: {len(data)}")
print(f"方向数据范围: {data['Direction'].min():.2f} - {data['Direction'].max():.2f}")

# 2. 尝试读取Location.csv文件（GPS轨迹）
green_data = None
if os.path.exists('Location.csv'):
    try:
        green_data = pd.read_csv('Location.csv', delimiter=',', skiprows=1, header=None,
                                names=['Time', 'Latitude', 'Longitude', 'Height', 
                                       'Velocity', 'Direction', 'Horizontal_Accuracy', 
                                       'Vertical_Accuracy'])
        print(f"成功读取GPS轨迹数据点: {len(green_data)}")
    except Exception as e:
        print(f"读取Location.csv出错: {e}")
        green_data = None
else:
    print("未找到Location.csv文件，将只绘制惯导轨迹")

# 3. 创建地图
plt.figure(figsize=(12, 10))

# 计算地图边界（包含惯导和GPS轨迹）
all_lons = data['Longitude'].tolist()
all_lats = data['Latitude'].tolist()

if green_data is not None:
    all_lons.extend(green_data['Longitude'])
    all_lats.extend(green_data['Latitude'])

min_lon, max_lon = min(all_lons), max(all_lons)
min_lat, max_lat = min(all_lats), max(all_lats)

# 扩展边界
lon_padding = max(0.001, (max_lon - min_lon) * 0.1)
lat_padding = max(0.001, (max_lat - min_lat) * 0.1)

print(f"经度范围: {min_lon:.6f} - {max_lon:.6f}, 纬度范围: {min_lat:.6f} - {max_lat:.6f}")
print(f"经度差: {max_lon-min_lon:.6f}, 纬度差: {max_lat-min_lat:.6f}")

# 创建Basemap实例
m = Basemap(
    projection='merc',
    llcrnrlon=min_lon - lon_padding,
    llcrnrlat=min_lat - lat_padding,
    urcrnrlon=max_lon + lon_padding,
    urcrnrlat=max_lat + lat_padding,
    resolution='i',
    lat_ts=(min_lat + max_lat)/2,  # 添加标准纬线
    suppress_ticks=True
)

# 绘制地图背景
m.drawmapboundary(fill_color='lightblue')
m.fillcontinents(color='beige', lake_color='lightblue')
m.drawcoastlines()
m.drawrivers(color='blue')

# 将经纬度转换为地图坐标（惯导轨迹）
lons = data['Longitude'].values.astype(float)
lats = data['Latitude'].values.astype(float)
x, y = m(lons, lats)

# 4. 绘制惯导行进轨迹
m.plot(x, y, 'b-', linewidth=3, label='惯导轨迹')
m.scatter(x[0], y[0], s=100, c='green', marker='o', label='起点')
m.scatter(x[-1], y[-1], s=100, c='red', marker='s', label='终点')

# 5. 绘制GPS轨迹（如果存在）
if green_data is not None:
    green_lons = green_data['Longitude'].values.astype(float)
    green_lats = green_data['Latitude'].values.astype(float)
    green_x, green_y = m(green_lons, green_lats)
    m.plot(green_x, green_y, 'g-', linewidth=3, label='GPS轨迹')

# 6. 绘制方向箭头 - 修正方案
print("开始绘制方向箭头...")
time_interval = 15  # 秒

# 计算箭头点索引
if len(data) > 1:
    time_diff = data['Time'].iloc[1] - data['Time'].iloc[0]
    step = max(1, int(time_interval / time_diff))
    print(f"数据时间间隔: {time_diff:.4f}秒, 箭头间隔步长: {step}")
else:
    step = 1

# 准备箭头数据
arrow_indices = range(0, len(data), step)
arrow_x = x[arrow_indices]
arrow_y = y[arrow_indices]

# 关键修正：使用固定箭头长度（米）
arrow_length = 300  # 箭头长度500米
directions = data['Direction'].iloc[arrow_indices].values

# 计算东方向和北方向分量（米制）
u = arrow_length * np.sin(np.radians(directions))  # 东方向分量
v = arrow_length * np.cos(np.radians(directions))  # 北方向分量
# 计算地图边长（取经度差和纬度差中较大的值）
map_size = max(max_lon - min_lon, max_lat - min_lat)
# 根据地图边长动态设置箭头长度（取地图边长的15%，但不超过200米）
arrow_length = min(map_size * 0.15, 200)  
# 适当减小箭头宽度
width = 0.005  
# 调整箭头头部形状参数
headwidth = 3
headlength = 5
# 设置箭头颜色透明度
alpha = 1.0  

# 使用quiver绘制所有箭头（使用米制单位）
q = m.quiver(arrow_x, arrow_y, u, v, 
             color='r', scale=500, scale_units='inches',
             width=width, headwidth=headwidth, headlength=headlength,
             alpha=alpha, label='方向指示', zorder=10)

# 根据新的箭头长度重新设置比例尺
plt.quiverkey(q, 0.1, 0.1, arrow_length, 
              f"{arrow_length}米", labelpos='E',
              fontproperties={'size': 5, 'weight': 'bold'})

print(f"绘制了 {len(arrow_indices)} 个方向箭头")

# 7. 添加图例和标题
plt.legend(loc='best', fontsize=12)
plt.title('手持移动轨迹与方向指示图', fontsize=16)

# 保存并显示图像
plt.savefig('轨迹图_含方向指示.png', dpi=300, bbox_inches='tight')
print("图像已保存为 '轨迹图_含方向指示.png'")

# 添加调试信息
print("箭头坐标示例:")
for i in range(min(5, len(arrow_indices))):
    idx = arrow_indices[i]
    print(f"点 {idx}: 地图坐标({arrow_x[i]:.1f}, {arrow_y[i]:.1f}), 方向({u[i]:.1f}, {v[i]:.1f})")
    
plt.show()
