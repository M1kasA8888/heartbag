import streamlit as st
import pandas as pd
import numpy as np
import folium
from streamlit_folium import st_folium
from folium import plugins
from shapely.geometry import Point, Polygon, LineString
from shapely.ops import unary_union
import plotly.graph_objects as go
import plotly.express as px
from datetime import datetime, timedelta
import time
import math
from flight_planner import FlightPlanner, DroneSimulator

st.set_page_config(
    page_title="无人机航线规划系统",
    page_icon="✈️",
    layout="wide"
)

# 初始化session state
if 'obstacles' not in st.session_state:
    st.session_state.obstacles = []  # 存储障碍区
if 'waypoints' not in st.session_state:
    st.session_state.waypoints = []  # 存储航点
if 'flight_plan' not in st.session_state:
    st.session_state.flight_plan = None
if 'is_flying' not in st.session_state:
    st.session_state.is_flying = False
if 'drone_pos' not in st.session_state:
    st.session_state.drone_pos = None
if 'selected_points' not in st.session_state:
    st.session_state.selected_points = []

st.title("✈️ 无人机智能航线规划系统")
st.markdown("---")

# 侧边栏 - 参数设置
with st.sidebar:
    st.header("⚙️ 参数设置")
    
    # 安全半径设置
    safe_radius = st.slider("🛡️ 安全半径 (米)", 10, 100, 30, 
                             help="无人机与障碍物保持的最小距离")
    
    # 飞行参数
    st.subheader("飞行参数")
    flight_speed = st.slider("飞行速度 (m/s)", 5, 30, 15)
    start_altitude = st.number_input("起始高度 (米)", 50, 500, 100)
    
    # 地图中心设置
    st.subheader("地图设置")
    center_lat = st.number_input("中心纬度", -90.0, 90.0, 39.9042)
    center_lon = st.number_input("中心经度", -180.0, 180.0, 116.4074)
    zoom_start = st.slider("地图缩放级别", 10, 18, 14)
    
    st.markdown("---")
    st.markdown("### 📝 操作说明")
    st.markdown("""
    1. 点击地图**标记障碍区边界点**
    2. 点击**完成障碍区**保存
    3. 设置**起飞点**和**目标点**
    4. 点击**规划航线**生成路径
    5. 点击**开始飞行**监控飞行
    """)

# 主区域布局
col1, col2 = st.columns([2, 1])

with col1:
    st.subheader("🗺️ 航线规划地图")
    
    # 创建地图
    m = folium.Map(location=[center_lat, center_lon], zoom_start=zoom_start)
    
    # 绘制已保存的障碍区
    for i, obstacle in enumerate(st.session_state.obstacles):
        if len(obstacle) >= 3:
            # 绘制障碍区多边形
            folium.Polygon(
                locations=obstacle,
                color='red',
                weight=2,
                fill=True,
                fill_color='red',
                fill_opacity=0.3,
                popup=f'障碍区 {i+1}'
            ).add_to(m)
            
            # 绘制安全缓冲区
            # 在实际应用中，这里会绘制膨胀后的区域
    
    # 绘制航点
    if st.session_state.waypoints:
        # 起飞点
        folium.Marker(
            st.session_state.waypoints[0],
            popup='起飞点',
            icon=folium.Icon(color='green', icon='play', prefix='fa')
        ).add_to(m)
        
        # 目标点
        if len(st.session_state.waypoints) > 1:
            folium.Marker(
                st.session_state.waypoints[-1],
                popup='目标点',
                icon=folium.Icon(color='red', icon='flag-checkered', prefix='fa')
            ).add_to(m)
        
        # 绘制航线
        if len(st.session_state.waypoints) >= 2:
            folium.PolyLine(
                st.session_state.waypoints,
                color='blue',
                weight=3,
                opacity=0.8,
                popup='规划航线'
            ).add_to(m)
            
            # 添加航点标记
            for i, wp in enumerate(st.session_state.waypoints[1:-1], 1):
                folium.Marker(
                    wp,
                    popup=f'航点 {i}',
                    icon=folium.Icon(color='orange', icon='info-sign')
                ).add_to(m)
    
    # 绘制当前无人机位置
    if st.session_state.drone_pos:
        folium.Marker(
            st.session_state.drone_pos,
            popup='无人机当前位置',
            icon=folium.Icon(color='blue', icon='helicopter', prefix='fa')
        ).add_to(m)
    
    # 添加绘图工具
    draw = plugins.Draw(
        draw_options={
            'polyline': False,
            'rectangle': False,
            'circle': False,
            'marker': True,
            'polygon': {'allowIntersection': False},
            'circlemarker': False
        },
        edit_options={'edit': True}
    )
    draw.add_to(m)
    
    # 显示地图
    output = st_folium(m, width=700, height=500)
    
    # 处理地图绘图
    if output and 'last_active_drawing' in output:
        drawing = output['last_active_drawing']
        if drawing:
            if drawing['geometry']['type'] == 'Polygon':
                coords = drawing['geometry']['coordinates'][0]
                points = [[c[1], c[0]] for c in coords]  # 转换格式
                st.session_state.selected_points = points
            elif drawing['geometry']['type'] == 'Point':
                point = [drawing['geometry']['coordinates'][1], 
                        drawing['geometry']['coordinates'][0]]
                if len(st.session_state.waypoints) == 0:
                    st.session_state.waypoints.append(point)
                    st.success("起飞点已设置")
                elif len(st.session_state.waypoints) == 1:
                    st.session_state.waypoints.append(point)
                    st.success("目标点已设置")
                else:
                    st.warning("已有起飞点和目标点，请清除后重试")

with col2:
    st.subheader("🎯 障碍区管理")
    
    # 显示当前绘制的点
    if st.session_state.selected_points:
        st.write("当前绘制的边界点:")
        for i, point in enumerate(st.session_state.selected_points):
            st.text(f"点{i+1}: ({point[0]:.4f}, {point[1]:.4f})")
        
        col_btn1, col_btn2 = st.columns(2)
        with col_btn1:
            if st.button("✅ 完成障碍区", use_container_width=True):
                if len(st.session_state.selected_points) >= 3:
                    st.session_state.obstacles.append(st.session_state.selected_points.copy())
                    st.session_state.selected_points = []
                    st.success("障碍区已添加")
                    st.rerun()
                else:
                    st.error("至少需要3个点才能形成障碍区")
        with col_btn2:
            if st.button("🗑️ 清除绘制", use_container_width=True):
                st.session_state.selected_points = []
                st.rerun()
    
    # 显示已保存的障碍区
    if st.session_state.obstacles:
        st.write("---")
        st.write("已保存的障碍区:")
        for i, obs in enumerate(st.session_state.obstacles):
            with st.expander(f"障碍区 {i+1}"):
                st.write(f"边界点数: {len(obs)}")
                if st.button(f"删除障碍区 {i+1}", key=f"del_{i}"):
                    st.session_state.obstacles.pop(i)
                    st.rerun()
    
    # 清除所有
    if st.button("🗑️ 清除所有障碍区", use_container_width=True):
        st.session_state.obstacles = []
        st.rerun()
    
    if st.button("🗺️ 清除航线", use_container_width=True):
        st.session_state.waypoints = []
        st.session_state.flight_plan = None
        st.rerun()

st.markdown("---")

# 航线规划区域
col1, col2, col3 = st.columns(3)

with col1:
    if st.button("🚀 规划航线", use_container_width=True, type="primary"):
        if len(st.session_state.waypoints) >= 2:
            planner = FlightPlanner(st.session_state.obstacles, safe_radius)
            flight_plan = planner.plan_route(
                st.session_state.waypoints[0],
                st.session_state.waypoints[-1]
            )
            if flight_plan:
                st.session_state.flight_plan = flight_plan
                st.success("航线规划成功！")
            else:
                st.error("无法规划安全航线，请调整障碍区或航点")
        else:
            st.warning("请先设置起飞点和目标点")

with col2:
    if st.button("📊 显示航线信息", use_container_width=True):
        if st.session_state.flight_plan:
            info = st.session_state.flight_plan
            st.info(f"""
            📏 总航程: {info['total_distance']:.2f} 米
            ⏱️ 预计时间: {info['estimated_time']:.2f} 秒
            🛡️ 安全半径: {safe_radius} 米
            📍 航点数量: {len(info['waypoints'])}
            """)
        else:
            st.warning("请先规划航线")

with col3:
    if st.button("🎮 开始飞行监控", use_container_width=True):
        if st.session_state.flight_plan:
            st.session_state.is_flying = True
        else:
            st.warning("请先规划航线")

# 飞行监控界面
if st.session_state.is_flying:
    st.markdown("---")
    st.subheader("🎮 飞行监控界面")
    
    # 创建飞行模拟器
    if 'simulator' not in st.session_state:
        st.session_state.simulator = DroneSimulator(
            st.session_state.flight_plan['waypoints'],
            flight_speed
        )
        st.session_state.start_time = datetime.now()
    
    # 监控仪表盘
    col1, col2, col3, col4, col5 = st.columns(5)
    
    # 更新飞行状态
    if st.session_state.simulator.update():
        # 获取当前状态
        status = st.session_state.simulator.get_status()
        
        with col1:
            st.metric("📍 当前航点", f"{status['current_waypoint']}/{status['total_waypoints']}")
        with col2:
            st.metric("⚡ 飞行速度", f"{flight_speed} m/s")
        with col3:
            elapsed = (datetime.now() - st.session_state.start_time).total_seconds()
            st.metric("⏱️ 已用时间", f"{elapsed:.1f} 秒")
        with col4:
            st.metric("📏 剩余距离", f"{status['remaining_distance']:.1f} 米")
        with col5:
            remaining_time = status['remaining_distance'] / flight_speed if flight_speed > 0 else 0
            st.metric("⏰ 预计到达", f"{remaining_time:.1f} 秒")
        
        # 第二行仪表
        col1, col2, col3, col4, col5 = st.columns(5)
        with col1:
            # 电池模拟
            battery = max(0, 100 - (elapsed / 600) * 100)  # 10分钟飞行时间
            st.metric("🔋 电量", f"{battery:.0f}%")
        with col2:
            st.metric("📊 完成进度", f"{status['progress']:.1f}%")
        with col3:
            st.metric("🎯 当前航段进度", f"{status['segment_progress']:.1f}%")
        with col4:
            st.metric("🛡️ 安全状态", "正常" if status['is_safe'] else "警告")
        
        # 实时位置更新地图
        if status['position']:
            st.session_state.drone_pos = [status['position'][0], status['position'][1]]
        
        # 进度条
        st.progress(status['progress'] / 100)
        
        # 实时高度和速度图表
        if 'altitude_data' not in st.session_state:
            st.session_state.altitude_data = []
        
        st.session_state.altitude_data.append({
            'time': elapsed,
            'altitude': start_altitude + np.sin(elapsed * 0.5) * 5,  # 模拟高度变化
            'speed': flight_speed
        })
        
        if len(st.session_state.altitude_data) > 50:
            st.session_state.altitude_data = st.session_state.altitude_data[-50:]
        
        fig = go.Figure()
        df_plot = pd.DataFrame(st.session_state.altitude_data)
        fig.add_trace(go.Scatter(x=df_plot['time'], y=df_plot['altitude'],
                                 mode='lines', name='飞行高度', line=dict(color='green')))
        fig.add_trace(go.Scatter(x=df_plot['time'], y=df_plot['speed'],
                                 mode='lines', name='飞行速度', line=dict(color='blue')))
        fig.update_layout(title="实时飞行数据",
                         xaxis_title="时间 (秒)",
                         yaxis_title="数值")
        st.plotly_chart(fig, use_container_width=True)
        
        # 自动刷新
        if status['progress'] >= 100:
            st.success("✅ 飞行任务完成！")
            if st.button("🔄 新任务"):
                st.session_state.is_flying = False
                st.session_state.simulator = None
                st.session_state.altitude_data = []
                st.rerun()
        else:
            time.sleep(0.5)
            st.rerun()
    else:
        st.success("✅ 飞行任务完成！")
        if st.button("🔄 新任务"):
            st.session_state.is_flying = False
            st.session_state.simulator = None
            st.session_state.altitude_data = []
            st.rerun()
