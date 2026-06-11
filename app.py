# 在文件开头的 session state 初始化部分添加
if 'simulator' not in st.session_state:
    st.session_state.simulator = None
if 'is_flying' not in st.session_state:
    st.session_state.is_flying = False
if 'start_time' not in st.session_state:
    st.session_state.start_time = None
if 'altitude_data' not in st.session_state:
    st.session_state.altitude_data = []
if 'flight_status' not in st.session_state:
    st.session_state.flight_status = None

# ... 中间代码保持不变 ...

# 修改后的"开始飞行监控"按钮逻辑
with col3:
    if st.button("🎮 开始飞行监控", use_container_width=True, type="primary"):
        if st.session_state.flight_plan:
            # 重置飞行状态
            st.session_state.is_flying = True
            st.session_state.simulator = DroneSimulator(
                st.session_state.flight_plan['waypoints'],
                flight_speed
            )
            st.session_state.start_time = datetime.now()
            st.session_state.altitude_data = []
            st.session_state.flight_status = "飞行中"
            st.success("飞行监控已启动！")
            st.rerun()
        else:
            st.error("❌ 请先规划航线")

# 修改后的飞行监控界面（完全重写）
if st.session_state.is_flying and st.session_state.simulator:
    st.markdown("---")
    st.subheader("🎮 飞行监控界面")
    
    # 创建占位符用于实时更新
    status_placeholder = st.empty()
    metrics_placeholder = st.empty()
    chart_placeholder = st.empty()
    progress_placeholder = st.empty()
    control_placeholder = st.empty()
    
    # 飞行循环
    flight_running = True
    update_count = 0
    
    while flight_running and st.session_state.is_flying:
        # 更新飞行状态
        still_flying = st.session_state.simulator.update()
        
        # 获取当前状态
        status = st.session_state.simulator.get_status()
        elapsed = (datetime.now() - st.session_state.start_time).total_seconds()
        
        # 更新仪表盘
        with metrics_placeholder.container():
            col1, col2, col3, col4, col5 = st.columns(5)
            
            with col1:
                st.metric("📍 当前航点", f"{status['current_waypoint']}/{status['total_waypoints']}")
            with col2:
                st.metric("⚡ 飞行速度", f"{flight_speed} m/s")
            with col3:
                st.metric("⏱️ 已用时间", f"{elapsed:.1f} 秒")
            with col4:
                st.metric("📏 剩余距离", f"{status['remaining_distance']:.1f} 米")
            with col5:
                remaining_time = status['remaining_distance'] / flight_speed if flight_speed > 0 else 0
                st.metric("⏰ 预计到达", f"{remaining_time:.1f} 秒")
        
        # 第二行仪表
        with status_placeholder.container():
            col1, col2, col3, col4, col5 = st.columns(5)
            with col1:
                # 电池模拟（10分钟飞行时间）
                battery = max(0, 100 - (elapsed / 600) * 100)
                st.metric("🔋 电量", f"{battery:.0f}%")
            with col2:
                st.metric("📊 完成进度", f"{status['progress']:.1f}%")
            with col3:
                st.metric("🎯 当前航段进度", f"{status['segment_progress']:.1f}%")
            with col4:
                # 安全状态检测
                is_safe = st.session_state.simulator.check_safety() if hasattr(st.session_state.simulator, 'check_safety') else True
                st.metric("🛡️ 安全状态", "✅ 正常" if is_safe else "⚠️ 警告")
            with col5:
                st.metric("📍 当前位置", f"{status['position'][0]:.4f}, {status['position'][1]:.4f}")
        
        # 更新地图上的无人机位置
        if status['position']:
            st.session_state.drone_pos = [status['position'][0], status['position'][1]]
        
        # 进度条
        with progress_placeholder:
            st.progress(int(status['progress']))
            st.caption(f"飞行进度: {status['progress']:.1f}%")
        
        # 记录实时数据
        current_altitude = 100 + np.sin(elapsed * 0.5) * 10  # 模拟高度变化
        st.session_state.altitude_data.append({
            'time': elapsed,
            'altitude': current_altitude,
            'speed': flight_speed,
            'distance_remaining': status['remaining_distance']
        })
        
        # 只保留最近100个数据点
        if len(st.session_state.altitude_data) > 100:
            st.session_state.altitude_data = st.session_state.altitude_data[-100:]
        
        # 实时图表
        with chart_placeholder.container():
            if len(st.session_state.altitude_data) > 1:
                df_plot = pd.DataFrame(st.session_state.altitude_data)
                
                fig = go.Figure()
                fig.add_trace(go.Scatter(x=df_plot['time'], y=df_plot['altitude'],
                                         mode='lines', name='飞行高度 (米)', 
                                         line=dict(color='green', width=2)))
                fig.add_trace(go.Scatter(x=df_plot['time'], y=df_plot['speed'],
                                         mode='lines', name='飞行速度 (m/s)', 
                                         line=dict(color='blue', width=2)))
                fig.add_trace(go.Scatter(x=df_plot['time'], y=df_plot['distance_remaining'] / 10,
                                         mode='lines', name='剩余距离/10', 
                                         line=dict(color='orange', width=2, dash='dash')))
                
                fig.update_layout(title="📈 实时飞行数据",
                                 xaxis_title="飞行时间 (秒)",
                                 yaxis_title="数值",
                                 hovermode='x unified',
                                 legend=dict(yanchor="top", y=0.99, xanchor="left", x=0.01))
                fig.update_xgrid(show=True, gridwidth=1, gridcolor='lightgray')
                fig.update_ygrid(show=True, gridwidth=1, gridcolor='lightgray')
                
                st.plotly_chart(fig, use_container_width=True)
        
        # 控制按钮
        with control_placeholder:
            col1, col2, col3 = st.columns(3)
            with col2:
                if st.button("🛑 终止飞行", key="stop_flight"):
                    st.session_state.is_flying = False
                    st.warning("飞行已终止")
                    flight_running = False
                    break
        
        # 检查是否完成
        if status['progress'] >= 100 or not still_flying:
            st.session_state.is_flying = False
            with status_placeholder.container():
                st.success("🎉 飞行任务圆满完成！🎉")
                st.balloons()
                
                # 显示飞行总结
                st.markdown("### 📊 飞行总结")
                col1, col2, col3 = st.columns(3)
                with col1:
                    st.metric("总飞行时间", f"{elapsed:.1f} 秒")
                with col2:
                    st.metric("总飞行距离", f"{status['completed_distance']:.1f} 米")
                with col3:
                    avg_speed = status['completed_distance'] / elapsed if elapsed > 0 else 0
                    st.metric("平均速度", f"{avg_speed:.1f} m/s")
            
            if st.button("🔄 开始新任务", key="new_mission"):
                st.session_state.is_flying = False
                st.session_state.simulator = None
                st.session_state.altitude_data = []
                st.session_state.drone_pos = None
                st.rerun()
            flight_running = False
            break
        
        # 控制刷新率（每秒更新10次）
        time.sleep(0.1)
        update_count += 1
        
        # 每10次更新刷新一次页面（约1秒刷新一次界面）
        if update_count % 10 == 0:
            st.rerun()
