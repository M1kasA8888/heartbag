import streamlit as st
import pandas as pd
import plotly.express as px
import plotly.graph_objects as go
from datetime import datetime
import time
from heartbeat import DroneHeartbeatSimulator

st.set_page_config(
    page_title="无人机心跳监测系统",
    page_icon="🚁",
    layout="wide"
)

st.title("🚁 无人机心跳监测系统")
st.markdown("---")

# 侧边栏控制
with st.sidebar:
    st.header("⚙️ 控制面板")
    duration = st.slider("模拟运行时间 (秒)", 10, 60, 30)
    
    if st.button("🚀 开始模拟", type="primary"):
        st.session_state['simulate'] = True
        st.session_state['duration'] = duration
        st.session_state['data'] = None
    
    st.markdown("---")
    st.markdown("### 📊 功能说明")
    st.mark.info("""
    - 每秒发送一次心跳信号
    - 包含序号和时间戳
    - 3秒未收到报超时
    - 实时可视化展示
    """)

# 主区域
col1, col2, col3, col4 = st.columns(4)

if 'simulate' not in st.session_state:
    st.session_state['simulate'] = False

if st.session_state['simulate']:
    with st.spinner('模拟运行中...'):
        simulator = DroneHeartbeatSimulator()
        
        # 创建占位符用于实时显示
        status_placeholder = st.empty()
        chart_placeholder = st.empty()
        table_placeholder = st.empty()
        
        # 存储实时数据
        send_data = []
        receive_data = []
        timeout_data = []
        
        start_time = time.time()
        
        # 启动接收线程
        import threading
        receive_thread = threading.Thread(target=simulator.receive_heartbeat, daemon=True)
        receive_thread.start()
        
        # 发送心跳
        while (time.time() - start_time) < st.session_state['duration']:
            # 发送心跳
            send_heartbeat = simulator.send_heartbeat()
            send_data.append(send_heartbeat)
            
            # 更新显示
            with status_placeholder.container():
                col1.metric("📤 已发送", len(send_data))
                col2.metric("📥 已接收", len(simulator.receive_log))
                col3.metric("⚠️ 超时次数", len(simulator.timeout_log))
                
                if len(send_data) > 0:
                    success_rate = (len(simulator.receive_log) / len(send_data)) * 100
                    col4.metric("✅ 成功率", f"{success_rate:.1f}%")
            
            # 实时图表
            if len(simulator.receive_log) > 0:
                df_receive = pd.DataFrame(simulator.receive_log)
                df_receive['序号'] = df_receive['seq']
                df_receive['延迟(秒)'] = df_receive['delay']
                
                fig = px.line(df_receive, x='序号', y='延迟(秒)', 
                             title="心跳延迟实时监控",
                             markers=True)
                chart_placeholder.plotly_chart(fig, use_container_width=True)
            
            # 实时表格
            if len(simulator.receive_log) > 0:
                df_table = pd.DataFrame(simulator.receive_log[-10:])
                df_table_display = df_table[['seq', 'timestamp', 'receive_time', 'delay']]
                df_table_display.columns = ['序号', '发送时间', '接收时间', '延迟(秒)']
                table_placeholder.dataframe(df_table_display, use_container_width=True)
            
            time.sleep(1)
        
        simulator.running = False
        time.sleep(3.5)  # 等待接收线程结束
        
        # 保存最终数据
        st.session_state['data'] = simulator.get_data_for_visualization()
        st.session_state['simulate'] = False
        
        st.success("✅ 模拟完成！")
        st.rerun()

# 显示历史数据
if st.session_state.get('data') is not None:
    data = st.session_state['data']
    
    st.markdown("## 📈 详细数据分析")
    
    # 统计卡片
    col1, col2, col3, col4, col5 = st.columns(5)
    with col1:
        st.metric("总发送次数", data['statistics']['total_sent'])
    with col2:
        st.metric("总接收次数", data['statistics']['total_received'])
    with col3:
        st.metric("超时次数", data['statistics']['total_timeouts'])
    with col4:
        st.metric("成功率", f"{data['statistics']['success_rate']}%")
    with col5:
        st.metric("平均延迟", f"{data['statistics']['avg_delay']}秒")
    
    # 图表展示
    tab1, tab2, tab3 = st.tabs(["📊 延迟分析", "📋 接收记录", "⚠️ 超时记录"])
    
    with tab1:
        if data['receive_log']:
            df_delay = pd.DataFrame(data['receive_log'])
            fig1 = px.scatter(df_delay, x='seq', y='delay', 
                             title="心跳延迟分布",
                             labels={'seq': '心跳序号', 'delay': '延迟(秒)'},
                             trendline="lowess")
            fig1.add_hline(y=data['statistics']['avg_delay'], 
                          line_dash="dash", 
                          line_color="red",
                          annotation_text=f"平均延迟: {data['statistics']['avg_delay']}秒")
            st.plotly_chart(fig1, use_container_width=True)
    
    with tab2:
        if data['receive_log']:
            df_receive_full = pd.DataFrame(data['receive_log'])
            df_display = df_receive_full[['seq', 'timestamp', 'receive_time', 'delay']]
            df_display.columns = ['序号', '发送时间', '接收时间', '延迟(秒)']
            st.dataframe(df_display, use_container_width=True, height=400)
            
            # 下载按钮
            csv = df_display.to_csv(index=False)
            st.download_button(
                label="📥 下载接收记录 (CSV)",
                data=csv,
                file_name="heartbeat_receive_log.csv",
                mime="text/csv"
            )
    
    with tab3:
        if data['timeout_log']:
            df_timeout = pd.DataFrame(data['timeout_log'])
            st.dataframe(df_timeout, use_container_width=True)
            
            # 超时时间轴
            if len(data['timeout_log']) > 0:
                timeout_times = [t['timeout_time'] for t in data['timeout_log']]
                fig2 = go.Figure(data=[go.Scatter(
                    x=timeout_times,
                    y=[1]*len(timeout_times),
                    mode='markers',
                    marker=dict(size=15, color='red', symbol='x'),
                    name='超时事件'
                )])
                fig2.update_layout(title="超时事件时间轴",
                                  xaxis_title="时间",
                                  yaxis_title="",
                                  showlegend=False)
                st.plotly_chart(fig2, use_container_width=True)
        else:
            st.info("🎉 没有发生超时，连接稳定！")
    
    # 重置按钮
    if st.button("🔄 重新模拟"):
        st.session_state['data'] = None
        st.rerun()
