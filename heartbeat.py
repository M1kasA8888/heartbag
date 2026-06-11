import time
import datetime
import threading
import queue

class DroneHeartbeatSimulator:
    def __init__(self):
        self.sequence_number = 0
        self.send_log = []
        self.receive_log = []
        self.timeout_log = []
        self.send_queue = queue.Queue()
        self.running = True
        
    def send_heartbeat(self):
        """模拟发送心跳信号"""
        self.sequence_number += 1
        send_time = datetime.datetime.now()
        heartbeat_data = {
            'seq': self.sequence_number,
            'timestamp': send_time.strftime("%Y-%m-%d %H:%M:%S.%f")[:-3],
            'timestamp_obj': send_time
        }
        
        self.send_log.append(heartbeat_data)
        self.send_queue.put(heartbeat_data.copy())
        
        print(f"[发送] 序号: {heartbeat_data['seq']}, 时间: {heartbeat_data['timestamp']}")
        return heartbeat_data
    
    def receive_heartbeat(self):
        """模拟接收心跳信号，检测超时"""
        while self.running:
            try:
                received = self.send_queue.get(timeout=3)
                
                receive_time = datetime.datetime.now()
                received['receive_time'] = receive_time.strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
                received['receive_time_obj'] = receive_time
                received['delay'] = round((receive_time - received['timestamp_obj']).total_seconds(), 3)
                
                self.receive_log.append(received)
                
                print(f"[接收] 序号: {received['seq']}, 延迟: {received['delay']:.3f}秒")
                
            except queue.Empty:
                timeout_time = datetime.datetime.now()
                timeout_record = {
                    'timeout_time': timeout_time.strftime("%Y-%m-%d %H:%M:%S.%f")[:-3],
                    'expected_seq': self.sequence_number + 1,
                    'last_received_seq': self.receive_log[-1]['seq'] if self.receive_log else 0,
                    'message': '连接超时: 3秒内未收到心跳'
                }
                self.timeout_log.append(timeout_record)
                print(f"[超时] {timeout_record['message']}")
    
    def get_data_for_visualization(self):
        """生成用于可视化的数据列表"""
        return {
            'send_log': self.send_log,
            'receive_log': self.receive_log,
            'timeout_log': self.timeout_log,
            'statistics': {
                'total_sent': len(self.send_log),
                'total_received': len(self.receive_log),
                'total_timeouts': len(self.timeout_log),
                'success_rate': round(len(self.receive_log) / len(self.send_log) * 100, 2) if self.send_log else 0,
                'avg_delay': round(sum([r['delay'] for r in self.receive_log]) / len(self.receive_log), 3) if self.receive_log else 0
            }
        }
