# TEKNOFEST 2025 Su Altı Roket Aracı - OPTİMİZASYON ÖNERİLERİ

## 📊 Proje Durumu ve Analiz

### ✅ **GÜÇLÜ YÖNLER**
- Kapsamlı görev yönetimi sistemi (Mission Manager)
- Profesyonel hardware pin mapping standardı
- Tam test coverage (her sistem için ayrı testler)
- MAVLink entegrasyonu ile Pixhawk haberleşmesi
- Güvenlik sistemleri (emergency stop, fail-safe)
- Video demo organizasyonu (şartname uyumlu)
- X-konfigürasyon fin kontrol matrisi

### 🔧 **GELİŞTİRİLECEK ALANLAR**
- PID kontrolcü parametrelerinin optimizasyonu
- Sensor fusion implementasyonu
- Path planning algoritması
- Real-time telemetri dashboard
- Adaptive control systems
- Machine learning tabanlı optimizasyon

---

## 🎯 **PRİORİTE 1: KRİTİK OPTİMİZASYONLAR**

### 1.1 PID Kontrolcü İyileştirmeleri

#### Mevcut Durum:
```python
CONTROL_PARAMS = {
    'depth_pid': {'kp': 100.0, 'ki': 5.0, 'kd': 30.0},
    'heading_pid': {'kp': 3.0, 'ki': 0.1, 'kd': 0.5},
    'position_pid': {'kp': 2.0, 'ki': 0.05, 'kd': 0.3}
}
```

#### Önerilen İyileştirmeler:

**A) Adaptive PID Gain Scheduling**
```python
# Derinliğe göre PID parametrelerini ayarla
def get_adaptive_depth_pid(current_depth):
    if current_depth < 1.0:  # Yüzeye yakın
        return {'kp': 80.0, 'ki': 3.0, 'kd': 25.0}
    elif current_depth < 3.0:  # Orta derinlik
        return {'kp': 120.0, 'ki': 8.0, 'kd': 40.0}
    else:  # Derin
        return {'kp': 150.0, 'ki': 12.0, 'kd': 50.0}
```

**B) Anti-Windup Protection**
```python
class AdvancedPIDController:
    def __init__(self, kp, ki, kd, max_output=500, integral_limit=None):
        # Integral windup protection
        self.integral_limit = integral_limit or (max_output / ki if ki > 0 else float('inf'))
        
    def update(self, setpoint, measurement):
        # ... PID hesaplama ...
        
        # Anti-windup clamp
        if abs(self.integral) > self.integral_limit:
            self.integral = math.copysign(self.integral_limit, self.integral)
```

**C) Derivative Filtering**
```python
def filtered_derivative(self, error, dt, alpha=0.1):
    """Low-pass filtered derivative to reduce noise"""
    raw_derivative = (error - self.previous_error) / dt
    self.filtered_derivative = alpha * raw_derivative + (1 - alpha) * self.filtered_derivative
    return self.filtered_derivative
```

### 1.2 Sensor Fusion İyileştirmesi

#### **Extended Kalman Filter (EKF) Implementasyonu**

```python
import numpy as np
from scipy.linalg import inv

class UnderwaterEKF:
    """Extended Kalman Filter for underwater vehicle state estimation"""
    
    def __init__(self):
        # State vector: [x, y, z, vx, vy, vz, roll, pitch, yaw]
        self.state = np.zeros(9)
        self.P = np.eye(9) * 0.1  # Covariance matrix
        
        # Process noise
        self.Q = np.diag([0.1, 0.1, 0.05, 0.2, 0.2, 0.1, 0.01, 0.01, 0.02])
        
        # Measurement noise
        self.R = {
            'gps': np.diag([1.0, 1.0]),  # GPS position noise
            'depth': 0.01,               # Depth sensor noise
            'imu': np.diag([0.01, 0.01, 0.05])  # IMU attitude noise
        }
    
    def predict(self, dt, control_input):
        """Prediction step"""
        # State transition function F
        F = np.eye(9)
        F[0:3, 3:6] = np.eye(3) * dt  # Position = velocity * dt
        
        # Control input matrix B (thrust, fin deflections)
        B = self.get_control_matrix(dt)
        
        # Predict state
        self.state = F @ self.state + B @ control_input
        
        # Predict covariance
        self.P = F @ self.P @ F.T + self.Q
    
    def update_gps(self, gps_measurement):
        """Update with GPS measurement"""
        if gps_measurement is None:
            return
            
        # Measurement function (GPS measures x, y position)
        H = np.zeros((2, 9))
        H[0, 0] = 1  # x
        H[1, 1] = 1  # y
        
        # Innovation
        z = gps_measurement - H @ self.state
        S = H @ self.P @ H.T + self.R['gps']
        K = self.P @ H.T @ inv(S)
        
        # Update state and covariance
        self.state += K @ z
        self.P = (np.eye(9) - K @ H) @ self.P
    
    def update_depth(self, depth_measurement):
        """Update with depth sensor measurement"""
        if depth_measurement is None:
            return
            
        # Depth measures z position (negative)
        H = np.zeros((1, 9))
        H[0, 2] = -1  # -z
        
        z = depth_measurement - H @ self.state
        S = H @ self.P @ H.T + self.R['depth']
        K = self.P @ H.T / S
        
        self.state += K * z
        self.P -= K @ H @ self.P
    
    def get_estimated_position(self):
        """Get current position estimate"""
        return self.state[0:3]
    
    def get_estimated_velocity(self):
        """Get current velocity estimate"""
        return self.state[3:6]
```

### 1.3 Path Planning A* Algoritması

```python
import heapq
import math
from typing import List, Tuple, Optional

class AStarPathPlanner:
    """A* path planning for underwater navigation"""
    
    def __init__(self, grid_size=1.0, max_depth=10.0):
        self.grid_size = grid_size  # meters
        self.max_depth = max_depth
        self.obstacles = set()  # Set of (x, y, z) obstacle coordinates
        
    def add_obstacle(self, x, y, z, radius=2.0):
        """Add circular obstacle"""
        grid_x = int(x / self.grid_size)
        grid_y = int(y / self.grid_size)
        grid_z = int(z / self.grid_size)
        grid_radius = int(radius / self.grid_size)
        
        for dx in range(-grid_radius, grid_radius + 1):
            for dy in range(-grid_radius, grid_radius + 1):
                for dz in range(-grid_radius, grid_radius + 1):
                    if dx*dx + dy*dy + dz*dz <= grid_radius*grid_radius:
                        self.obstacles.add((grid_x + dx, grid_y + dy, grid_z + dz))
    
    def heuristic(self, a: Tuple[int, int, int], b: Tuple[int, int, int]) -> float:
        """3D Euclidean distance heuristic"""
        return math.sqrt(sum((a[i] - b[i])**2 for i in range(3)))
    
    def get_neighbors(self, node: Tuple[int, int, int]) -> List[Tuple[int, int, int]]:
        """Get valid 3D neighbors"""
        x, y, z = node
        neighbors = []
        
        # 26-connected neighborhood (3x3x3 - center)
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                for dz in [-1, 0, 1]:
                    if dx == dy == dz == 0:
                        continue
                        
                    nx, ny, nz = x + dx, y + dy, z + dz
                    
                    # Check bounds (depth limit)
                    if nz > 0 or nz < -int(self.max_depth / self.grid_size):
                        continue
                    
                    # Check obstacles
                    if (nx, ny, nz) not in self.obstacles:
                        neighbors.append((nx, ny, nz))
        
        return neighbors
    
    def find_path(self, start: Tuple[float, float, float], 
                  goal: Tuple[float, float, float]) -> Optional[List[Tuple[float, float, float]]]:
        """Find optimal path from start to goal"""
        
        # Convert to grid coordinates
        start_grid = (int(start[0] / self.grid_size), 
                     int(start[1] / self.grid_size), 
                     int(start[2] / self.grid_size))
        goal_grid = (int(goal[0] / self.grid_size), 
                    int(goal[1] / self.grid_size), 
                    int(goal[2] / self.grid_size))
        
        # A* algorithm
        open_set = [(0, start_grid)]
        came_from = {}
        g_score = {start_grid: 0}
        f_score = {start_grid: self.heuristic(start_grid, goal_grid)}
        
        while open_set:
            current_f, current = heapq.heappop(open_set)
            
            if current == goal_grid:
                # Reconstruct path
                path = []
                while current in came_from:
                    # Convert back to world coordinates
                    world_pos = (current[0] * self.grid_size, 
                               current[1] * self.grid_size, 
                               current[2] * self.grid_size)
                    path.append(world_pos)
                    current = came_from[current]
                path.reverse()
                return path
            
            for neighbor in self.get_neighbors(current):
                # Movement cost (higher for diagonal and vertical movement)
                move_cost = 1.0
                if abs(neighbor[0] - current[0]) + abs(neighbor[1] - current[1]) > 1:
                    move_cost = math.sqrt(2)  # Diagonal
                if neighbor[2] != current[2]:
                    move_cost *= 1.2  # Vertical movement penalty
                
                tentative_g = g_score.get(current, float('inf')) + move_cost
                
                if tentative_g < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self.heuristic(neighbor, goal_grid)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))
        
        return None  # No path found

# Mission planner'a entegrasyon örneği
class SmartMissionPlanner:
    def __init__(self):
        self.path_planner = AStarPathPlanner()
        self.ekf = UnderwaterEKF()
        
    def plan_mission_1_path(self, start_pos, obstacles=None):
        """Plan optimal path for Mission 1"""
        # Add known obstacles (shoreline, boats, etc.)
        if obstacles:
            for obs in obstacles:
                self.path_planner.add_obstacle(obs['x'], obs['y'], obs['z'], obs['radius'])
        
        # Mission 1 waypoints
        waypoint_1 = (start_pos[0] + 10, start_pos[1], -2.0)  # 10m straight
        waypoint_2 = (start_pos[0] + 50, start_pos[1], -2.0)  # 50m offshore  
        waypoint_3 = start_pos  # Return to start
        
        # Plan path segments
        path_segments = []
        current_pos = start_pos
        
        for waypoint in [waypoint_1, waypoint_2, waypoint_3]:
            segment = self.path_planner.find_path(current_pos, waypoint)
            if segment:
                path_segments.extend(segment)
                current_pos = waypoint
            else:
                print(f"⚠️ Path planning failed to {waypoint}")
                return None
        
        return path_segments
```

---

## 🎯 **PRİORİTE 2: PERFORMANS OPTİMİZASYONLARI**

### 2.1 Multi-Threading Optimizasyonu

```python
import threading
import queue
import time
from concurrent.futures import ThreadPoolExecutor

class OptimizedMissionManager:
    """Thread-safe mission management with parallel processing"""
    
    def __init__(self):
        self.sensor_queue = queue.Queue(maxsize=100)
        self.control_queue = queue.Queue(maxsize=50)
        self.telemetry_queue = queue.Queue(maxsize=200)
        
        # Thread pools
        self.sensor_executor = ThreadPoolExecutor(max_workers=3, thread_name_prefix="sensor")
        self.control_executor = ThreadPoolExecutor(max_workers=2, thread_name_prefix="control")
        self.telemetry_executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="telemetry")
        
        self.running = False
        
    def start_parallel_processing(self):
        """Start all processing threads"""
        self.running = True
        
        # Sensor reading threads (high frequency)
        self.sensor_executor.submit(self.gps_reader_thread)
        self.sensor_executor.submit(self.depth_reader_thread)  
        self.sensor_executor.submit(self.imu_reader_thread)
        
        # Control threads (medium frequency)
        self.control_executor.submit(self.depth_control_thread)
        self.control_executor.submit(self.navigation_control_thread)
        
        # Telemetry thread (low frequency)
        self.telemetry_executor.submit(self.telemetry_thread)
    
    def gps_reader_thread(self):
        """High-frequency GPS reading"""
        while self.running:
            try:
                gps_data = self.read_gps_data()
                if gps_data:
                    self.sensor_queue.put(('gps', gps_data, time.time()))
                time.sleep(0.1)  # 10Hz
            except Exception as e:
                print(f"GPS thread error: {e}")
    
    def depth_reader_thread(self):
        """High-frequency depth reading"""  
        while self.running:
            try:
                depth_data = self.read_depth_sensor()
                if depth_data:
                    self.sensor_queue.put(('depth', depth_data, time.time()))
                time.sleep(0.05)  # 20Hz
            except Exception as e:
                print(f"Depth thread error: {e}")
    
    def sensor_fusion_processor(self):
        """Process sensor data with EKF"""
        while self.running:
            try:
                if not self.sensor_queue.empty():
                    sensor_type, data, timestamp = self.sensor_queue.get_nowait()
                    
                    # Update EKF based on sensor type
                    if sensor_type == 'gps':
                        self.ekf.update_gps(data)
                    elif sensor_type == 'depth':
                        self.ekf.update_depth(data)
                    elif sensor_type == 'imu':
                        self.ekf.update_imu(data)
                        
                time.sleep(0.02)  # 50Hz processing
            except queue.Empty:
                pass
            except Exception as e:
                print(f"Sensor fusion error: {e}")
```

### 2.2 Caching ve Memoization

```python
from functools import lru_cache
import time

class PerformanceOptimizedController:
    def __init__(self):
        self.last_calculation_time = {}
        self.calculation_cache = {}
        self.cache_timeout = 0.05  # 50ms cache
    
    @lru_cache(maxsize=128)
    def calculate_distance_bearing_cached(self, lat1, lon1, lat2, lon2):
        """Cached distance/bearing calculation"""
        return self.calculate_distance_bearing(lat1, lon1, lat2, lon2)
    
    def get_cached_or_calculate(self, cache_key, calculation_func, *args, **kwargs):
        """Generic caching mechanism with timeout"""
        current_time = time.time()
        
        if cache_key in self.calculation_cache:
            cached_value, cache_time = self.calculation_cache[cache_key]
            if current_time - cache_time < self.cache_timeout:
                return cached_value
        
        # Recalculate
        result = calculation_func(*args, **kwargs)
        self.calculation_cache[cache_key] = (result, current_time)
        return result
```

### 2.3 Hardware-Specific Optimizasyonlar

```python
import psutil
import os

class HardwareOptimizer:
    """Raspberry Pi 4B specific optimizations"""
    
    def __init__(self):
        self.cpu_count = psutil.cpu_count()
        self.memory_info = psutil.virtual_memory()
        
    def optimize_system_performance(self):
        """Apply system-level optimizations"""
        
        # CPU frequency scaling
        self.set_cpu_governor('performance')
        
        # Memory optimization
        self.optimize_memory_usage()
        
        # I/O optimization
        self.optimize_io_scheduler()
        
        # Thread priorities
        self.set_thread_priorities()
    
    def set_cpu_governor(self, governor='performance'):
        """Set CPU frequency governor"""
        try:
            os.system(f'echo {governor} | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor')
            print(f"✅ CPU governor set to: {governor}")
        except Exception as e:
            print(f"❌ Failed to set CPU governor: {e}")
    
    def optimize_memory_usage(self):
        """Optimize memory allocation"""
        # Disable swap if available (for real-time performance)
        try:
            os.system('sudo swapoff -a')
            print("✅ Swap disabled for real-time performance")
        except:
            pass
        
        # Set memory overcommit
        os.system('echo 1 | sudo tee /proc/sys/vm/overcommit_memory')
    
    def set_thread_priorities(self):
        """Set real-time priorities for critical threads"""
        try:
            import os
            import sched
            
            # Set high priority for current process
            os.nice(-10)  # Higher priority
            
            # Real-time scheduling for critical threads
            # (This requires appropriate permissions)
            print("✅ Thread priorities optimized")
        except Exception as e:
            print(f"⚠️ Could not set thread priorities: {e}")
```

---

## 🎯 **PRİORİTE 3: YANI ÖZELLIKLER VE GELIŞTİRMELER**

### 3.1 Machine Learning Tabanlı Kontrol

```python
import numpy as np
from sklearn.neural_network import MLPRegressor
import pickle

class MLPIDController:
    """Machine Learning enhanced PID controller"""
    
    def __init__(self, base_pid_params):
        self.base_pid = PIDController(**base_pid_params)
        self.ml_model = None
        self.training_data = []
        self.feature_history = []
        
    def collect_training_data(self, features, performance_metric):
        """Collect data for ML training"""
        self.training_data.append({
            'features': features,  # [error, derivative, integral, depth, speed, etc.]
            'performance': performance_metric  # Stability metric, overshoot, settling time
        })
    
    def train_adaptive_model(self):
        """Train ML model to predict optimal PID gains"""
        if len(self.training_data) < 100:
            return False
        
        X = np.array([data['features'] for data in self.training_data])
        y = np.array([data['performance'] for data in self.training_data])
        
        # Train neural network
        self.ml_model = MLPRegressor(
            hidden_layer_sizes=(64, 32, 16),
            activation='relu',
            solver='adam',
            max_iter=1000
        )
        
        self.ml_model.fit(X, y)
        
        # Save model
        with open('ml_pid_model.pkl', 'wb') as f:
            pickle.dump(self.ml_model, f)
        
        print("✅ ML-PID model trained and saved")
        return True
    
    def get_adaptive_gains(self, current_state):
        """Get ML-predicted PID gains"""
        if self.ml_model is None:
            return self.base_pid.kp, self.base_pid.ki, self.base_pid.kd
        
        features = np.array(current_state).reshape(1, -1)
        predicted_gains = self.ml_model.predict(features)[0]
        
        # Constrain gains to reasonable bounds
        kp = np.clip(predicted_gains[0], 50, 200)
        ki = np.clip(predicted_gains[1], 1, 20) 
        kd = np.clip(predicted_gains[2], 10, 80)
        
        return kp, ki, kd
```

### 3.2 Real-Time Telemetri Dashboard

```python
import asyncio
import websockets
import json
import threading
from datetime import datetime

class RealTimeTelemetryServer:
    """WebSocket-based real-time telemetry dashboard"""
    
    def __init__(self, port=8765):
        self.port = port
        self.connected_clients = set()
        self.telemetry_data = {}
        self.running = False
        
    async def register_client(self, websocket):
        """Register new client connection"""
        self.connected_clients.add(websocket)
        print(f"📱 New client connected: {websocket.remote_address}")
        
        # Send initial data
        await websocket.send(json.dumps({
            'type': 'initial_data',
            'data': self.telemetry_data
        }))
        
    async def unregister_client(self, websocket):
        """Unregister client connection"""
        self.connected_clients.remove(websocket)
        print(f"📱 Client disconnected: {websocket.remote_address}")
        
    async def broadcast_telemetry(self, data):
        """Broadcast telemetry to all connected clients"""
        if self.connected_clients:
            message = json.dumps({
                'type': 'telemetry_update',
                'timestamp': datetime.now().isoformat(),
                'data': data
            })
            
            disconnected = set()
            for websocket in self.connected_clients.copy():
                try:
                    await websocket.send(message)
                except websockets.exceptions.ConnectionClosed:
                    disconnected.add(websocket)
            
            # Remove disconnected clients
            self.connected_clients -= disconnected
    
    async def handle_client(self, websocket, path):
        """Handle client connection"""
        await self.register_client(websocket)
        try:
            async for message in websocket:
                # Handle client commands
                command = json.loads(message)
                await self.handle_command(command, websocket)
        except websockets.exceptions.ConnectionClosed:
            pass
        finally:
            await self.unregister_client(websocket)
    
    def update_telemetry(self, key, value):
        """Update telemetry data"""
        self.telemetry_data[key] = value
        
        # Broadcast to clients (non-blocking)
        if self.running and self.connected_clients:
            asyncio.create_task(self.broadcast_telemetry({key: value}))
    
    def start_server(self):
        """Start WebSocket server"""
        self.running = True
        start_server = websockets.serve(self.handle_client, "localhost", self.port)
        
        print(f"🌐 Telemetry dashboard: http://localhost:{self.port}")
        
        # Run server in separate thread
        def run_server():
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            loop.run_until_complete(start_server)
            loop.run_forever()
            
        server_thread = threading.Thread(target=run_server, daemon=True)
        server_thread.start()
        
        return server_thread

# HTML Dashboard Template
DASHBOARD_HTML = """
<!DOCTYPE html>
<html>
<head>
    <title>TEKNOFEST Underwater Vehicle - Real-Time Dashboard</title>
    <script src="https://cdn.plot.ly/plotly-latest.min.js"></script>
    <style>
        body { font-family: Arial, sans-serif; margin: 20px; }
        .metric { display: inline-block; margin: 10px; padding: 10px; border: 1px solid #ccc; border-radius: 5px; }
        .status-ok { background-color: #d4edda; }
        .status-warning { background-color: #fff3cd; }
        .status-error { background-color: #f8d7da; }
    </style>
</head>
<body>
    <h1>🚀 TEKNOFEST Su Altı Roket Aracı - Telemetri Dashboard</h1>
    
    <div id="status-panel">
        <div class="metric" id="depth-status">
            <h3>🌊 Derinlik</h3>
            <div id="depth-value">-- m</div>
        </div>
        <div class="metric" id="position-status">
            <h3>📍 Pozisyon</h3>
            <div id="position-value">--, --</div>
        </div>
        <div class="metric" id="battery-status">
            <h3>🔋 Batarya</h3>
            <div id="battery-value">-- V</div>
        </div>
        <div class="metric" id="mission-status">
            <h3>🎯 Görev</h3>
            <div id="mission-value">--</div>
        </div>
    </div>
    
    <div id="depth-chart" style="width:100%; height:300px;"></div>
    <div id="position-chart" style="width:100%; height:300px;"></div>
    
    <script>
        const ws = new WebSocket('ws://localhost:8765');
        
        // Initialize charts
        const depthTrace = { x: [], y: [], type: 'scatter', name: 'Depth' };
        Plotly.newPlot('depth-chart', [depthTrace], {title: 'Derinlik Zaman Serisi'});
        
        const positionTrace = { x: [], y: [], mode: 'markers+lines', name: 'Path' };
        Plotly.newPlot('position-chart', [positionTrace], {title: 'Araç Rotası'});
        
        ws.onmessage = function(event) {
            const message = JSON.parse(event.data);
            
            if (message.type === 'telemetry_update') {
                updateDashboard(message.data);
            }
        };
        
        function updateDashboard(data) {
            // Update metric displays
            if (data.depth !== undefined) {
                document.getElementById('depth-value').textContent = data.depth.toFixed(2) + ' m';
                
                // Update depth chart
                depthTrace.x.push(new Date());
                depthTrace.y.push(data.depth);
                Plotly.redraw('depth-chart');
            }
            
            if (data.position !== undefined) {
                document.getElementById('position-value').textContent = 
                    data.position.lat.toFixed(6) + ', ' + data.position.lon.toFixed(6);
                
                // Update position chart
                positionTrace.x.push(data.position.lon);
                positionTrace.y.push(data.position.lat);
                Plotly.redraw('position-chart');
            }
            
            if (data.battery !== undefined) {
                const batteryDiv = document.getElementById('battery-status');
                document.getElementById('battery-value').textContent = data.battery.toFixed(1) + ' V';
                
                // Color code based on voltage
                batteryDiv.className = 'metric ' + 
                    (data.battery > 20.0 ? 'status-ok' : 
                     data.battery > 19.0 ? 'status-warning' : 'status-error');
            }
        }
    </script>
</body>
</html>
"""

### 3.3 Predictive Maintenance System

```python
import numpy as np
from scipy import stats
import matplotlib.pyplot as plt

class PredictiveMaintenanceSystem:
    """AI-powered predictive maintenance for underwater vehicle"""
    
    def __init__(self):
        self.health_metrics = {
            'motor_efficiency': [],
            'servo_response_time': [],
            'battery_degradation': [],
            'depth_sensor_drift': [],
            'communication_latency': []
        }
        
        self.health_thresholds = {
            'motor_efficiency': 0.85,      # Below 85% efficiency
            'servo_response_time': 0.1,    # Above 100ms response
            'battery_degradation': 0.8,    # Below 80% capacity
            'depth_sensor_drift': 0.05,    # Above 5cm drift
            'communication_latency': 0.5   # Above 500ms latency
        }
        
    def collect_health_data(self, metric_name, value):
        """Collect health metric data"""
        if metric_name in self.health_metrics:
            self.health_metrics[metric_name].append({
                'timestamp': time.time(),
                'value': value
            })
            
            # Keep only last 1000 samples
            if len(self.health_metrics[metric_name]) > 1000:
                self.health_metrics[metric_name] = self.health_metrics[metric_name][-1000:]
    
    def analyze_trend(self, metric_name, lookback_hours=24):
        """Analyze trend for predictive maintenance"""
        if metric_name not in self.health_metrics:
            return None
            
        data = self.health_metrics[metric_name]
        if len(data) < 10:
            return None
        
        # Filter recent data
        current_time = time.time()
        cutoff_time = current_time - (lookback_hours * 3600)
        recent_data = [d for d in data if d['timestamp'] > cutoff_time]
        
        if len(recent_data) < 5:
            return None
        
        # Linear regression for trend analysis
        timestamps = np.array([d['timestamp'] for d in recent_data])
        values = np.array([d['value'] for d in recent_data])
        
        slope, intercept, r_value, p_value, std_err = stats.linregress(timestamps, values)
        
        # Predict future value
        future_time = current_time + (24 * 3600)  # 24 hours ahead
        predicted_value = slope * future_time + intercept
        
        # Determine health status
        current_value = values[-1]
        threshold = self.health_thresholds.get(metric_name, 0.5)
        
        if metric_name in ['motor_efficiency', 'battery_degradation']:
            # Lower is worse
            status = 'healthy' if current_value > threshold else 'degraded'
            predicted_status = 'healthy' if predicted_value > threshold else 'degraded'
        else:
            # Higher is worse
            status = 'healthy' if current_value < threshold else 'degraded'
            predicted_status = 'healthy' if predicted_value < threshold else 'degraded'
        
        return {
            'metric': metric_name,
            'current_value': current_value,
            'predicted_value': predicted_value,
            'trend_slope': slope,
            'confidence': abs(r_value),
            'current_status': status,
            'predicted_status': predicted_status,
            'time_to_maintenance': self.calculate_maintenance_time(slope, current_value, threshold, metric_name)
        }
    
    def calculate_maintenance_time(self, slope, current_value, threshold, metric_name):
        """Calculate estimated time until maintenance needed"""
        if abs(slope) < 1e-10:  # No significant trend
            return float('inf')
        
        if metric_name in ['motor_efficiency', 'battery_degradation']:
            if slope >= 0:  # Improving or stable
                return float('inf')
            time_to_threshold = (current_value - threshold) / abs(slope)
        else:
            if slope <= 0:  # Improving or stable
                return float('inf')
            time_to_threshold = (threshold - current_value) / slope
        
        return max(0, time_to_threshold / 3600)  # Convert to hours
    
    def generate_maintenance_report(self):
        """Generate comprehensive maintenance report"""
        report = {
            'timestamp': datetime.now().isoformat(),
            'overall_health': 'good',
            'critical_issues': [],
            'warnings': [],
            'recommendations': []
        }
        
        critical_count = 0
        
        for metric_name in self.health_metrics:
            analysis = self.analyze_trend(metric_name)
            
            if analysis:
                if analysis['current_status'] == 'degraded':
                    critical_count += 1
                    report['critical_issues'].append({
                        'component': metric_name,
                        'current_value': analysis['current_value'],
                        'threshold': self.health_thresholds[metric_name],
                        'time_to_maintenance': analysis['time_to_maintenance']
                    })
                
                elif analysis['predicted_status'] == 'degraded' and analysis['time_to_maintenance'] < 48:
                    report['warnings'].append({
                        'component': metric_name,
                        'predicted_failure_hours': analysis['time_to_maintenance'],
                        'recommended_action': f"Schedule maintenance for {metric_name}"
                    })
        
        # Overall health assessment
        if critical_count > 0:
            report['overall_health'] = 'critical'
        elif len(report['warnings']) > 0:
            report['overall_health'] = 'warning'
        
        # Generate recommendations
        report['recommendations'] = self.generate_recommendations(report)
        
        return report
    
    def generate_recommendations(self, report):
        """Generate maintenance recommendations"""
        recommendations = []
        
        for issue in report['critical_issues']:
            component = issue['component']
            
            if component == 'motor_efficiency':
                recommendations.append("🔧 Motor performance is degraded. Check propeller condition and motor bearings.")
            elif component == 'servo_response_time':
                recommendations.append("🔧 Servo response is slow. Check power supply and mechanical binding.")
            elif component == 'battery_degradation':
                recommendations.append("🔋 Battery capacity is low. Consider battery replacement.")
            elif component == 'depth_sensor_drift':
                recommendations.append("📏 Depth sensor needs recalibration. Check for fouling or damage.")
            elif component == 'communication_latency':
                recommendations.append("📡 Communication issues detected. Check antenna and connections.")
        
        return recommendations
```

---

## 📋 **İMPLEMENTASYON PLANI**

### **Kritik Optimizasyonlar**
- [ ] PID parametrelerini fine-tune et
- [ ] Adaptive PID gain scheduling implement et
- [ ] Anti-windup protection ekle
- [ ] Sensor fusion EKF başlat

### **Performans İyileştirmeleri**  
- [ ] Multi-threading optimizasyonu
- [ ] Caching ve memoization sistemi
- [ ] Hardware-specific optimizasyonlar (Raspberry Pi 4B)
- [ ] A* path planning algoritması

### **Yeni Özellikler**
- [ ] Real-time telemetri dashboard
- [ ] Predictive maintenance sistemi
- [ ] ML-enhanced PID controller
- [ ] Pixhawk Lua script finalize

### **Test ve Doğrulama**
- [ ] Tüm optimizasyonları test et
- [ ] Performance benchmarking
- [ ] Güvenlik testleri
- [ ] Video demo yeniden çek

---

## 🔧 **KURULUM TALİMATLARI**

### 1. Pixhawk Lua Script Kurulumu
```bash
# Lua script'i Pixhawk'a yükle
scp pixhawk_underwater_frame.lua pi@192.168.1.1:/home/pi/ardusub/scripts/
```

### 2. BlueOS Setup Wizard Kullanımı
```bash
# BlueOS arayüzünde Extensions > Setup Wizard
# blueos_setup_wizard.json dosyasını import et
```

### 3. Optimizasyon Paketlerini Yükle
```bash
# ML dependencies
pip install scikit-learn numpy scipy

# WebSocket server
pip install websockets asyncio

# Performance monitoring
pip install psutil
```

### 4. Sistem Optimizasyonları Uygula
```bash
# CPU governor
echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor

# Memory optimization  
sudo swapoff -a
echo 1 | sudo tee /proc/sys/vm/overcommit_memory
```

---

## 📊 **BEKLENEN GELİŞMELER**

| Optimizasyon | Mevcut | Hedef | İyileşme |
|-------------|---------|--------|----------|
| Derinlik Doğruluğu | ±20cm | ±5cm | %75 |
| Heading Kontrolü | ±5° | ±2° | %60 |
| Battery Life | 45 dk | 60 dk | %33 |
| Görev Süresi | 300s | 240s | %20 |
| Pozisyon Doğruluğu | ±2m | ±1m | %50 |

---

Bu optimizasyonlar TEKNOFEST 2025 için aracınızın performansını önemli ölçüde artıracak! 🚀 