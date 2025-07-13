#!/usr/bin/env python3
"""
ESP32 Temperature Server with Immediate Data Ingestion
Fast boot for production deployment with minimal data loss
"""

from http.server import HTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse, parse_qs
import json
import datetime
import csv
import os
from collections import deque
import statistics
import requests
import time
import threading
import signal
import sys

class TempHandler(BaseHTTPRequestHandler):
    # Class variables to store recent readings
    readings_1min = deque(maxlen=60)    # 1 reading per second for 1 minute
    readings_1hour = deque(maxlen=3600) # 1 reading per second for 1 hour
    readings_4hours = deque(maxlen=14400) # 1 reading per second for 4 hours
    
    # Anomaly detection and safe mode control
    anomaly_log = deque(maxlen=100)     # Store recent anomalies
    safe_mode_commands = deque(maxlen=50) # Store recent safe mode commands
    last_safe_command_time = {}         # Track last command time per node
    
    # Thresholds for anomaly detection
    HIGH_STD_THRESHOLD = 2.0            # °C - High standard deviation threshold
    RAPID_RISE_THRESHOLD = 5.0          # °C - Rapid temperature rise threshold  
    RAPID_RISE_TIME_WINDOW = 300        # seconds - Time window for rapid rise detection
    SAFE_MODE_COOLDOWN = 900            # seconds - Minimum time between safe mode commands per node
    
    # ESP-NOW receiver endpoint for safe mode commands (if available)
    ESPNOW_RECEIVER_IP = "192.168.8.200"  # Update this to your receiver's IP
    ESPNOW_RECEIVER_PORT = 8081            # HTTP endpoint on receiver for safe mode commands
    
    # Server state
    _historical_data_loaded = False
    _server_ready = False
    
    @classmethod
    def start_background_loader(cls):
        """Start background thread to load historical data without blocking server startup"""
        def load_historical():
            try:
                print("📚 Loading historical data in background...")
                cls.load_historical_data()
                cls._historical_data_loaded = True
                print("✅ Historical data loaded successfully")
            except Exception as e:
                print(f"⚠️  Warning: Failed to load historical data: {e}")
                print("📡 Server continuing with fresh data collection")
            finally:
                cls._server_ready = True
        
        # Start background thread
        loader_thread = threading.Thread(target=load_historical, daemon=True)
        loader_thread.start()
        print("🚀 Server starting immediately - data ingestion ready!")
        return loader_thread
    
    @classmethod
    def load_historical_data(cls):
        """Load existing CSV data into memory for rolling averages with robust error handling"""
        csv_file = 'temperature_log.csv'
        if not os.path.isfile(csv_file):
            print("No existing temperature log found - starting fresh")
            return
        
        loaded_count = 0
        error_count = 0
        
        try:
            with open(csv_file, 'r') as f:
                reader = csv.DictReader(f)
                
                print(f"🔍 Loading historical temperature data from {csv_file}")
                
                # Process each row with error handling
                for line_num, row in enumerate(reader, start=2):  # Start at 2 because header is line 1
                    try:
                        # Validate required fields exist
                        required_fields = ['timestamp', 'temp_c', 'temp_f']
                        for field in required_fields:
                            if field not in row or not row[field]:
                                raise ValueError(f"Missing required field: {field}")
                        
                        # Parse and validate data
                        timestamp = datetime.datetime.fromisoformat(row['timestamp'])
                        temp_c = float(row['temp_c'])
                        temp_f = float(row['temp_f'])
                        
                        # Basic sanity checks
                        if not (-50 <= temp_c <= 150):  # Reasonable temperature range
                            raise ValueError(f"Temperature out of range: {temp_c}°C")
                        
                        temp_data = {
                            'timestamp': timestamp,
                            'temp_c': temp_c,
                            'temp_f': temp_f
                        }
                        
                        # Add to deques (only keep recent data)
                        cls.readings_4hours.append(temp_data)
                        loaded_count += 1
                        
                    except (ValueError, KeyError, TypeError) as e:
                        error_count += 1
                        if error_count <= 5:  # Only show first 5 errors to avoid spam
                            print(f"⚠️  Warning: Skipping corrupted data at line {line_num}: {e}")
                        elif error_count == 6:
                            print(f"⚠️  ... and {error_count-5} more errors (suppressing further warnings)")
                        
                        # Continue processing despite errors
                        continue
                
                # Now populate the shorter-term deques from the 4-hour data
                recent_data = list(cls.readings_4hours)
                if recent_data:
                    # Sort by timestamp to ensure proper ordering
                    recent_data.sort(key=lambda x: x['timestamp'])
                    
                    # Get cutoff times
                    now = datetime.datetime.now()
                    hour_ago = now - datetime.timedelta(hours=1)
                    minute_ago = now - datetime.timedelta(minutes=1)
                    
                    # Populate shorter-term deques
                    for data in recent_data:
                        if data['timestamp'] >= hour_ago:
                            cls.readings_1hour.append(data)
                        if data['timestamp'] >= minute_ago:
                            cls.readings_1min.append(data)
                
                print(f"✅ Successfully loaded {loaded_count} temperature readings")
                if error_count > 0:
                    print(f"⚠️  Skipped {error_count} corrupted/invalid rows")
                print(f"📊 Data in memory:")
                print(f"   Last 1min:  {len(cls.readings_1min)} samples")
                print(f"   Last 1hr:   {len(cls.readings_1hour)} samples") 
                print(f"   Last 4hrs:  {len(cls.readings_4hours)} samples")
                print("📚 Historical data ready for rolling averages!")
                
        except Exception as e:
            print(f"❌ Error loading historical data: {e}")
            print("🔄 Starting with empty data - server will continue normally")
    
    def do_GET(self):
        parsed = urlparse(self.path)
        
        if parsed.path == '/ingest':
            # Parse query parameters
            params = parse_qs(parsed.query)
            
            if 'tempC' in params:
                # Extract all parameters
                room = params.get('room', ['unknown'])[0]
                sensor = params.get('sensor', ['unknown'])[0]
                temp_c = float(params['tempC'][0])
                temp_f = float(params.get('tempF', [temp_c * 9.0 / 5.0 + 32.0])[0])
                raw_adc = int(params.get('rawADC', [0])[0])
                voltage = float(params.get('voltage', [0.0])[0])
                resistance = float(params.get('resistance', [0.0])[0])
                beta = float(params.get('beta', [0.0])[0])
                r0 = float(params.get('r0', [0.0])[0])
                r_fixed = float(params.get('rFixed', [0.0])[0])
                
                timestamp = datetime.datetime.now().isoformat()
                
                # Add to rolling averages
                temp_data = {
                    'timestamp': datetime.datetime.now(),
                    'temp_c': temp_c,
                    'temp_f': temp_f
                }
                self.readings_1min.append(temp_data)
                self.readings_1hour.append(temp_data)
                self.readings_4hours.append(temp_data)
                
                # Calculate averages and stats (use available data even if historical not loaded)
                stats = self.calculate_stats()
                
                # Check for temperature anomalies and potentially trigger safe mode
                # Only do full anomaly detection if we have sufficient data
                if self._historical_data_loaded or len(self.readings_1hour) > 30:
                    is_anomaly, anomalies, recommended_action = self.check_temperature_anomalies(room, temp_c, stats)
                    
                    if is_anomaly and recommended_action in ["safe", "emergency"]:
                        # Send safe mode command to the problematic node
                        safe_command_sent = self.send_safe_mode_command(room, recommended_action)
                        if safe_command_sent:
                            print(f"🛡️  Safe mode activated for {room} due to temperature anomalies")
                else:
                    is_anomaly, anomalies = False, []
                
                # Log to console with stats (always show immediate data)
                print(f"\n[{timestamp}] {room} - {sensor}: {temp_c:.2f}°C ({temp_f:.2f}°F)")
                print(f"  Raw ADC: {raw_adc}, Voltage: {voltage:.3f}V, Resistance: {resistance:.1f}Ω")
                if is_anomaly:
                    print(f"  🚨 ANOMALY DETECTED: {', '.join(anomalies)}")
                
                # Show loading status if historical data still loading
                if not self._historical_data_loaded:
                    print(f"  📚 Historical data loading... (using {len(self.readings_1hour)} recent samples)")
                
                print(f"  📊 ROLLING AVERAGES:")
                print(f"    Last 1min:  {stats['avg_1min_c']:.2f}°C ({stats['avg_1min_f']:.2f}°F) [{len(self.readings_1min)} samples]")
                print(f"    Last 1hr:   {stats['avg_1hr_c']:.2f}°C ({stats['avg_1hr_f']:.2f}°F) [{len(self.readings_1hour)} samples]")
                print(f"    Last 4hrs:  {stats['avg_4hr_c']:.2f}°C ({stats['avg_4hr_f']:.2f}°F) [{len(self.readings_4hours)} samples]")
                print(f"  📈 TEMPERATURE TRENDS:")
                print(f"    Min/Max 1hr: {stats['min_1hr_c']:.2f}°C / {stats['max_1hr_c']:.2f}°C (Δ{stats['range_1hr_c']:.2f}°C)")
                print(f"    Std Dev 1hr: {stats['std_1hr_c']:.2f}°C")
                if stats['trend']:
                    print(f"    Trend: {stats['trend']} ({stats['trend_change']:.2f}°C over last {stats['trend_minutes']}min)")
                print(f"  ⏱️  Total readings: {stats['total_readings']}")
                print("-" * 80)
                
                # Save to CSV file
                self.save_to_csv(timestamp, room, sensor, temp_c, temp_f, raw_adc, voltage, resistance, beta, r0, r_fixed)
                
                # Check for anomalies
                self.detect_anomalies(room, sensor, temp_c, temp_f, stats)
                
                # Send success response
                self.send_response(200)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                response = {
                    'status': 'success',
                    'timestamp': timestamp,
                    'room': room,
                    'sensor': sensor,
                    'temp_c': temp_c,
                    'temp_f': temp_f,
                    'stats': stats
                }
                self.wfile.write(json.dumps(response).encode())
            else:
                # Missing temperature parameter
                self.send_response(400)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                response = {'status': 'error', 'message': 'Missing tempC parameter'}
                self.wfile.write(json.dumps(response).encode())
        
        elif parsed.path == '/':
            # Simple status page
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()
            html = """
            <html>
            <head><title>Temperature Server</title></head>
            <body>
                <h1>ESP32 Temperature Server</h1>
                <p>Server is running and ready to receive temperature data.</p>
                <p>Send GET requests to: <code>/ingest?tempC=25.5</code></p>
            </body>
            </html>
            """
            self.wfile.write(html.encode())
        else:
            # 404 for other paths
            self.send_response(404)
            self.end_headers()
    
    def calculate_stats(self):
        """Calculate rolling averages and statistics"""
        stats = {
            'total_readings': len(self.readings_4hours),
            'avg_1min_c': 0, 'avg_1min_f': 0,
            'avg_1hr_c': 0, 'avg_1hr_f': 0,
            'avg_4hr_c': 0, 'avg_4hr_f': 0,
            'min_1hr_c': 0, 'max_1hr_c': 0, 'range_1hr_c': 0,
            'std_1hr_c': 0,
            'trend': None, 'trend_change': 0, 'trend_minutes': 0
        }
        
        # 1 minute averages
        if self.readings_1min:
            temps_c = [r['temp_c'] for r in self.readings_1min]
            temps_f = [r['temp_f'] for r in self.readings_1min]
            stats['avg_1min_c'] = statistics.mean(temps_c)
            stats['avg_1min_f'] = statistics.mean(temps_f)
        
        # 1 hour averages and stats
        if self.readings_1hour:
            temps_c = [r['temp_c'] for r in self.readings_1hour]
            temps_f = [r['temp_f'] for r in self.readings_1hour]
            stats['avg_1hr_c'] = statistics.mean(temps_c)
            stats['avg_1hr_f'] = statistics.mean(temps_f)
            stats['min_1hr_c'] = min(temps_c)
            stats['max_1hr_c'] = max(temps_c)
            stats['range_1hr_c'] = stats['max_1hr_c'] - stats['min_1hr_c']
            if len(temps_c) > 1:
                stats['std_1hr_c'] = statistics.stdev(temps_c)
        
        # 4 hour averages
        if self.readings_4hours:
            temps_c = [r['temp_c'] for r in self.readings_4hours]
            temps_f = [r['temp_f'] for r in self.readings_4hours]
            stats['avg_4hr_c'] = statistics.mean(temps_c)
            stats['avg_4hr_f'] = statistics.mean(temps_f)
        
        # Calculate trend (compare last 5 minutes to previous 5 minutes)
        if len(self.readings_1hour) >= 600:  # At least 10 minutes of data
            recent_temps = [r['temp_c'] for r in list(self.readings_1hour)[-300:]]  # Last 5 minutes
            previous_temps = [r['temp_c'] for r in list(self.readings_1hour)[-600:-300]]  # Previous 5 minutes
            
            recent_avg = statistics.mean(recent_temps)
            previous_avg = statistics.mean(previous_temps)
            change = recent_avg - previous_avg
            
            stats['trend_change'] = change
            stats['trend_minutes'] = 5
            
            if abs(change) < 0.1:
                stats['trend'] = "STABLE"
            elif change > 0:
                stats['trend'] = "RISING"
            else:
                stats['trend'] = "FALLING"
        
        return stats
    
    def save_to_csv(self, timestamp, room, sensor, temp_c, temp_f, raw_adc, voltage, resistance, beta, r0, r_fixed):
        """Save temperature reading and all sensor data to CSV file"""
        csv_file = 'temperature_log.csv'
        file_exists = os.path.isfile(csv_file)
        
        with open(csv_file, 'a', newline='') as f:
            writer = csv.writer(f)
            if not file_exists:
                # Write header if file is new
                writer.writerow(['timestamp', 'room', 'sensor', 'temp_c', 'temp_f', 'raw_adc', 'voltage', 'resistance', 'beta', 'r0', 'r_fixed'])
            writer.writerow([timestamp, room, sensor, temp_c, temp_f, raw_adc, voltage, resistance, beta, r0, r_fixed])
    
    def detect_anomalies(self, room, sensor, temp_c, temp_f, stats):
        """Detect anomalies in temperature readings and trigger safe mode if necessary"""
        current_time = time.time()
        node_id = f"{room}-{sensor}"
        
        # Check for high standard deviation anomaly
        if stats['std_1hr_c'] > self.HIGH_STD_THRESHOLD:
            anomaly = {
                'type': 'high_std',
                'room': room,
                'sensor': sensor,
                'temp_c': temp_c,
                'temp_f': temp_f,
                'std_dev': stats['std_1hr_c'],
                'timestamp': datetime.datetime.now().isoformat()
            }
            self.log_anomaly(anomaly)
            self.trigger_safe_mode(node_id, "High temperature variation detected")
        
        # Check for rapid rise anomaly
        if len(self.readings_1hour) >= 600:  # Ensure we have at least 10 minutes of data
            recent_temps = [r['temp_c'] for r in list(self.readings_1hour)[-300:]]  # Last 5 minutes
            previous_temps = [r['temp_c'] for r in list(self.readings_1hour)[-600:-300]]  # Previous 5 minutes
            
            recent_avg = statistics.mean(recent_temps)
            previous_avg = statistics.mean(previous_temps)
            change = recent_avg - previous_avg
            
            # Rapid rise detected
            if change > self.RAPID_RISE_THRESHOLD:
                anomaly = {
                    'type': 'rapid_rise',
                    'room': room,
                    'sensor': sensor,
                    'temp_c': temp_c,
                    'temp_f': temp_f,
                    'change': change,
                    'timestamp': datetime.datetime.now().isoformat()
                }
                self.log_anomaly(anomaly)
                self.trigger_safe_mode(node_id, "Rapid temperature rise detected")
    
    def log_anomaly(self, anomaly):
        """Log anomaly details to console and file"""
        print(f"🚨 ANOMALY DETECTED: {anomaly['type']} - {anomaly['room']} - {anomaly['sensor']}")
        print(f"  Temperature: {anomaly['temp_c']}°C / {anomaly['temp_f']}°F")
        print(f"  Details: {anomaly}")
        
        # Save to CSV log file
        csv_file = 'anomaly_log.csv'
        file_exists = os.path.isfile(csv_file)
        
        with open(csv_file, 'a', newline='') as f:
            writer = csv.writer(f)
            if not file_exists:
                # Write header if file is new
                writer.writerow(['timestamp', 'room', 'sensor', 'temp_c', 'temp_f', 'anomaly_type', 'details'])
            writer.writerow([anomaly['timestamp'], anomaly['room'], anomaly['sensor'], anomaly['temp_c'], anomaly['temp_f'], anomaly['type'], json.dumps(anomaly)])
    
    def trigger_safe_mode(self, node_id, reason):
        """Trigger safe mode for the given node ID by sending a command via ESP-NOW or HTTP"""
        current_time = time.time()
        
        # Check cooldown period
        if node_id in self.last_safe_command_time:
            elapsed_time = current_time - self.last_safe_command_time[node_id]
            if elapsed_time < self.SAFE_MODE_COOLDOWN:
                print(f"Safe mode command suppressed for {node_id} - cooldown active")
                return  # Suppress command, cooldown active
        
        # Send safe mode command (HTTP request to ESP-NOW receiver)
        try:
            url = f"http://{self.ESPNOW_RECEIVER_IP}:{self.ESPNOW_RECEIVER_PORT}/safemode"
            payload = {
                'node_id': node_id,
                'reason': reason,
                'timestamp': datetime.datetime.now().isoformat()
            }
            response = requests.post(url, json=payload, timeout=5)
            
            if response.status_code == 200:
                print(f"✅ Safe mode triggered for {node_id}: {reason}")
                
                # Log the safe mode command
                command = {
                    'node_id': node_id,
                    'reason': reason,
                    'timestamp': datetime.datetime.now().isoformat()
                }
                self.safe_mode_commands.append(command)
                
                # Update last command time
                self.last_safe_command_time[node_id] = current_time
            else:
                print(f"❌ Failed to trigger safe mode for {node_id} - HTTP error {response.status_code}")
        except Exception as e:
            print(f"❌ Failed to trigger safe mode for {node_id} - {e}")
    
    @classmethod
    def check_temperature_anomalies(cls, room, temp_c, stats):
        """
        Check for temperature anomalies that might indicate battery issues
        Returns: (is_anomaly, anomaly_type, recommended_action)
        """
        anomalies = []
        recommended_action = "normal"
        
        # Check 1: High standard deviation indicates erratic readings
        if stats['std_1hr_c'] > cls.HIGH_STD_THRESHOLD and len(cls.readings_1hour) > 30:
            anomalies.append(f"High temperature variance: {stats['std_1hr_c']:.2f}°C std dev")
            recommended_action = "safe"
        
        # Check 2: Rapid temperature rise (often indicates failing battery/sensor)
        if len(cls.readings_1hour) >= cls.RAPID_RISE_TIME_WINDOW:
            recent_window = list(cls.readings_1hour)[-cls.RAPID_RISE_TIME_WINDOW:]
            if len(recent_window) >= cls.RAPID_RISE_TIME_WINDOW:
                start_temp = statistics.mean([r['temp_c'] for r in recent_window[:30]])  # First 30 readings
                end_temp = statistics.mean([r['temp_c'] for r in recent_window[-30:]])   # Last 30 readings
                temp_rise = end_temp - start_temp
                
                if temp_rise > cls.RAPID_RISE_THRESHOLD:
                    anomalies.append(f"Rapid temperature rise: {temp_rise:.2f}°C in {cls.RAPID_RISE_TIME_WINDOW}s")
                    recommended_action = "safe" if temp_rise < cls.RAPID_RISE_THRESHOLD * 1.5 else "emergency"
        
        # Check 3: Temperature readings way outside normal range
        if len(cls.readings_4hours) > 100:
            recent_4hr_temps = [r['temp_c'] for r in cls.readings_4hours]
            mean_4hr = statistics.mean(recent_4hr_temps)
            std_4hr = statistics.stdev(recent_4hr_temps) if len(recent_4hr_temps) > 1 else 0
            
            # Z-score based outlier detection
            if std_4hr > 0:
                z_score = abs(temp_c - mean_4hr) / std_4hr
                if z_score > 4:  # Very unusual reading
                    anomalies.append(f"Temperature outlier: {temp_c:.2f}°C (z-score: {z_score:.2f})")
                    recommended_action = "safe"
        
        # Log anomaly if found
        if anomalies:
            anomaly_data = {
                'timestamp': datetime.datetime.now(),
                'room': room,
                'temp_c': temp_c,
                'anomalies': anomalies,
                'action': recommended_action,
                'stats': {
                    'std_1hr': stats['std_1hr_c'],
                    'trend': stats.get('trend', 'unknown'),
                    'trend_change': stats.get('trend_change', 0)
                }
            }
            cls.anomaly_log.append(anomaly_data)
            
            print(f"🚨 TEMPERATURE ANOMALY DETECTED for {room}:")
            for anomaly in anomalies:
                print(f"   • {anomaly}")
            print(f"   • Recommended action: {recommended_action.upper()}")
            
            return True, anomalies, recommended_action
        
        return False, [], "normal"
    
    @classmethod
    def send_safe_mode_command(cls, room, action="safe", duration_minutes=0):
        """
        Send safe mode command to ESP-NOW receiver
        Returns: success (bool)
        """
        current_time = time.time()
        
        # Check cooldown period
        if room in cls.last_safe_command_time:
            time_since_last = current_time - cls.last_safe_command_time[room]
            if time_since_last < cls.SAFE_MODE_COOLDOWN:
                print(f"⏳ Safe mode command for {room} skipped (cooldown: {int(cls.SAFE_MODE_COOLDOWN - time_since_last)}s remaining)")
                return False
        
        # Prepare command data
        command_data = {
            'target_node': room,
            'command': 1 if action == "safe" else 2 if action == "emergency" else 0,
            'duration_minutes': duration_minutes,
            'timestamp': datetime.datetime.now().isoformat(),
            'reason': "server_anomaly_detection"
        }
        
        # Try to send to ESP-NOW receiver via HTTP
        try:
            url = f"http://{cls.ESPNOW_RECEIVER_IP}:{cls.ESPNOW_RECEIVER_PORT}/safe_mode"
            response = requests.post(url, json=command_data, timeout=5)
            
            if response.status_code == 200:
                cls.last_safe_command_time[room] = current_time
                cls.safe_mode_commands.append({
                    'timestamp': datetime.datetime.now(),
                    'room': room,
                    'action': action,
                    'duration': duration_minutes,
                    'success': True
                })
                print(f"✅ Safe mode command sent successfully to {room}: {action}")
                return True
            else:
                print(f"❌ Safe mode command failed: HTTP {response.status_code}")
                return False
                
        except requests.exceptions.RequestException as e:
            print(f"❌ Failed to send safe mode command to {room}: {e}")
            # Log failed attempt but don't spam
            cls.safe_mode_commands.append({
                'timestamp': datetime.datetime.now(),
                'room': room,
                'action': action,
                'duration': duration_minutes,
                'success': False,
                'error': str(e)
            })
            return False
    
    def log_message(self, format, *args):
        """Override to customize logging"""
        return  # Suppress default request logging

def signal_handler(sig, frame):
    """Graceful shutdown handler"""
    print('\n🛑 Shutting down server...')
    sys.exit(0)

if __name__ == '__main__':
    # Register signal handler for graceful shutdown
    signal.signal(signal.SIGINT, signal_handler)
    
    server_address = ('', 8080)  # Listen on all interfaces, port 8080
    
    print("🌡️  ESP32 Temperature Server - Fast Boot Mode")
    print("=" * 60)
    print(f"📡 Listening immediately on http://localhost:8080")
    print("ESP32 should send data to: http://YOUR_COMPUTER_IP:8080/ingest")
    print("� Data ingestion ready - no startup delay!")
    print("� Historical data loading in background...")
    print("🚨 Anomaly detection configuration:")
    print(f"   • High std dev threshold: {TempHandler.HIGH_STD_THRESHOLD}°C")
    print(f"   • Rapid rise threshold: {TempHandler.RAPID_RISE_THRESHOLD}°C in {TempHandler.RAPID_RISE_TIME_WINDOW}s")
    print(f"   • Safe mode cooldown: {TempHandler.SAFE_MODE_COOLDOWN}s per node")
    print(f"🔗 ESP-NOW receiver endpoint: http://{TempHandler.ESPNOW_RECEIVER_IP}:{TempHandler.ESPNOW_RECEIVER_PORT}")
    print("🛡️  Automatic safe mode commands enabled")
    print("=" * 60)
    
    # Create HTTP server
    httpd = HTTPServer(server_address, TempHandler)
    
    # Start background historical data loading (non-blocking)
    loader_thread = TempHandler.start_background_loader()
    
    print("✅ Server is LIVE - accepting temperature data now!")
    print("📊 Statistics will improve as historical data loads")
    print("Press Ctrl+C to stop")
    print("=" * 60)
    
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\n🛑 Server stopped gracefully.")
    finally:
        httpd.server_close()
        # Give background thread a moment to finish
        if loader_thread.is_alive():
            loader_thread.join(timeout=1)
