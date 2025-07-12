#!/usr/bin/env python3
"""
Simple HTTP server to receive temperature data from ESP32
"""

from http.server import HTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse, parse_qs
import json
import datetime
import csv
import os
from collections import deque
import statistics

class TempHandler(BaseHTTPRequestHandler):
    # Class variables to store recent readings
    readings_1min = deque(maxlen=60)    # 1 reading per second for 1 minute
    readings_1hour = deque(maxlen=3600) # 1 reading per second for 1 hour
    readings_4hours = deque(maxlen=14400) # 1 reading per second for 4 hours
    
    @classmethod
    def load_historical_data(cls):
        """Load existing CSV data into memory for rolling averages"""
        csv_file = 'temperature_log.csv'
        if not os.path.isfile(csv_file):
            print("No existing temperature log found - starting fresh")
            return
        
        try:
            with open(csv_file, 'r') as f:
                reader = csv.DictReader(f)
                rows = list(reader)
                
                print(f"Loading {len(rows)} historical temperature readings...")
                
                # Load the most recent readings into our deques
                for row in rows[-14400:]:  # Last 4 hours max
                    timestamp = datetime.datetime.fromisoformat(row['timestamp'])
                    temp_c = float(row['temp_c'])
                    temp_f = float(row['temp_f'])
                    
                    temp_data = {
                        'timestamp': timestamp,
                        'temp_c': temp_c,
                        'temp_f': temp_f
                    }
                    
                    # Add to all relevant deques
                    cls.readings_4hours.append(temp_data)
                    if len(rows) - rows.index(row) <= 3600:  # Last hour
                        cls.readings_1hour.append(temp_data)
                    if len(rows) - rows.index(row) <= 60:    # Last minute
                        cls.readings_1min.append(temp_data)
                
                print(f"Loaded into memory:")
                print(f"  Last 1min:  {len(cls.readings_1min)} samples")
                print(f"  Last 1hr:   {len(cls.readings_1hour)} samples") 
                print(f"  Last 4hrs:  {len(cls.readings_4hours)} samples")
                print("Historical data ready for rolling averages!")
                
        except Exception as e:
            print(f"Error loading historical data: {e}")
            print("Starting with empty data...")
    
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
                
                # Calculate averages and stats
                stats = self.calculate_stats()
                
                # Log to console with stats
                print(f"\n[{timestamp}] {room} - {sensor}: {temp_c:.2f}°C ({temp_f:.2f}°F)")
                print(f"  Raw ADC: {raw_adc}, Voltage: {voltage:.3f}V, Resistance: {resistance:.1f}Ω")
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
    
    def log_message(self, format, *args):
        """Override to customize logging"""
        return  # Suppress default request logging

if __name__ == '__main__':
    server_address = ('', 8080)  # Listen on all interfaces, port 8080
    
    print("Temperature server starting...")
    print(f"Listening on http://localhost:8080")
    print("ESP32 should send data to: http://YOUR_COMPUTER_IP:8080/ingest")
    print("📊 Server will display rolling averages for 1min, 1hr, and 4hrs")
    print("📈 Trend analysis and statistics included")
    print("=" * 80)
    
    # Load historical data before starting server
    TempHandler.load_historical_data()
    print("=" * 80)
    
    httpd = HTTPServer(server_address, TempHandler)
    print("Press Ctrl+C to stop")
    print("=" * 80)
    
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\nServer stopped.")
        httpd.server_close()
