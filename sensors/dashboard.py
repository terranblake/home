#!/usr/bin/env python3
"""
Real-time Temperature Dashboard
Web-based dashboard with interactive graphs for temperature monitoring
"""

from flask import Flask, render_template, jsonify, request
import pandas as pd
import json
import os
from datetime import datetime, timedelta
import threading
import time

app = Flask(__name__)

class TemperatureDashboard:
    def __init__(self, csv_file='temperature_log.csv'):
        self.csv_file = csv_file
        self.data_cache = None
        self.last_update = None
        self.last_file_mtime = None
        self.update_interval = 1  # seconds - faster updates
        self.start_monitoring()
    
    def celsius_to_fahrenheit(self, celsius):
        """Convert Celsius to Fahrenheit"""
        return (celsius * 9/5) + 32
    
    def load_data(self):
        """Load temperature data from CSV"""
        if not os.path.exists(self.csv_file):
            return pd.DataFrame()
        
        try:
            df = pd.read_csv(self.csv_file)
            # Let pandas automatically detect the datetime format
            df['timestamp'] = pd.to_datetime(df['timestamp'], errors='coerce')
            # Drop any rows where timestamp parsing failed
            df = df.dropna(subset=['timestamp'])
            df = df.sort_values('timestamp')
            return df
        except Exception as e:
            print(f"Error loading data: {e}")
            return pd.DataFrame()
    
    def get_data(self, hours_back=24, room_filter=None, sensor_filter=None):
        """Get temperature data for the specified time range"""
        if self.data_cache is None or self.needs_update():
            self.data_cache = self.load_data()
            self.last_update = datetime.now()
        
        df = self.data_cache.copy()
        if df.empty:
            return df
        
        # Filter by time range
        cutoff_time = datetime.now() - timedelta(hours=hours_back)
        df = df[df['timestamp'] >= cutoff_time]
        
        # Filter by room if specified
        if room_filter and room_filter != [] and 'room' in df.columns:
            if isinstance(room_filter, list):
                df = df[df['room'].isin(room_filter)]
            else:
                df = df[df['room'] == room_filter]
        
        # Filter by sensor if specified
        if sensor_filter and sensor_filter != [] and 'sensor' in df.columns:
            if isinstance(sensor_filter, list):
                df = df[df['sensor'].isin(sensor_filter)]
            else:
                df = df[df['sensor'] == sensor_filter]
        
        return df
    
    def needs_update(self):
        """Check if data cache needs updating based on file modification time"""
        if self.last_update is None:
            return True
        
        try:
            current_mtime = os.path.getmtime(self.csv_file)
            if self.last_file_mtime is None or current_mtime > self.last_file_mtime:
                self.last_file_mtime = current_mtime
                return True
        except FileNotFoundError:
            pass
        
        return (datetime.now() - self.last_update).seconds > self.update_interval
    
    def get_hourly_comparison(self, days_back=7, room_filter=None, sensor_filter=None):
        """Get hourly averages across multiple days for comparison"""
        df = self.get_data(hours_back=days_back * 24, room_filter=room_filter, sensor_filter=sensor_filter)
        if df.empty:
            return {}
        
        df['hour'] = df['timestamp'].dt.hour
        df['date'] = df['timestamp'].dt.date
        
        # Check if we need to group by room and sensor
        has_room = 'room' in df.columns
        has_sensor = 'sensor' in df.columns
        
        if not has_room and not has_sensor:
            # Legacy behavior (no grouping)
            hourly_data = {}
            for date in df['date'].unique():
                day_data = df[df['date'] == date]
                hourly_avg = day_data.groupby('hour')['temp_c'].mean()
                hourly_data[str(date)] = hourly_avg.to_dict()
            return hourly_data
        
        # Group by room and sensor
        group_cols = ['date', 'hour']
        if has_room:
            group_cols.append('room')
        if has_sensor:
            group_cols.append('sensor')
        
        # Group data by date, hour, room, and sensor
        grouped = df.groupby(group_cols)['temp_c'].mean().reset_index()
        
        # Create a structure that groups data by room:sensor and then by date
        result = {
            'legacy': {},  # for backward compatibility
            'grouped': {}  # new format with room/sensor grouping
        }
        
        # Generate legacy format first (for backward compatibility)
        for date in df['date'].unique():
            date_str = str(date)
            day_data = df[df['date'] == date]
            hourly_avg = day_data.groupby('hour')['temp_c'].mean()
            result['legacy'][date_str] = hourly_avg.to_dict()
        
        # Generate new grouped format
        for _, row in grouped.iterrows():
            date = str(row['date'])
            hour = int(row['hour'])
            temp = float(round(row['temp_c'], 2))
            
            # Create key based on available columns
            if has_room and has_sensor:
                key = f"{row['room']}:{row['sensor']}"
            elif has_room:
                key = f"{row['room']}:unknown"
            else:
                key = f"unknown:{row['sensor']}"
            
            # Initialize if this room:sensor combo doesn't exist yet
            if key not in result['grouped']:
                result['grouped'][key] = {}
            
            # Initialize if this date doesn't exist for this room:sensor yet
            if date not in result['grouped'][key]:
                result['grouped'][key][date] = {}
            
            # Add hourly data for this room:sensor and date
            result['grouped'][key][date][hour] = temp
        
        # For backward compatibility, if it's a simple query with one room/sensor,
        # return the legacy format directly
        if len(result['grouped']) == 1:
            return result['legacy']
        
        return result
    
    def get_trend_comparison(self, hours=1, room_filter=None, sensor_filter=None):
        """Get temperature trends comparing current period vs previous day same period"""
        df = self.get_data(hours_back=48, room_filter=room_filter, sensor_filter=sensor_filter)  # Get 48 hours of data
        if df.empty:
            return {}
        
        now = datetime.now()
        
        # Current period
        current_start = now - timedelta(hours=hours)
        current_data = df[df['timestamp'] >= current_start]
        
        # Previous day same period
        prev_start = current_start - timedelta(days=1)
        prev_end = now - timedelta(days=1)
        prev_data = df[(df['timestamp'] >= prev_start) & (df['timestamp'] <= prev_end)]
        
        if current_data.empty or prev_data.empty:
            return {}
        
        # Calculate relative time (minutes from start of period)
        current_data = current_data.copy()
        prev_data = prev_data.copy()
        
        current_data['minutes'] = (current_data['timestamp'] - current_start).dt.total_seconds() / 60
        prev_data['minutes'] = (prev_data['timestamp'] - prev_start).dt.total_seconds() / 60
        
        # Function to resample data with room/sensor grouping
        def resample_data_grouped(data, label):
            if data.empty:
                return {}
            
            result = {
                'minutes': [],
                'temperatures': [],
                'rooms': [],
                'sensors': [],
                'label': label,
                'groups': {}
            }
            
            # Ensure we have room and sensor columns
            has_room = 'room' in data.columns
            has_sensor = 'sensor' in data.columns
            
            if not has_room and not has_sensor:
                # Legacy case - no grouping possible
                data['interval'] = (data['minutes'] // 5).astype(int)
                resampled = data.groupby('interval')['temp_c'].mean()
                
                return {
                    'minutes': (resampled.index * 5).tolist(),
                    'temperatures': resampled.round(2).tolist(),
                    'label': label,
                    'avg': round(data['temp_c'].mean(), 2),
                    'min': round(data['temp_c'].min(), 2),
                    'max': round(data['temp_c'].max(), 2)
                }
            
            # Group by room and sensor if available
            group_cols = []
            if has_room:
                group_cols.append('room')
            if has_sensor:
                group_cols.append('sensor')
                
            # Add 5-minute interval for resampling
            data['interval'] = (data['minutes'] // 5).astype(int)
            group_cols.append('interval')
            
            # Group by room, sensor, and interval
            grouped = data.groupby(group_cols).agg({
                'temp_c': 'mean',
                'minutes': 'mean'  # Average minute within interval
            }).reset_index()
            
            # Process each room/sensor combination
            for name, group in grouped.groupby(group_cols[:-1] if len(group_cols) > 1 else group_cols):
                # Create a key for this room/sensor combo
                if len(group_cols) > 2:  # both room and sensor
                    room, sensor = name
                    key = f"{room}:{sensor}"
                elif has_room:  # just room
                    room = name
                    sensor = "unknown"
                    key = f"{room}:unknown"
                else:  # just sensor
                    room = "unknown"
                    sensor = name
                    key = f"unknown:{sensor}"
                
                # Sort by interval for proper line rendering
                sorted_group = group.sort_values('interval')
                
                # Store grouped data
                result['groups'][key] = {
                    'minutes': sorted_group['minutes'].round(2).tolist(),
                    'temperatures': sorted_group['temp_c'].round(2).tolist(),
                    'room': room,
                    'sensor': sensor,
                    'avg': round(sorted_group['temp_c'].mean(), 2),
                    'min': round(sorted_group['temp_c'].min(), 2),
                    'max': round(sorted_group['temp_c'].max(), 2)
                }
                
                # Also add to flat lists for backward compatibility
                result['minutes'].extend(sorted_group['minutes'].round(2).tolist())
                result['temperatures'].extend(sorted_group['temp_c'].round(2).tolist())
                result['rooms'].extend([room] * len(sorted_group))
                result['sensors'].extend([sensor] * len(sorted_group))
            
            # Add overall statistics for backward compatibility
            result['avg'] = round(data['temp_c'].mean(), 2)
            result['min'] = round(data['temp_c'].min(), 2)
            result['max'] = round(data['temp_c'].max(), 2)
            
            return result
        
        return {
            'current': resample_data_grouped(current_data, f'Last {hours}h'),
            'previous': resample_data_grouped(prev_data, f'Previous day same {hours}h'),
            'comparison_hours': hours
        }
    
    def get_daily_summary(self, days_back=30, room_filter=None, sensor_filter=None):
        """Get daily temperature summary"""
        df = self.get_data(hours_back=days_back * 24, room_filter=room_filter, sensor_filter=sensor_filter)
        if df.empty:
            return {}
        
        df['date'] = df['timestamp'].dt.date
        daily_stats = df.groupby('date')['temp_c'].agg({
            'min': 'min',
            'max': 'max',
            'avg': 'mean',
            'count': 'count'
        }).round(2)
        
        # Convert to list format for easier frontend handling
        summary_data = []
        for date, stats in daily_stats.to_dict('index').items():
            summary_data.append({
                'date': str(date),
                'min': stats['min'],
                'max': stats['max'],
                'avg': stats['avg'],
                'count': int(stats['count'])
            })
        
        return sorted(summary_data, key=lambda x: x['date'])
    
    def start_monitoring(self):
        """Start background monitoring thread"""
        def monitor():
            while True:
                if self.needs_update():
                    self.data_cache = self.load_data()
                    self.last_update = datetime.now()
                time.sleep(0.5)  # Check every 0.5 seconds for file changes
        
        monitor_thread = threading.Thread(target=monitor, daemon=True)
        monitor_thread.start()

# Global dashboard instance
dashboard = TemperatureDashboard()

@app.route('/')
def index():
    """Main dashboard page"""
    return render_template('dashboard.html')

@app.route('/api/realtime')
def api_realtime():
    """Get real-time temperature data"""
    hours = request.args.get('hours', 1, type=int)
    room = request.args.getlist('room[]') or None
    sensors = request.args.getlist('sensors[]') or None
    unit = request.args.get('unit', 'c')  # 'c' for Celsius, 'f' for Fahrenheit
    
    df = dashboard.get_data(hours_back=hours, room_filter=room, sensor_filter=sensors)
    
    if df.empty:
        return jsonify({'timestamps': [], 'temperatures': [], 'rooms': []})
    
    temperatures = df['temp_c'].round(2)
    if unit == 'f':
        temperatures = temperatures.apply(lambda x: round(dashboard.celsius_to_fahrenheit(x), 2))
    
    return jsonify({
        'timestamps': df['timestamp'].dt.strftime('%Y-%m-%d %H:%M:%S').tolist(),
        'temperatures': temperatures.tolist(),
        'rooms': df['room'].tolist() if 'room' in df.columns else ['unknown'] * len(df),
        'sensors': df['sensor'].tolist() if 'sensor' in df.columns else ['unknown'] * len(df),
        'last_update': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
        'unit': unit
    })

@app.route('/api/trend_comparison')
def api_trend_comparison():
    """Get trend comparison data"""
    hours = request.args.get('hours', 1, type=int)
    room = request.args.getlist('room[]') or None
    sensors = request.args.getlist('sensors[]') or None
    unit = request.args.get('unit', 'c')
    
    data = dashboard.get_trend_comparison(hours=hours, room_filter=room, sensor_filter=sensors)
    
    # Convert to Fahrenheit if requested
    if unit == 'f' and data:
        for period in ['current', 'previous']:
            if period in data and 'temperatures' in data[period]:
                data[period]['temperatures'] = [round(dashboard.celsius_to_fahrenheit(t), 2) for t in data[period]['temperatures']]
                data[period]['avg'] = round(dashboard.celsius_to_fahrenheit(data[period]['avg']), 2)
                data[period]['min'] = round(dashboard.celsius_to_fahrenheit(data[period]['min']), 2)
                data[period]['max'] = round(dashboard.celsius_to_fahrenheit(data[period]['max']), 2)
    
    return jsonify(data)

@app.route('/api/hourly_comparison')
def api_hourly_comparison():
    """Get hourly temperature comparison across days"""
    days = request.args.get('days', 7, type=int)
    unit = request.args.get('unit', 'c')
    room = request.args.getlist('room[]') or None
    sensors = request.args.getlist('sensors[]') or None
    
    data = dashboard.get_hourly_comparison(days_back=days, room_filter=room, sensor_filter=sensors)
    
    # Convert to Fahrenheit if requested
    if unit == 'f':
        for date in data:
            for hour in data[date]:
                if data[date][hour] is not None:
                    data[date][hour] = round(dashboard.celsius_to_fahrenheit(data[date][hour]), 2)
    
    return jsonify(data)

@app.route('/api/daily_summary')
def api_daily_summary():
    """Get daily temperature summary"""
    days = request.args.get('days', 30, type=int)
    unit = request.args.get('unit', 'c')
    room = request.args.getlist('room[]') or None
    sensors = request.args.getlist('sensors[]') or None
    
    data = dashboard.get_daily_summary(days_back=days, room_filter=room, sensor_filter=sensors)
    
    # Convert to Fahrenheit if requested
    if unit == 'f':
        for day in data:
            day['min'] = round(dashboard.celsius_to_fahrenheit(day['min']), 2)
            day['max'] = round(dashboard.celsius_to_fahrenheit(day['max']), 2)
            day['avg'] = round(dashboard.celsius_to_fahrenheit(day['avg']), 2)
    
    return jsonify(data)

@app.route('/api/insights')
def api_insights():
    """Get intelligent insights from temperature data"""
    unit = request.args.get('unit', 'c')
    room = request.args.getlist('room[]') or None
    sensors = request.args.getlist('sensors[]') or None
    
    # Get different time periods for analysis
    current_1h = dashboard.get_data(hours_back=1, room_filter=room, sensor_filter=sensors)
    current_24h = dashboard.get_data(hours_back=24, room_filter=room, sensor_filter=sensors)
    previous_24h = dashboard.get_data(hours_back=48, room_filter=room, sensor_filter=sensors)
    week_data = dashboard.get_data(hours_back=168, room_filter=room, sensor_filter=sensors)  # 7 days
    
    def convert_temp(temp):
        if unit == 'f':
            return round(dashboard.celsius_to_fahrenheit(temp), 2)
        return round(temp, 2)
    
    insights = {
        'temperature_analysis': {},
        'trend_analysis': {},
        'system_performance': {},
        'environment_insights': {},
        'alerts': []
    }
    
    if not current_24h.empty:
        # Temperature Analysis
        current_temp = current_24h.iloc[-1]['temp_c'] if not current_24h.empty else None
        avg_24h = current_24h['temp_c'].mean()
        std_24h = current_24h['temp_c'].std()
        
        # Calculate temperature stability
        stability = "High" if std_24h < 2 else "Medium" if std_24h < 4 else "Low"
        
        # Temperature range analysis
        temp_range = current_24h['temp_c'].max() - current_24h['temp_c'].min()
        range_assessment = "Stable" if temp_range < 3 else "Moderate" if temp_range < 6 else "Variable"
        
        insights['temperature_analysis'] = {
            'current_vs_avg': convert_temp(current_temp - avg_24h) if current_temp else 0,
            'stability': stability,
            'range_24h': convert_temp(temp_range),
            'range_assessment': range_assessment,
            'std_deviation': convert_temp(std_24h)
        }
        
        # Trend Analysis
        if not current_1h.empty and len(current_1h) > 1:
            recent_trend = current_1h['temp_c'].iloc[-1] - current_1h['temp_c'].iloc[0]
            trend_direction = "Rising" if recent_trend > 0.5 else "Falling" if recent_trend < -0.5 else "Stable"
            
            # Compare current period with previous day
            current_hour = datetime.now().hour
            if not previous_24h.empty:
                prev_day_same_hour = previous_24h[previous_24h['timestamp'].dt.hour == current_hour]
                if not prev_day_same_hour.empty:
                    comparison = current_temp - prev_day_same_hour['temp_c'].mean()
                    comparison_text = f"{convert_temp(abs(comparison))}° {'warmer' if comparison > 0 else 'cooler'} than yesterday"
                else:
                    comparison_text = "No comparison data"
            else:
                comparison_text = "Insufficient data"
            
            insights['trend_analysis'] = {
                'hourly_trend': trend_direction,
                'hourly_change': convert_temp(recent_trend),
                'vs_yesterday': comparison_text
            }
    
    # System Performance Analysis
    if not current_24h.empty:
        # Calculate data collection metrics
        timestamps = pd.to_datetime(current_24h['timestamp'])
        time_diffs = timestamps.diff().dt.total_seconds().dropna()
        
        avg_interval = time_diffs.mean()
        collection_rate = len(current_24h) / 24  # readings per hour
        
        # Reliability based on expected vs actual readings
        expected_readings = 24 * 60  # assuming 1 reading per minute ideal
        reliability = min(100, (len(current_24h) / expected_readings) * 100)
        
        insights['system_performance'] = {
            'collection_rate': f"{collection_rate:.1f} readings/hour",
            'reliability': f"{reliability:.1f}%",
            'avg_interval': f"{avg_interval:.0f} seconds"
        }
    
    # Environment Insights (would be enhanced with multiple rooms)
    if not week_data.empty:
        # Daily pattern analysis
        week_data['hour'] = week_data['timestamp'].dt.hour
        hourly_avg = week_data.groupby('hour')['temp_c'].mean()
        
        warmest_hour = hourly_avg.idxmax()
        coolest_hour = hourly_avg.idxmin()
        daily_pattern = hourly_avg.max() - hourly_avg.min()
        
        insights['environment_insights'] = {
            'warmest_hour': f"{warmest_hour}:00",
            'coolest_hour': f"{coolest_hour}:00",
            'daily_variation': convert_temp(daily_pattern),
            'pattern_strength': "Strong" if daily_pattern > 4 else "Moderate" if daily_pattern > 2 else "Weak"
        }
    
    # Generate alerts
    if current_temp:
        if abs(current_temp - avg_24h) > std_24h * 2:
            insights['alerts'].append({
                'type': 'temperature_anomaly',
                'message': f"Current temperature is unusually {'high' if current_temp > avg_24h else 'low'}",
                'severity': 'medium'
            })
        
        if temp_range > 8:
            insights['alerts'].append({
                'type': 'high_variability',
                'message': "High temperature variability detected in last 24h",
                'severity': 'low'
            })
    
    if reliability < 80:
        insights['alerts'].append({
            'type': 'data_quality',
            'message': "Sensor data collection reliability below 80%",
            'severity': 'medium'
        })
    
    return jsonify(insights)

@app.route('/api/stats')
def api_stats():
    """Get current statistics"""
    unit = request.args.get('unit', 'c')
    room = request.args.getlist('room[]') or None
    sensors = request.args.getlist('sensors[]') or None
    df = dashboard.get_data(hours_back=24, room_filter=room, sensor_filter=sensors)
    
    if df.empty:
        return jsonify({'error': 'No data available'})
    
    latest = df.iloc[-1] if not df.empty else None
    
    def convert_temp(temp):
        if unit == 'f':
            return round(dashboard.celsius_to_fahrenheit(temp), 2)
        return round(temp, 2)
    
    stats = {
        'current_temp': convert_temp(latest['temp_c']) if latest is not None else None,
        'last_reading': latest['timestamp'].strftime('%Y-%m-%d %H:%M:%S') if latest is not None else None,
        'total_readings': len(df),
        'min_24h': convert_temp(df['temp_c'].min()),
        'max_24h': convert_temp(df['temp_c'].max()),
        'avg_24h': convert_temp(df['temp_c'].mean()),
        'rooms': df['room'].unique().tolist() if 'room' in df.columns else ['unknown'],
        'sensors': df['sensor'].unique().tolist() if 'sensor' in df.columns else ['unknown'],
        'unit': unit
    }
    
    return jsonify(stats)

@app.route('/api/filters')
def api_filters():
    """Get available filters (rooms and sensors)"""
    df = dashboard.data_cache
    
    if df is None or df.empty:
        return jsonify({
            'rooms': [],
            'sensors': []
        })
    
    # Get unique rooms and sensors
    rooms = df['room'].unique().tolist() if 'room' in df.columns else []
    sensors = df['sensor'].unique().tolist() if 'sensor' in df.columns else []
    
    return jsonify({
        'rooms': rooms,
        'sensors': sensors
    })

if __name__ == '__main__':
    print("Starting Temperature Dashboard...")
    print("Dashboard will be available at: http://localhost:5001")
    print("Real-time temperature monitoring with interactive graphs")
    print("=" * 60)
    
    # Create templates directory if it doesn't exist
    os.makedirs('templates', exist_ok=True)
    
    app.run(host='0.0.0.0', port=5001, debug=False, threaded=True)
