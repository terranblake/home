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
            df['timestamp'] = pd.to_datetime(df['timestamp'])
            df = df.sort_values('timestamp')
            return df
        except Exception as e:
            print(f"Error loading data: {e}")
            return pd.DataFrame()
    
    def get_data(self, hours_back=24, room_filter=None):
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
        if room_filter and 'room' in df.columns:
            df = df[df['room'] == room_filter]
        
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
    
    def get_hourly_comparison(self, days_back=7):
        """Get hourly averages across multiple days for comparison"""
        df = self.get_data(hours_back=days_back * 24)
        if df.empty:
            return {}
        
        df['hour'] = df['timestamp'].dt.hour
        df['date'] = df['timestamp'].dt.date
        
        hourly_data = {}
        for date in df['date'].unique():
            day_data = df[df['date'] == date]
            hourly_avg = day_data.groupby('hour')['temp_c'].mean()
            hourly_data[str(date)] = hourly_avg.to_dict()
        
        return hourly_data
    
    def get_trend_comparison(self, hours=1):
        """Get temperature trends comparing current period vs previous day same period"""
        df = self.get_data(hours_back=48)  # Get 48 hours of data
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
        
        # Resample to 5-minute intervals for smoother comparison
        def resample_data(data, label):
            if data.empty:
                return {}
            
            # Group by 5-minute intervals
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
        
        return {
            'current': resample_data(current_data, f'Last {hours}h'),
            'previous': resample_data(prev_data, f'Previous day same {hours}h'),
            'comparison_hours': hours
        }
    
    def get_daily_summary(self, days_back=30):
        """Get daily temperature summary"""
        df = self.get_data(hours_back=days_back * 24)
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
    room = request.args.get('room', None)
    unit = request.args.get('unit', 'c')  # 'c' for Celsius, 'f' for Fahrenheit
    
    df = dashboard.get_data(hours_back=hours, room_filter=room)
    
    if df.empty:
        return jsonify({'timestamps': [], 'temperatures': [], 'rooms': []})
    
    temperatures = df['temp_c'].round(2)
    if unit == 'f':
        temperatures = temperatures.apply(lambda x: round(dashboard.celsius_to_fahrenheit(x), 2))
    
    return jsonify({
        'timestamps': df['timestamp'].dt.strftime('%Y-%m-%d %H:%M:%S').tolist(),
        'temperatures': temperatures.tolist(),
        'rooms': df['room'].tolist() if 'room' in df.columns else ['unknown'] * len(df),
        'last_update': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
        'unit': unit
    })

@app.route('/api/trend_comparison')
def api_trend_comparison():
    """Get trend comparison data"""
    hours = request.args.get('hours', 1, type=int)
    unit = request.args.get('unit', 'c')
    
    data = dashboard.get_trend_comparison(hours=hours)
    
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
    
    data = dashboard.get_hourly_comparison(days_back=days)
    
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
    
    data = dashboard.get_daily_summary(days_back=days)
    
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
    
    # Get different time periods for analysis
    current_1h = dashboard.get_data(hours_back=1)
    current_24h = dashboard.get_data(hours_back=24)
    previous_24h = dashboard.get_data(hours_back=48)
    week_data = dashboard.get_data(hours_back=168)  # 7 days
    
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
    df = dashboard.get_data(hours_back=24)
    
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
        'unit': unit
    }
    
    return jsonify(stats)

if __name__ == '__main__':
    print("Starting Temperature Dashboard...")
    print("Dashboard will be available at: http://localhost:5001")
    print("Real-time temperature monitoring with interactive graphs")
    print("=" * 60)
    
    # Create templates directory if it doesn't exist
    os.makedirs('templates', exist_ok=True)
    
    app.run(host='0.0.0.0', port=5001, debug=False, threaded=True)
