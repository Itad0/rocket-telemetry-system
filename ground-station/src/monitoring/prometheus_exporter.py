"""
Prometheus Metrics Exporter for Telemetry System
"""

import logging
import time
from typing import Dict, Any
from datetime import datetime

logger = logging.getLogger(__name__)


class PrometheusExporter:
    """Exports system metrics in Prometheus format"""
    
    def __init__(self, port: int):
        self.port = port
        self.metrics = {
            'telemetry_packets_total': 0,
            'telemetry_altitude_meters': 0,
            'telemetry_temperature_celsius': 0,
            'system_uptime_seconds': 0,
            'active_faults_total': 0,
            'packet_loss_rate': 0.0
        }
        self.start_time = time.time()
        self.running = False
        
        logger.info(f"Prometheus exporter initialized on port {port}")
        
    async def start(self):
        """Start the metrics server"""
        self.running = True
        logger.info(f"Prometheus exporter started on port {self.port}")
        # In production, this would start an actual HTTP server
        
    async def stop(self):
        """Stop the metrics server"""
        self.running = False
        logger.info("Prometheus exporter stopped")
        
    def update_telemetry_metrics(self, telemetry: Dict[str, Any]):
        """Update metrics from telemetry data"""
        self.metrics['telemetry_packets_total'] += 1
        
        if 'altitude' in telemetry:
            self.metrics['telemetry_altitude_meters'] = telemetry['altitude']
            
        if 'temperature' in telemetry:
            self.metrics['telemetry_temperature_celsius'] = telemetry['temperature']
            
        logger.debug(f"Updated telemetry metrics: {self.metrics['telemetry_packets_total']} packets")
        
    def update_system_metrics(self, system_data: Dict[str, Any]):
        """Update system metrics"""
        if 'uptime' in system_data:
            self.metrics['system_uptime_seconds'] = system_data['uptime']
            
        if 'packets_received' in system_data and 'packets_lost' in system_data:
            total = system_data['packets_received'] + system_data['packets_lost']
            if total > 0:
                self.metrics['packet_loss_rate'] = system_data['packets_lost'] / total
                
        if 'active_faults' in system_data:
            self.metrics['active_faults_total'] = len(system_data.get('active_faults', []))
            
    def get_metrics(self) -> str:
        """Get metrics in Prometheus format"""
        # Update uptime
        self.metrics['system_uptime_seconds'] = time.time() - self.start_time
        
        # Format metrics in Prometheus exposition format
        output = []
        output.append("# HELP telemetry_packets_total Total number of telemetry packets received")
        output.append("# TYPE telemetry_packets_total counter")
        output.append(f"telemetry_packets_total {self.metrics['telemetry_packets_total']}")
        
        output.append("# HELP telemetry_altitude_meters Current altitude in meters")
        output.append("# TYPE telemetry_altitude_meters gauge")
        output.append(f"telemetry_altitude_meters {self.metrics['telemetry_altitude_meters']}")
        
        output.append("# HELP telemetry_temperature_celsius Current temperature in Celsius")
        output.append("# TYPE telemetry_temperature_celsius gauge")
        output.append(f"telemetry_temperature_celsius {self.metrics['telemetry_temperature_celsius']}")
        
        output.append("# HELP system_uptime_seconds System uptime in seconds")
        output.append("# TYPE system_uptime_seconds counter")
        output.append(f"system_uptime_seconds {self.metrics['system_uptime_seconds']:.2f}")
        
        output.append("# HELP packet_loss_rate Packet loss rate (0-1)")
        output.append("# TYPE packet_loss_rate gauge")
        output.append(f"packet_loss_rate {self.metrics['packet_loss_rate']:.4f}")
        
        output.append("# HELP active_faults_total Number of active faults")
        output.append("# TYPE active_faults_total gauge")
        output.append(f"active_faults_total {self.metrics['active_faults_total']}")
        
        return '\n'.join(output)
        
    def increment_counter(self, metric_name: str, value: float = 1.0):
        """Increment a counter metric"""
        if metric_name in self.metrics:
            self.metrics[metric_name] += value
            
    def set_gauge(self, metric_name: str, value: float):
        """Set a gauge metric"""
        self.metrics[metric_name] = value
        
    def get_stats(self) -> Dict[str, Any]:
        """Get exporter statistics"""
        return {
            'port': self.port,
            'running': self.running,
            'metrics_count': len(self.metrics),
            'uptime': time.time() - self.start_time
        }