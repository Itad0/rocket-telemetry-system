"""
Report Generator Module for Flight Summaries
"""

import os
import json
import logging
from datetime import datetime, timedelta
from typing import Dict, Any, List, Optional
from dataclasses import dataclass

logger = logging.getLogger(__name__)


@dataclass
class FlightSummary:
    """Flight summary data structure"""
    flight_id: str
    launch_time: datetime
    landing_time: datetime
    max_altitude: float
    max_velocity: float
    max_acceleration: float
    flight_duration: float
    distance_traveled: float
    anomalies_detected: List[Dict[str, Any]]
    mission_success: bool
    data_quality_score: float


class ReportGenerator:
    """Generates flight reports and summaries"""
    
    def __init__(self, database_manager):
        self.db = database_manager
        self.output_dir = "reports/generated"
        os.makedirs(self.output_dir, exist_ok=True)
        logger.info("Report generator initialized")
        
    async def generate_flight_report(self, telemetry_data: Dict[str, Any], 
                                   metrics: Dict[str, Any]) -> str:
        """Generate comprehensive flight report"""
        
        logger.info("Generating flight report...")
        
        # Analyze flight data
        flight_summary = await self._analyze_flight_data(telemetry_data)
        
        # Create report content
        report_content = self._create_report_content(flight_summary, metrics)
        
        # Save report
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = f"flight_report_{flight_summary.flight_id}_{timestamp}.json"
        filepath = os.path.join(self.output_dir, filename)
        
        with open(filepath, 'w') as f:
            json.dump(report_content, f, indent=2, default=str)
            
        logger.info(f"Flight report generated: {filepath}")
        return filepath
        
    async def _analyze_flight_data(self, telemetry_data: Dict[str, Any]) -> FlightSummary:
        """Analyze flight data and create summary"""
        
        # For demo, create sample summary
        flight_id = telemetry_data.get('flight_id', 'DEMO001')
        
        # Get flight data from database
        flight_data = await self.db.get_flight_data(flight_id)
        
        if not flight_data:
            # Use current telemetry for basic summary
            return FlightSummary(
                flight_id=flight_id,
                launch_time=datetime.now() - timedelta(minutes=10),
                landing_time=datetime.now(),
                max_altitude=telemetry_data.get('altitude', 1000),
                max_velocity=150.0,
                max_acceleration=25.0,
                flight_duration=600.0,
                distance_traveled=5000.0,
                anomalies_detected=[],
                mission_success=True,
                data_quality_score=0.95
            )
            
        # Analyze actual flight data
        altitudes = [d.get('altitude', 0) for d in flight_data]
        max_altitude = max(altitudes) if altitudes else 0
        
        # Detect anomalies
        anomalies = self._detect_anomalies(flight_data)
        
        return FlightSummary(
            flight_id=flight_id,
            launch_time=datetime.now() - timedelta(minutes=10),
            landing_time=datetime.now(),
            max_altitude=max_altitude,
            max_velocity=150.0,
            max_acceleration=25.0,
            flight_duration=600.0,
            distance_traveled=5000.0,
            anomalies_detected=anomalies,
            mission_success=len(anomalies) < 5,
            data_quality_score=0.95
        )
        
    def _detect_anomalies(self, flight_data: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        """Detect anomalies in flight data"""
        anomalies = []
        
        # Check for altitude spikes
        for i in range(1, len(flight_data)):
            if 'altitude' in flight_data[i] and 'altitude' in flight_data[i-1]:
                alt_change = abs(flight_data[i]['altitude'] - flight_data[i-1]['altitude'])
                if alt_change > 100:  # More than 100m change
                    anomalies.append({
                        'type': 'ALTITUDE_SPIKE',
                        'timestamp': flight_data[i].get('timestamp'),
                        'description': f'Sudden altitude change: {alt_change:.1f}m',
                        'severity': 'HIGH'
                    })
                    
        # Check for GPS issues
        gps_weak = [d for d in flight_data if d.get('satellites', 10) < 4]
        if gps_weak:
            anomalies.append({
                'type': 'GPS_WEAK_SIGNAL',
                'timestamp': gps_weak[0].get('timestamp'),
                'description': f'GPS signal weak in {len(gps_weak)} samples',
                'severity': 'MEDIUM'
            })
            
        return anomalies[:10]  # Limit to 10 anomalies
        
    def _create_report_content(self, summary: FlightSummary, 
                             metrics: Dict[str, Any]) -> Dict[str, Any]:
        """Create report content structure"""
        
        return {
            'report_metadata': {
                'generated_at': datetime.now().isoformat(),
                'report_version': '1.0',
                'generator': 'RocketTelemetrySystem'
            },
            'flight_summary': {
                'flight_id': summary.flight_id,
                'launch_time': summary.launch_time.isoformat(),
                'landing_time': summary.landing_time.isoformat(),
                'duration_seconds': summary.flight_duration,
                'mission_success': summary.mission_success,
                'data_quality_score': summary.data_quality_score
            },
            'performance_metrics': {
                'max_altitude_m': summary.max_altitude,
                'max_velocity_ms': summary.max_velocity,
                'max_acceleration_ms2': summary.max_acceleration,
                'distance_traveled_m': summary.distance_traveled
            },
            'system_metrics': {
                'packets_received': metrics.get('packets_received', 0),
                'packets_lost': metrics.get('packets_lost', 0),
                'packet_loss_rate': metrics.get('packets_lost', 0) / max(1, metrics.get('packets_received', 1)),
                'uptime_seconds': metrics.get('uptime', 0)
            },
            'anomalies': summary.anomalies_detected,
            'anomaly_count': len(summary.anomalies_detected)
        }
        
    async def generate_anomaly_report(self) -> str:
        """Generate anomaly-focused report"""
        
        # Get recent anomalies
        anomalies = await self.db.get_recent_anomalies(hours=24)
        
        report = {
            'report_type': 'anomaly_analysis',
            'generated_at': datetime.now().isoformat(),
            'time_range': '24_hours',
            'total_anomalies': len(anomalies),
            'anomalies_by_type': {},
            'anomalies': anomalies
        }
        
        # Group by type
        for anomaly in anomalies:
            atype = anomaly.get('type', 'UNKNOWN')
            if atype not in report['anomalies_by_type']:
                report['anomalies_by_type'][atype] = 0
            report['anomalies_by_type'][atype] += 1
            
        # Save report
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = f"anomaly_report_{timestamp}.json"
        filepath = os.path.join(self.output_dir, filename)
        
        with open(filepath, 'w') as f:
            json.dump(report, f, indent=2)
            
        logger.info(f"Anomaly report generated: {filepath}")
        return filepath
        
    async def generate_system_report(self, metrics: Dict[str, Any]) -> str:
        """Generate system health report"""
        
        report = {
            'report_type': 'system_health',
            'generated_at': datetime.now().isoformat(),
            'system_metrics': metrics,
            'status': 'healthy' if metrics.get('uptime', 0) > 0 else 'unknown'
        }
        
        # Save report
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = f"system_report_{timestamp}.json"
        filepath = os.path.join(self.output_dir, filename)
        
        with open(filepath, 'w') as f:
            json.dump(report, f, indent=2)
            
        logger.info(f"System report generated: {filepath}")
        return filepath