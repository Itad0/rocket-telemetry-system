"""
Database Manager Module for Rocket Telemetry System
"""

import asyncio
import logging
from typing import Dict, Any, List, Optional
from datetime import datetime, timedelta
import json

logger = logging.getLogger(__name__)


class DatabaseManager:
    """Manages telemetry data storage and retrieval"""
    
    def __init__(self, database_url: str):
        self.db_url = database_url
        self.connected = False
        # In-memory storage for demo (would be actual DB in production)
        self.telemetry_storage = []
        self.report_metadata = []
        logger.info(f"Database manager initialized with URL: {database_url}")
        
    async def connect(self):
        """Connect to database"""
        # In production, this would establish actual DB connection
        self.connected = True
        logger.info("Database connected")
        
    async def disconnect(self):
        """Disconnect from database"""
        self.connected = False
        logger.info("Database disconnected")
        
    async def store_telemetry(self, data: Dict[str, Any]):
        """Store telemetry data"""
        if not self.connected:
            await self.connect()
            
        # Add timestamp if not present
        if 'timestamp' not in data:
            data['timestamp'] = datetime.now().isoformat()
            
        # Store in memory (would be DB insert in production)
        self.telemetry_storage.append(data)
        
        # Keep only last 10000 records in memory
        if len(self.telemetry_storage) > 10000:
            self.telemetry_storage = self.telemetry_storage[-10000:]
            
        logger.debug(f"Stored telemetry: {data.get('timestamp')}")
        
    async def get_flight_data(self, flight_id: str) -> List[Dict[str, Any]]:
        """Get all data for a specific flight"""
        # Filter by flight_id
        flight_data = [
            record for record in self.telemetry_storage
            if record.get('flight_id') == flight_id
        ]
        
        logger.info(f"Retrieved {len(flight_data)} records for flight {flight_id}")
        return flight_data
        
    async def get_recent_telemetry(self, minutes: int = 30) -> List[Dict[str, Any]]:
        """Get recent telemetry data"""
        cutoff_time = datetime.now() - timedelta(minutes=minutes)
        
        recent_data = []
        for record in self.telemetry_storage:
            try:
                # Parse timestamp
                if isinstance(record.get('timestamp'), str):
                    record_time = datetime.fromisoformat(record['timestamp'].replace('Z', '+00:00'))
                else:
                    record_time = datetime.fromtimestamp(record.get('timestamp', 0) / 1000)
                    
                if record_time > cutoff_time:
                    recent_data.append(record)
            except:
                continue
                
        logger.info(f"Retrieved {len(recent_data)} records from last {minutes} minutes")
        return recent_data
        
    async def get_telemetry_range(self, start_time: datetime, end_time: datetime) -> List[Dict[str, Any]]:
        """Get telemetry data within a time range"""
        range_data = []
        
        for record in self.telemetry_storage:
            try:
                if isinstance(record.get('timestamp'), str):
                    record_time = datetime.fromisoformat(record['timestamp'].replace('Z', '+00:00'))
                else:
                    record_time = datetime.fromtimestamp(record.get('timestamp', 0) / 1000)
                    
                if start_time <= record_time <= end_time:
                    range_data.append(record)
            except:
                continue
                
        return range_data
        
    async def store_report_metadata(self, metadata: Dict[str, Any]):
        """Store report metadata"""
        metadata['created_at'] = datetime.now().isoformat()
        self.report_metadata.append(metadata)
        logger.info(f"Stored report metadata for flight {metadata.get('flight_id')}")
        
    async def get_recent_anomalies(self, hours: int = 24) -> List[Dict[str, Any]]:
        """Get recent anomalies"""
        # For demo, return some sample anomalies
        return [
            {
                'type': 'GPS_WEAK_SIGNAL',
                'timestamp': (datetime.now() - timedelta(hours=2)).isoformat(),
                'description': 'GPS satellites dropped below 4',
                'severity': 'MEDIUM'
            },
            {
                'type': 'ALTITUDE_SPIKE',
                'timestamp': (datetime.now() - timedelta(hours=1)).isoformat(),
                'description': 'Sudden altitude change detected',
                'severity': 'HIGH'
            }
        ]
        
    def get_stats(self) -> Dict[str, Any]:
        """Get database statistics"""
        return {
            'total_records': len(self.telemetry_storage),
            'total_reports': len(self.report_metadata),
            'connected': self.connected,
            'oldest_record': self.telemetry_storage[0]['timestamp'] if self.telemetry_storage else None,
            'newest_record': self.telemetry_storage[-1]['timestamp'] if self.telemetry_storage else None
        }