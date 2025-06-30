"""
MQTT Handler Module for Telemetry System
"""

import asyncio
import logging
import json
from typing import Dict, Any, Callable, Optional

logger = logging.getLogger(__name__)


class MQTTHandler:
    """Handles MQTT communication for telemetry data"""
    
    def __init__(self, host: str, port: int, username: str, 
                 password: str, use_tls: bool = True):
        self.host = host
        self.port = port
        self.username = username
        self.password = password
        self.use_tls = use_tls
        self.connected = False
        self.subscriptions = {}
        self.message_count = 0
        
        logger.info(f"MQTT handler initialized: {host}:{port} (TLS: {use_tls})")
        
    async def connect(self):
        """Connect to MQTT broker"""
        # In production, this would use actual MQTT client
        # For demo, simulate connection
        await asyncio.sleep(0.5)  # Simulate connection time
        self.connected = True
        logger.info(f"Connected to MQTT broker at {self.host}:{self.port}")
        
    async def disconnect(self):
        """Disconnect from MQTT broker"""
        self.connected = False
        logger.info("Disconnected from MQTT broker")
        
    async def publish(self, topic: str, message: str, qos: int = 1):
        """Publish message to topic"""
        if not self.connected:
            logger.error("Cannot publish - not connected to broker")
            return False
            
        self.message_count += 1
        
        # Log the publication
        logger.debug(f"Publishing to {topic} (QoS {qos}): {message[:100]}...")
        
        # In production, this would actually publish to MQTT broker
        # For demo, just log and return success
        return True
        
    async def subscribe(self, topic: str, callback: Callable, qos: int = 1):
        """Subscribe to a topic with a callback"""
        if not self.connected:
            logger.error("Cannot subscribe - not connected to broker")
            return False
            
        self.subscriptions[topic] = callback
        logger.info(f"Subscribed to {topic} (QoS {qos})")
        return True
        
    async def unsubscribe(self, topic: str):
        """Unsubscribe from a topic"""
        if topic in self.subscriptions:
            del self.subscriptions[topic]
            logger.info(f"Unsubscribed from {topic}")
            return True
        return False
        
    def get_stats(self) -> Dict[str, Any]:
        """Get MQTT statistics"""
        return {
            'connected': self.connected,
            'host': self.host,
            'port': self.port,
            'use_tls': self.use_tls,
            'messages_published': self.message_count,
            'active_subscriptions': list(self.subscriptions.keys())
        }
        
    async def publish_telemetry(self, telemetry_data: Dict[str, Any]):
        """Publish telemetry data to standard topics"""
        # Publish to different topics based on data type
        if 'altitude' in telemetry_data:
            await self.publish('telemetry/altitude', str(telemetry_data['altitude']))
            
        if 'gps' in telemetry_data:
            await self.publish('telemetry/gps', json.dumps(telemetry_data['gps']))
            
        # Publish complete data
        await self.publish('telemetry/complete', json.dumps(telemetry_data))
        
    def is_connected(self) -> bool:
        """Check if connected to broker"""
        return self.connected