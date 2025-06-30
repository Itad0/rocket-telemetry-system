"""
Telemetry Receiver Module for LoRa Communication
"""

import asyncio
import logging
import random
import time
from typing import Dict, Any, Optional

logger = logging.getLogger(__name__)


class TelemetryReceiver:
    """Receives telemetry data via LoRa"""
    
    def __init__(self, frequency: float, spreading_factor: int, 
                 bandwidth: int, coding_rate: int):
        self.frequency = frequency
        self.sf = spreading_factor
        self.bw = bandwidth
        self.cr = coding_rate
        self.running = False
        self.packet_count = 0
        
        # Simulated signal quality
        self.rssi = -70  # Received Signal Strength Indicator
        self.snr = 10    # Signal-to-Noise Ratio
        
        logger.info(f"Telemetry receiver initialized: {frequency}MHz, SF{spreading_factor}, BW{bandwidth}kHz")
        
    async def start(self):
        """Start the receiver"""
        self.running = True
        logger.info("Telemetry receiver started")
        
    async def stop(self):
        """Stop the receiver"""
        self.running = False
        logger.info("Telemetry receiver stopped")
        
    async def receive_packet(self) -> Optional[Dict[str, Any]]:
        """Receive a telemetry packet"""
        if not self.running:
            return None
            
        # Simulate packet reception delay
        await asyncio.sleep(0.1)
        
        # Simulate packet loss (5% chance)
        if random.random() < 0.05:
            return None
            
        # Simulate signal quality variation
        self.rssi = -70 + random.gauss(0, 5)
        self.snr = 10 + random.gauss(0, 2)
        
        # For demo, return simulated encrypted packet
        self.packet_count += 1
        
        # Simulated encrypted telemetry data
        packet = {
            'rssi': self.rssi,
            'snr': self.snr,
            'frequency': self.frequency,
            'packet_id': self.packet_count,
            'timestamp': int(time.time() * 1000),
            'data': {
                'version': 1,
                'key_id': 'telemetry_key_v1',
                'iv': ''.join(random.choices('0123456789abcdef', k=32)),
                'ciphertext': ''.join(random.choices('0123456789abcdef', k=64)),
                'mac': ''.join(random.choices('0123456789abcdef', k=64))
            }
        }
        
        logger.debug(f"Received packet #{self.packet_count} - RSSI: {self.rssi:.1f} dBm, SNR: {self.snr:.1f} dB")
        return packet
        
    def get_stats(self) -> Dict[str, Any]:
        """Get receiver statistics"""
        return {
            'packets_received': self.packet_count,
            'frequency': self.frequency,
            'spreading_factor': self.sf,
            'bandwidth': self.bw,
            'coding_rate': self.cr,
            'last_rssi': self.rssi,
            'last_snr': self.snr,
            'status': 'running' if self.running else 'stopped'
        }
        
    def set_frequency(self, frequency: float):
        """Change receiver frequency"""
        self.frequency = frequency
        logger.info(f"Frequency changed to {frequency} MHz")
        
    def set_spreading_factor(self, sf: int):
        """Change spreading factor"""
        if 6 <= sf <= 12:
            self.sf = sf
            logger.info(f"Spreading factor changed to {sf}")
        else:
            logger.error(f"Invalid spreading factor: {sf}")