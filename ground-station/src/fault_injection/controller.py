"""
Fault Injection Controller for Rocket Telemetry System
"""

import asyncio
import time
import logging
from typing import Dict, Any, List, Optional
from dataclasses import dataclass
from enum import Enum
from datetime import datetime

logger = logging.getLogger(__name__)


class FaultType(Enum):
    """Types of faults that can be injected"""
    POWER_LOSS = "power_loss"
    SENSOR_DISCONNECT = "sensor_disconnect"
    COMMUNICATION_LOSS = "communication_loss"
    GPS_SIGNAL_LOSS = "gps_signal_loss"
    IMU_DRIFT = "imu_drift"
    PRESSURE_SENSOR_SPIKE = "pressure_sensor_spike"


@dataclass
class FaultScenario:
    """Definition of a fault scenario"""
    name: str
    fault_type: FaultType
    target_component: str
    duration: float  # seconds
    severity: float  # 0.0 to 1.0


class FaultInjectionController:
    """Main fault injection controller"""
    
    def __init__(self, gpio_pins: List[int] = None):
        self.gpio_pins = gpio_pins or [17, 27, 22, 23]
        self.active_faults: Dict[str, Dict[str, Any]] = {}
        self.fault_history: List[Dict[str, Any]] = []
        
        # Predefined fault scenarios
        self.scenarios = {
            "gps_signal_loss": FaultScenario(
                name="GPS Signal Loss",
                fault_type=FaultType.GPS_SIGNAL_LOSS,
                target_component="gps_module",
                duration=30.0,
                severity=1.0
            ),
            "sensor_power_failure": FaultScenario(
                name="Sensor Power Failure",
                fault_type=FaultType.POWER_LOSS,
                target_component="sensor_power",
                duration=5.0,
                severity=1.0
            )
        }
        
        logger.info(f"Fault injection controller initialized with pins: {gpio_pins}")
        
    async def inject_fault(self, fault_type: str, duration: Optional[float] = None, 
                          severity: Optional[float] = None) -> Dict[str, Any]:
        """Inject a fault into the system"""
        
        # Get scenario
        scenario = self.scenarios.get(fault_type)
        if not scenario:
            return {
                "success": False,
                "message": f"Unknown fault type: {fault_type}"
            }
            
        # Check if fault already active
        if fault_type in self.active_faults:
            return {
                "success": False,
                "message": f"Fault {fault_type} already active"
            }
            
        # Create fault record
        fault_id = f"{fault_type}_{int(time.time() * 1000)}"
        fault_record = {
            "id": fault_id,
            "type": fault_type,
            "start_time": datetime.now(),
            "duration": duration or scenario.duration,
            "severity": severity or scenario.severity,
            "status": "active"
        }
        
        # Activate fault
        self.active_faults[fault_type] = fault_record
        self.fault_history.append(fault_record)
        
        logger.info(f"Fault injected: {fault_type}")
        
        # Schedule fault removal
        asyncio.create_task(self._remove_fault_after_delay(fault_type, fault_record["duration"]))
        
        return {
            "success": True,
            "fault_id": fault_id,
            "message": f"Fault {fault_type} injected successfully"
        }
        
    async def _remove_fault_after_delay(self, fault_type: str, delay: float):
        """Remove a fault after specified delay"""
        await asyncio.sleep(delay)
        await self.remove_fault(fault_type)
        
    async def remove_fault(self, fault_type: str) -> Dict[str, Any]:
        """Remove an active fault"""
        if fault_type not in self.active_faults:
            return {
                "success": False,
                "message": f"Fault {fault_type} not active"
            }
            
        # Remove from active faults
        del self.active_faults[fault_type]
        logger.info(f"Fault cleared: {fault_type}")
        
        return {
            "success": True,
            "message": f"Fault {fault_type} cleared successfully"
        }
        
    def get_active_faults(self) -> List[Dict[str, Any]]:
        """Get list of currently active faults"""
        return [
            {
                "id": fault["id"],
                "type": fault["type"],
                "severity": fault["severity"],
                "elapsed": (datetime.now() - fault["start_time"]).total_seconds()
            }
            for fault in self.active_faults.values()
        ]