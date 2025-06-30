"""
Hardware-in-the-Loop (HIL) Simulator for Rocket Telemetry System
"""

import asyncio
import logging
from enum import Enum
from dataclasses import dataclass
import random
import math

logger = logging.getLogger(__name__)


class FlightPhase(Enum):
    """Flight phases for simulation"""
    BOOT = "BOOT"
    CALIBRATION = "CALIBRATION"
    LAUNCH = "LAUNCH"
    ASCENT = "ASCENT"
    APOGEE = "APOGEE"
    DESCENT = "DESCENT"
    LANDING = "LANDING"


@dataclass
class SimulationState:
    """Current state of the simulation"""
    time: float = 0.0
    altitude: float = 0.0
    velocity: float = 0.0
    acceleration: float = 0.0
    phase: FlightPhase = FlightPhase.BOOT
    temperature: float = 20.0
    pressure: float = 101325.0


class HILSimulator:
    """Hardware-in-the-Loop simulator"""
    
    def __init__(self):
        self.state = SimulationState()
        self.running = False
        logger.info("HIL Simulator initialized")
        
    async def load_scenario(self, scenario_name: str):
        """Load a simulation scenario"""
        logger.info(f"Loading scenario: {scenario_name}")
        # Scenario loading logic would go here
        
    async def step(self):
        """Perform one simulation step"""
        # Update simulation state
        self.state.time += 0.01  # 100Hz simulation
        
        # Simple altitude simulation
        if self.state.phase == FlightPhase.ASCENT:
            self.state.velocity += 9.81 * 0.01  # Simple acceleration
            self.state.altitude += self.state.velocity * 0.01
            
        # Generate sensor data
        return {
            'timestamp': int(self.state.time * 1000),
            'altitude': self.state.altitude + random.gauss(0, 0.1),
            'temperature': self.state.temperature + random.gauss(0, 0.5),
            'pressure': self.state.pressure,
            'state': self.state.phase.value
        }
        
    async def stop(self):
        """Stop the simulation"""
        self.running = False
        logger.info("HIL Simulator stopped")