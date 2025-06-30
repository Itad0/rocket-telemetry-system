#!/usr/bin/env python3
"""
Secure Rocket Telemetry System - Ground Station
Main implementation with three operating modes: Simulation, Live, Debug
"""

import asyncio
import os
import sys
import signal
import logging
import json
from datetime import datetime
from typing import Optional, Dict, Any
from enum import Enum

import click
from dotenv import load_dotenv
from rich.console import Console
from rich.table import Table
from rich.live import Live
from rich.panel import Panel

# Import all our modules
from .hil.simulator import HILSimulator
from .fault_injection.controller import FaultInjectionController
from .security.encryption import TelemetryEncryption, SecurityConfig
from .database.manager import DatabaseManager
from .telemetry.receiver import TelemetryReceiver
from .telemetry.mqtt_handler import MQTTHandler
from .monitoring.prometheus_exporter import PrometheusExporter
from .reporting.generator import ReportGenerator
from .camera.receiver import CameraReceiver
from .api.server import create_app

# Load environment variables
load_dotenv()

# Configure logging
logging.basicConfig(
    level=getattr(logging, os.getenv('LOG_LEVEL', 'INFO')),
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# Rich console for pretty output
console = Console()


class OperatingMode(Enum):
    """Ground station operating modes"""
    SIMULATION = "simulation"
    LIVE = "live"
    DEBUG = "debug"


class GroundStation:
    """Main ground station controller"""
    
    def __init__(self, mode: OperatingMode):
        self.mode = mode
        self.running = False
        self.components = {}
        self.telemetry_data = {}
        self.metrics = {
            'packets_received': 0,
            'packets_lost': 0,
            'uptime': 0,
            'last_packet_time': None
        }
        
        # Initialize components
        self._initialize_components()
        
    def _initialize_components(self):
        """Initialize components based on operating mode"""
        logger.info(f"Initializing ground station in {self.mode.value} mode")
        
        # Common components for all modes
        self.components['db'] = DatabaseManager(
            os.getenv('DATABASE_URL', 'sqlite:///telemetry.db')
        )
        self.components['mqtt'] = MQTTHandler(
            host=os.getenv('MQTT_BROKER_HOST', 'localhost'),
            port=int(os.getenv('MQTT_BROKER_PORT', 1883)),
            username=os.getenv('MQTT_USERNAME', 'user'),
            password=os.getenv('MQTT_PASSWORD', 'pass'),
            use_tls=os.getenv('MQTT_TLS_ENABLED', 'false').lower() == 'true'
        )
        self.components['prometheus'] = PrometheusExporter(
            port=int(os.getenv('PROMETHEUS_PORT', 9090))
        )
        self.components['report'] = ReportGenerator(self.components['db'])
        
        # Encryption
        self.cipher = TelemetryEncryption(SecurityConfig(enable_integrity_check=True))
        
        # Mode-specific components
        if self.mode in [OperatingMode.LIVE, OperatingMode.DEBUG]:
            self.components['telemetry'] = TelemetryReceiver(
                frequency=float(os.getenv('LORA_FREQUENCY', 915.0)),
                spreading_factor=int(os.getenv('LORA_SPREADING_FACTOR', 7)),
                bandwidth=int(os.getenv('LORA_BANDWIDTH', 125)),
                coding_rate=int(os.getenv('LORA_CODING_RATE', 5))
            )
            
            if os.getenv('CAMERA_ENABLED', 'true').lower() == 'true':
                self.components['camera'] = CameraReceiver()
        
        if self.mode in [OperatingMode.SIMULATION, OperatingMode.DEBUG]:
            self.components['hil'] = HILSimulator()
            
        # Fault injection
        if os.getenv('FAULT_INJECTION_ENABLED', 'true').lower() == 'true':
            self.components['fault'] = FaultInjectionController(
                gpio_pins=[int(p) for p in os.getenv('FAULT_GPIO_PINS', '17,27,22,23').split(',')]
            )
        
        # Create API
        self.app = create_app(self)
        
    async def start(self):
        """Start the ground station"""
        self.running = True
        logger.info("Starting ground station components...")
        
        # Start components
        tasks = []
        
        # Connect to services
        tasks.append(self.components['mqtt'].connect())
        tasks.append(self.components['prometheus'].start())
        
        # Start receivers
        if 'telemetry' in self.components:
            tasks.append(self.components['telemetry'].start())
            
        if 'camera' in self.components:
            tasks.append(self.components['camera'].start())
            
        # Wait for startup
        await asyncio.gather(*tasks)
        
        # Start main loops
        if 'telemetry' in self.components:
            asyncio.create_task(self._telemetry_loop())
            
        if 'hil' in self.components:
            asyncio.create_task(self._hil_loop())
            
        # Start UI
        asyncio.create_task(self._ui_loop())
        
        logger.info("Ground station started successfully")
        
    async def stop(self):
        """Stop the ground station"""
        self.running = False
        logger.info("Stopping ground station...")
        
        # Stop components
        if 'telemetry' in self.components:
            await self.components['telemetry'].stop()
            
        if 'hil' in self.components:
            await self.components['hil'].stop()
            
        if 'camera' in self.components:
            await self.components['camera'].stop()
            
        await self.components['mqtt'].disconnect()
        
        logger.info("Ground station stopped")
        
    async def _telemetry_loop(self):
        """Main telemetry reception loop"""
        while self.running:
            try:
                packet = await self.components['telemetry'].receive_packet()
                
                if packet:
                    # Decrypt
                    decrypted = self.cipher.decrypt_telemetry(packet['data'])
                    
                    # Update metrics
                    self.metrics['packets_received'] += 1
                    self.metrics['last_packet_time'] = datetime.now()
                    
                    # Store
                    await self.components['db'].store_telemetry(decrypted)
                    
                    # Publish
                    await self.components['mqtt'].publish(
                        'telemetry/live',
                        json.dumps(decrypted)
                    )
                    
                    self.telemetry_data = decrypted
                    
            except Exception as e:
                logger.error(f"Error in telemetry loop: {e}")
                
            await asyncio.sleep(0.01)
            
    async def _hil_loop(self):
        """HIL simulation loop"""
        await self.components['hil'].load_scenario('default_flight')
        
        while self.running:
            try:
                sim_data = await self.components['hil'].step()
                
                if self.mode == OperatingMode.SIMULATION:
                    self.telemetry_data = sim_data
                    await self.components['mqtt'].publish(
                        'telemetry/simulation',
                        json.dumps(sim_data)
                    )
                    
            except Exception as e:
                logger.error(f"Error in HIL loop: {e}")
                
            await asyncio.sleep(0.1)
            
    async def _ui_loop(self):
        """Update console UI"""
        while self.running:
            # Create status display
            table = Table(title=f"Ground Station - {self.mode.value.upper()}")
            table.add_column("Parameter", style="cyan")
            table.add_column("Value", style="green")
            
            if self.telemetry_data:
                table.add_row("Altitude", f"{self.telemetry_data.get('altitude', 0):.1f} m")
                table.add_row("State", str(self.telemetry_data.get('state', 'UNKNOWN')))
                
            table.add_row("Packets", str(self.metrics['packets_received']))
            
            console.clear()
            console.print(table)
            
            await asyncio.sleep(1)
            
    async def inject_fault(self, fault_type: str, parameters: Dict[str, Any]):
        """Inject a fault"""
        if 'fault' in self.components:
            return await self.components['fault'].inject_fault(
                fault_type,
                parameters.get('duration'),
                parameters.get('severity')
            )
        return {"success": False, "message": "Fault injection not available"}
        
    async def generate_report(self, report_type: str):
        """Generate a report"""
        return await self.components['report'].generate_flight_report(
            self.telemetry_data,
            self.metrics
        )


async def main_async(mode: str):
    """Async main function"""
    operating_mode = OperatingMode(mode)
    station = GroundStation(operating_mode)
    
    try:
        await station.start()
        # Keep running until interrupted
        while True:
            await asyncio.sleep(1)
    except KeyboardInterrupt:
        logger.info("Shutdown requested")
    finally:
        await station.stop()


@click.command()
@click.option('--mode', 
              type=click.Choice(['simulation', 'live', 'debug']), 
              default='debug',
              help='Operating mode for the ground station')
def main(mode: str):
    """Secure Rocket Telemetry Ground Station"""
    
    console.print(Panel.fit(
        f"[bold cyan]🚀 Secure Rocket Telemetry System[/bold cyan]\n"
        f"[green]Ground Station v1.0.0[/green]\n"
        f"Mode: [yellow]{mode.upper()}[/yellow]",
        border_style="blue"
    ))
    
    try:
        asyncio.run(main_async(mode))
    except KeyboardInterrupt:
        console.print("\n[red]Shutdown complete[/red]")
    except Exception as e:
        console.print(f"\n[red]Error: {e}[/red]")
        logger.exception("Fatal error")
        sys.exit(1)


if __name__ == '__main__':
    main()