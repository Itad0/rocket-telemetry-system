# Secure Rocket Telemetry & Test Automation System

## Overview

A defense-grade rocket telemetry system featuring Hardware-in-the-Loop (HIL) simulation, fault injection, encrypted data pipelines, and comprehensive test automation.

## System Architecture

### Onboard Flight System (STM32)
- **MCU**: STM32F4/F7 series with HSM support
- **Sensors**: MS5611, DHT22, INA219, NEO-6M GPS, BNO055 IMU, Camera module
- **Communication**: LoRa SX127x/SX126x with AES-128 encryption

### Ground Station (Raspberry Pi 5)
- **Operating Modes**: Simulation, Live Flight, and Debug
- **Services**: MQTT (TLS), FastAPI (HTTPS), Prometheus, Loki, Grafana

## Quick Start

1. Clone repository
2. Run `pip install -r requirements.txt`
3. Configure `.env` file
4. Run `docker-compose up -d`
5. Start ground station: `python ground-station/src/main.py`