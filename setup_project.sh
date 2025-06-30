#!/bin/bash
echo "Creating Rocket Telemetry System directories..."
mkdir -p firmware/STM32/Core/{Inc,Src}
mkdir -p ground-station/src/{main,hil,fault_injection,security}
mkdir -p docker/mosquitto/config
mkdir -p docs
mkdir -p tests
mkdir -p scripts
mkdir -p config
echo "Done! Directories created."
ls -la
