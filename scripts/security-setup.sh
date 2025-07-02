#!/bin/bash

# Secure Rocket Telemetry System - Security Setup Script

set -euo pipefail

echo "🔐 Setting up security infrastructure..."

# Create directories
mkdir -p docker/mosquitto/certs
mkdir -p backup/keys

# Generate CA certificate
echo "Generating CA certificate..."
openssl req -new -x509 -days 365 -extensions v3_ca -keyout docker/mosquitto/certs/ca.key -out docker/mosquitto/certs/ca.crt -subj "/C=US/ST=State/L=City/O=RocketTelemetry/CN=RocketCA"

# Generate server certificate
echo "Generating server certificate..."
openssl genrsa -out docker/mosquitto/certs/server.key 2048
openssl req -new -key docker/mosquitto/certs/server.key -out docker/mosquitto/certs/server.csr -subj "/C=US/ST=State/L=City/O=RocketTelemetry/CN=mqtt.rocket.local"
openssl x509 -req -in docker/mosquitto/certs/server.csr -CA docker/mosquitto/certs/ca.crt -CAkey docker/mosquitto/certs/ca.key -CAcreateserial -out docker/mosquitto/certs/server.crt -days 365

# Generate client certificates
echo "Generating client certificates..."
openssl genrsa -out docker/mosquitto/certs/client.key 2048
openssl req -new -key docker/mosquitto/certs/client.key -out docker/mosquitto/certs/client.csr -subj "/C=US/ST=State/L=City/O=RocketTelemetry/CN=ground_station"
openssl x509 -req -in docker/mosquitto/certs/client.csr -CA docker/mosquitto/certs/ca.crt -CAkey docker/mosquitto/certs/ca.key -CAcreateserial -out docker/mosquitto/certs/client.crt -days 365

# Set permissions
chmod 600 docker/mosquitto/certs/*.key
chmod 644 docker/mosquitto/certs/*.crt

# Generate MQTT passwords
echo "Generating MQTT passwords..."
touch docker/mosquitto/config/passwords
docker run --rm -v $(pwd)/docker/mosquitto/config:/mosquitto/config eclipse-mosquitto:2.0.18 mosquitto_passwd -b /mosquitto/config/passwords admin admin_password
docker run --rm -v $(pwd)/docker/mosquitto/config:/mosquitto/config eclipse-mosquitto:2.0.18 mosquitto_passwd -b /mosquitto/config/passwords ground_station station_password
docker run --rm -v $(pwd)/docker/mosquitto/config:/mosquitto/config eclipse-mosquitto:2.0.18 mosquitto_passwd -b /mosquitto/config/passwords flight_system flight_password

echo "✅ Security setup complete!"