-- Rocket Telemetry Database Schema

CREATE TABLE IF NOT EXISTS flights (
    id SERIAL PRIMARY KEY,
    flight_id VARCHAR(50) UNIQUE NOT NULL,
    launch_time TIMESTAMP NOT NULL,
    landing_time TIMESTAMP,
    mission_name VARCHAR(100),
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

CREATE TABLE IF NOT EXISTS telemetry (
    id SERIAL PRIMARY KEY,
    flight_id VARCHAR(50) REFERENCES flights(flight_id),
    timestamp BIGINT NOT NULL,
    altitude FLOAT,
    temperature FLOAT,
    pressure FLOAT,
    humidity FLOAT,
    voltage FLOAT,
    current FLOAT,
    gps_lat FLOAT,
    gps_lon FLOAT,
    gps_satellites INTEGER,
    orientation_roll FLOAT,
    orientation_pitch FLOAT,
    orientation_yaw FLOAT,
    state VARCHAR(20),
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

CREATE TABLE IF NOT EXISTS anomalies (
    id SERIAL PRIMARY KEY,
    flight_id VARCHAR(50) REFERENCES flights(flight_id),
    timestamp TIMESTAMP NOT NULL,
    anomaly_type VARCHAR(50) NOT NULL,
    severity VARCHAR(20),
    description TEXT,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

CREATE TABLE IF NOT EXISTS reports (
    id SERIAL PRIMARY KEY,
    flight_id VARCHAR(50) REFERENCES flights(flight_id),
    report_type VARCHAR(50),
    file_path VARCHAR(255),
    generated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- Indexes for performance
CREATE INDEX idx_telemetry_flight_id ON telemetry(flight_id);
CREATE INDEX idx_telemetry_timestamp ON telemetry(timestamp);
CREATE INDEX idx_anomalies_flight_id ON anomalies(flight_id);