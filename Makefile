# Rocket Telemetry System Makefile

.PHONY: help setup install test run docker-up docker-down clean

help:
	@echo "Available commands:"
	@echo "  make setup       - Initial project setup"
	@echo "  make install     - Install dependencies"
	@echo "  make test        - Run tests"
	@echo "  make run         - Run ground station"
	@echo "  make docker-up   - Start Docker services"
	@echo "  make docker-down - Stop Docker services"
	@echo "  make clean       - Clean temporary files"

setup:
	@echo "Setting up project..."
	@pip install -r requirements.txt
	@cp .env.example .env
	@mkdir -p logs reports/generated
	@chmod +x scripts/*.sh
	@echo "Setup complete! Edit .env with your configuration."

install:
	pip install -r requirements.txt

test:
	pytest tests/ -v

run:
	python -m ground_station.src.main --mode debug

docker-up:
	docker-compose -f docker/docker-compose.yml up -d

docker-down:
	docker-compose -f docker/docker-compose.yml down

clean:
	find . -type f -name "*.pyc" -delete
	find . -type d -name "__pycache__" -delete
	rm -rf .pytest_cache
	rm -rf logs/*.log