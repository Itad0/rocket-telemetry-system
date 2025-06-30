#!/usr/bin/env python3
"""
Secure Rocket Telemetry System - Ground Station
"""

import click

@click.command()
@click.option('--mode', default='debug', help='Operating mode')
def main(mode):
    """Main entry point for ground station"""
    print(f"🚀 Rocket Telemetry Ground Station")
    print(f"Mode: {mode}")
    print("System ready!")

if __name__ == '__main__':
    main()