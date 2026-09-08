# Configuration Files

This directory contains configuration files for different hardware platforms and environments.

## Purpose

Configuration files define hardware settings, sensor parameters, algorithm parameters, and platform-specific settings for the quadruped navigation system.

## Files

- `rpi4_bookworm_aarch64.yaml` - Raspberry Pi 4 configuration (Bookworm OS, ARM64)
- `sim_tb3.yaml` - TurtleBot3 simulation environment configuration

## Usage

Load configuration in your code:

```python
import yaml
with open('configs/rpi4_bookworm_aarch64.yaml') as f:
    config = yaml.safe_load(f)
```
