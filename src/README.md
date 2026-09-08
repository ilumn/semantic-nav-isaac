# Source Code

This directory contains the core algorithm implementations for the quadruped navigation system.

## Purpose

Source code is organized into modules:
- **navigation/** - Path planning algorithms (A*, local planner)
- **mapping/** - SLAM and mapping (LiDAR, camera-based)
- **perception/** - Object detection and tracking
- **utils/** - Common utilities and helpers

## Structure

Each module has:
- Implementation code
- README explaining input/output conventions
- Optional test/demo scripts

## Development

When adding new algorithms:
1. Create module directory
2. Add README with input/output specs
3. Implement algorithm
4. Add tests/demos if applicable
