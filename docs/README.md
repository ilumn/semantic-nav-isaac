# Documentation

For the supported simulator port, start with
[`ISAAC_SIM_PORT.md`](ISAAC_SIM_PORT.md). It defines the Isaac/ROS ownership
boundary, topic and TF contract, and end-to-end acceptance checks.

This directory contains the retained architecture material and the current
TurtleBot3 Isaac Sim port contract.

## Purpose

Documentation covers the Isaac simulator boundary, the semantic-navigation
architecture, presentation artifacts, and upstream patch notes.

## Structure

- `ISAAC_SIM_PORT.md` - Supported simulator, ROS topic/TF contract, and acceptance gates
- `semantic_nav_architecture_figures.pptx` - Clean presentation deck for the semantic navigation architecture
- `semantic_nav_architecture_figures.pdf` - PDF export of the architecture deck
- `semantic_nav_system_writeup.md` - Detailed companion writeup explaining the architecture, runtime flow, semantic memory, COLMAP enrichment, and real robot deployment model
- `patches/` - Documentation for upstream patches

## Reading Order

1. Start with `ISAAC_SIM_PORT.md` for the supported runtime contract.
2. Use the root `README.md` and `dev/isaac_sim/README.md` for commands.
3. Read `semantic_nav_system_writeup.md` for the deeper architecture.
