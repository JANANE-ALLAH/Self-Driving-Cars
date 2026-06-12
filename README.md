
# Self-Driving Cars - Industrial-Grade Bicycle Kinematic Model

[![Python Version](https://img.shields.io/badge/python-3.7+-blue.svg)](https://www.python.org/downloads/)
[![License](https://img.shields.io/badge/license-MIT-green.svg)](LICENSE)
[![Code Style](https://img.shields.io/badge/code%20style-PEP%208-purple.svg)](https://www.python.org/dev/peps/pep-0008/)

## Overview

This project provides a **production-ready implementation** of the bicycle kinematic model for autonomous vehicle simulation. It's designed for industrial applications with:

✅ **Modular Architecture** - Clean separation of concerns
✅ **Comprehensive Testing** - 100% unit test coverage
✅ **Configuration Management** - JSON/YAML support
✅ **Professional Documentation** - Full API reference
✅ **Visualization Tools** - Built-in plotting utilities
✅ **Type Hints** - Full type annotations
✅ **Error Handling** - Robust input validation

## Features

### Core Model
- **Kinematic Bicycle Model**: Implements accurate vehicle dynamics using differential equations
- **State Management**: Position (x, y), heading (θ), steering angle (δ)
- **Input Commands**: Velocity (v) and steering rate (ω) control
- **History Tracking**: Optional trajectory recording for analysis

### Trajectory Planning
- **Circular Paths**: Generate constant-radius trajectories
- **Figure-8 Paths**: Complex maneuver patterns
- **Square Paths**: Structured path with corners
- **Custom Trajectories**: Arbitrary velocity and steering profiles

### Configuration System
- **YAML Support**: Human-readable configuration files
- **JSON Support**: Standard format for integration
- **Validation**: Automatic parameter checking
- **Persistence**: Save and load configurations

## Installation

### From Source

```bash
# Clone repository
git clone https://github.com/JANANE-ALLAH/Self-Driving-Cars.git
cd Self-Driving-Cars

# Install in development mode
pip install -e .

# Install with test dependencies
pip install -e ".[dev]"
