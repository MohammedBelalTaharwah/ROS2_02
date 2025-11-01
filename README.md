# ROS2_02
week 2
# Robot Controller Package

[![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy-blue)](https://docs.ros.org/en/jazzy/)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![Python](https://img.shields.io/badge/Python-3.10+-green.svg)](https://www.python.org/)

## Overview

The `robot_controller` package is a comprehensive ROS 2 implementation demonstrating publisher-subscriber architecture, command processing, and velocity control for robotic systems. This package provides three independent nodes that communicate through standardized ROS 2 topics, offering a foundation for robot state monitoring, command execution, and motion control.

### Key Features

- **Modular Architecture**: Three independent, loosely-coupled nodes
- **Real-time Status Monitoring**: Periodic robot state publication
- **Command Processing**: Interactive command interpretation and execution
- **Motion Control**: Programmatic velocity command generation
- **Type-safe Communication**: Utilizes standard ROS 2 message types
- **Comprehensive Logging**: Detailed runtime information and diagnostics

## Table of Contents

- [System Requirements](#system-requirements)
- [Architecture](#architecture)
- [Installation](#installation)
- [Package Structure](#package-structure)
- [Node Specifications](#node-specifications)
- [Usage](#usage)
- [API Reference](#api-reference)
- [Testing](#testing)
- [Troubleshooting](#troubleshooting)
- [Contributing](#contributing)
- [License](#license)

## System Requirements

### Software Dependencies

- **Operating System**: Ubuntu 24.04 LTS (Noble Numbat) or compatible
- **ROS 2 Distribution**: Jazzy Jalisco
- **Python Version**: 3.10 or higher
- **Build System**: colcon

### ROS 2 Dependencies

- `rclpy`: ROS 2 Python client library
- `std_msgs`: Standard ROS 2 message definitions
- `geometry_msgs`: Geometric message definitions for spatial data

## Architecture

### System Design

The package implements a distributed node architecture where each node operates independently and communicates through the ROS 2 middleware layer. This design ensures:

- **Fault Isolation**: Individual node failures do not cascade
- **Scalability**: Nodes can be distributed across multiple processes or machines
- **Flexibility**: Components can be started, stopped, or replaced independently
- **Testability**: Each node can be tested in isolation

### Communication Topology

```
┌─────────────────────┐
│  Status Publisher   │
│                     │
└──────────┬──────────┘
           │ publishes
           ▼
    /robot/status
    (std_msgs/String)


┌─────────────────────┐
│ Command Subscriber  │
│                     │
└──────────▲──────────┘
           │ subscribes
           │
    /robot/command
    (std_msgs/String)


┌─────────────────────┐
│ Velocity Publisher  │
│                     │
└──────────┬──────────┘
           │ publishes
           ▼
      /cmd_vel
  (geometry_msgs/Twist)
```

## Installation

### Step 1: Environment Setup

Ensure ROS 2 Jazzy is installed and sourced:

```bash
source /opt/ros/jazzy/setup.bash
```

### Step 2: Workspace Creation

Create a new ROS 2 workspace:

```bash
mkdir -p ~/robot_project_ws/src
cd ~/robot_project_ws/src
```

### Step 3: Package Creation

Generate the package structure:

```bash
ros2 pkg create robot_controller \
    --build-type ament_python \
    --dependencies rclpy std_msgs geometry_msgs \
    --license Apache-2.0 \
    --maintainer-name "Your Name" \
    --maintainer-email "your.email@example.com"
```

### Step 4: Source Code Integration

1. Navigate to the package directory:
```bash
cd robot_controller
```

2. Create the nodes directory:
```bash
mkdir -p robot_controller
```

3. Copy node implementations to `robot_controller/`:
   - `status_publisher.py`
   - `command_subscriber.py`
   - `velocity_publisher.py`

4. Ensure the `__init__.py` file exists:
```bash
touch robot_controller/__init__.py
```

### Step 5: Configuration

Replace the default `setup.py` and `package.xml` with the provided configuration files.

### Step 6: Build

```bash
cd ~/robot_project_ws
colcon build --packages-select robot_controller
```

### Step 7: Environment Configuration

```bash
source install/setup.bash
```

To make this permanent, add to `~/.bashrc`:
```bash
echo "source ~/robot_project_ws/install/setup.bash" >> ~/.bashrc
```

## Package Structure

```
robot_controller/
├── robot_controller/          # Python package directory
│   ├── __init__.py           # Package initialization
│   ├── status_publisher.py   # Status publication node
│   ├── command_subscriber.py # Command processing node
│   └── velocity_publisher.py # Velocity control node
├── resource/                  # Package resources
│   └── robot_controller      # Package marker file
├── test/                      # Unit and integration tests
│   ├── test_copyright.py
│   ├── test_flake8.py
│   └── test_pep257.py
├── package.xml               # Package manifest
├── setup.py                  # Python package setup
├── setup.cfg                 # Setup configuration
└── README.md                 # This file
```

## Node Specifications

### Status Publisher Node

#### Description
The Status Publisher node implements a periodic broadcasting system that publishes robot operational status at regular intervals. This node serves as the primary information source for monitoring robot health and operational state.

#### Technical Specifications

| Parameter | Value |
|-----------|-------|
| **Node Name** | `status_publisher` |
| **Executable** | `status_publisher` |
| **Publication Rate** | 0.5 Hz (every 2 seconds) |
| **Output Topic** | `/robot/status` |
| **Message Type** | `std_msgs/msg/String` |
| **QoS Profile** | Default (depth: 10) |

#### Published Message Format

The status message contains:
- Sequential message counter
- ISO 8601 timestamp (HH:MM:SS format)
- Operational state descriptor

**Example Output:**
```
Robot Status #42 | Time: 14:32:10 | State: OPERATIONAL
```

#### Operational Behavior

1. Initializes ROS 2 node with specified name
2. Creates publisher on `/robot/status` topic
3. Establishes 2-second timer callback
4. Continuously publishes formatted status messages
5. Logs each publication event

#### Use Cases

- System health monitoring
- Uptime tracking
- Diagnostic data collection
- Integration with monitoring dashboards
- Audit trail generation

---

### Command Subscriber Node

#### Description
The Command Subscriber node implements a command processing system that receives, validates, and executes robot control commands. It maintains internal state tracking and provides comprehensive command execution feedback.

#### Technical Specifications

| Parameter | Value |
|-----------|-------|
| **Node Name** | `command_subscriber` |
| **Executable** | `command_subscriber` |
| **Subscription Topic** | `/robot/command` |
| **Message Type** | `std_msgs/msg/String` |
| **QoS Profile** | Default (depth: 10) |
| **Supported Commands** | `start`, `stop`, `reset` |

#### Command Specification

| Command | Description | State Transition |
|---------|-------------|------------------|
| `start` | Initiates robot operation | `IDLE` → `RUNNING` |
| `stop` | Halts robot operation | `RUNNING` → `IDLE` |
| `reset` | Resets robot to initial state | `*` → `IDLE` |

#### State Machine

```
     ┌─────┐
     │IDLE │ ◄────────┐
     └──┬──┘          │
        │ start    stop│
        ▼             │
    ┌────────┐        │
    │RUNNING │────────┘
    └────────┘
        ▲
        │ reset
        └─────── (any state)
```

#### Error Handling

- **Invalid Commands**: Logged as warnings with valid command list
- **Redundant Commands**: Detected and logged (e.g., start when already running)
- **Case Insensitive**: All commands processed in lowercase
- **Whitespace Tolerant**: Leading/trailing whitespace automatically trimmed

#### Use Cases

- Remote robot control
- Automated testing sequences
- Emergency stop functionality
- State management
- Command validation and logging

---

### Velocity Publisher Node

#### Description
The Velocity Publisher node generates and publishes motion commands for differential drive robots. It implements a sinusoidal motion pattern that demonstrates both linear and angular velocity control.

#### Technical Specifications

| Parameter | Value |
|-----------|-------|
| **Node Name** | `velocity_publisher` |
| **Executable** | `velocity_publisher` |
| **Publication Rate** | 1.0 Hz (every second) |
| **Output Topic** | `/cmd_vel` |
| **Message Type** | `geometry_msgs/msg/Twist` |
| **QoS Profile** | Default (depth: 10) |

#### Motion Profile

The node implements a circular motion pattern using sinusoidal functions:

```python
phase = (count % 8) / 8.0 * 2π
linear.x = 0.5 * cos(phase)  # m/s
angular.z = 0.5 * sin(phase)  # rad/s
```

#### Twist Message Structure

```
linear:
  x: float64    # Forward/backward velocity (m/s)
  y: float64    # Left/right velocity (m/s) - typically 0
  z: float64    # Up/down velocity (m/s) - typically 0
angular:
  x: float64    # Roll rate (rad/s) - typically 0
  y: float64    # Pitch rate (rad/s) - typically 0
  z: float64    # Yaw rate (rad/s)
```

#### Safety Features

- **Graceful Shutdown**: Publishes zero velocities on node termination
- **Bounded Velocities**: Maximum linear: 0.5 m/s, angular: 0.5 rad/s
- **Smooth Transitions**: Sinusoidal profile ensures smooth acceleration

#### Use Cases

- Motion planning demonstration
- Velocity control testing
- Robot simulation
- Algorithm development
- Educational purposes

## Usage

### Starting Individual Nodes

Each node can be launched independently in separate terminal sessions.

#### Terminal 1: Status Publisher

```bash
source ~/robot_project_ws/install/setup.bash
ros2 run robot_controller status_publisher
```

#### Terminal 2: Command Subscriber

```bash
source ~/robot_project_ws/install/setup.bash
ros2 run robot_controller command_subscriber
```

#### Terminal 3: Velocity Publisher

```bash
source ~/robot_project_ws/install/setup.bash
ros2 run robot_controller velocity_publisher
```

### Sending Commands

Commands are published to the `/robot/command` topic using the ROS 2 CLI:

```bash
# Start command
ros2 topic pub /robot/command std_msgs/String "data: 'start'" --once

# Stop command
ros2 topic pub /robot/command std_msgs/String "data: 'stop'" --once

# Reset command
ros2 topic pub /robot/command std_msgs/String "data: 'reset'" --once
```

### Monitoring Topics

#### Monitor Status Messages

```bash
ros2 topic echo /robot/status
```

#### Monitor Velocity Commands

```bash
ros2 topic echo /cmd_vel
```

#### Monitor All Topics

```bash
# List active topics
ros2 topic list

# Display topic information
ros2 topic info /robot/status

# Check publishing rate
ros2 topic hz /robot/status
```

### System Introspection

#### View Active Nodes

```bash
ros2 node list
```

**Expected Output:**
```
/command_subscriber
/status_publisher
/velocity_publisher
```

#### Inspect Node Information

```bash
ros2 node info /status_publisher
```

#### Visualize Node Graph

```bash
rqt_graph
```

## API Reference

### Status Publisher

#### Constructor
```python
StatusPublisher()
```
Initializes the status publisher node with a 2-second timer.

#### Methods

**`timer_callback()`**
- **Description**: Callback invoked every 2 seconds to publish status
- **Parameters**: None
- **Returns**: None
- **Side Effects**: Publishes message to `/robot/status`

---

### Command Subscriber

#### Constructor
```python
CommandSubscriber()
```
Initializes the command subscriber node and sets initial state to IDLE.

#### Methods

**`command_callback(msg: String)`**
- **Description**: Processes incoming command messages
- **Parameters**: `msg` - std_msgs/String containing command
- **Returns**: None
- **Validates**: Command against allowed set

**`process_start_command()`**
- **Description**: Handles robot start operation
- **State Transition**: IDLE → RUNNING

**`process_stop_command()`**
- **Description**: Handles robot stop operation
- **State Transition**: RUNNING → IDLE

**`process_reset_command()`**
- **Description**: Resets robot to initial state
- **State Transition**: ANY → IDLE

---

### Velocity Publisher

#### Constructor
```python
VelocityPublisher()
```
Initializes the velocity publisher with 1-second timer and motion parameters.

#### Methods

**`timer_callback()`**
- **Description**: Publishes velocity commands in sinusoidal pattern
- **Parameters**: None
- **Returns**: None
- **Frequency**: 1 Hz

**`stop_robot()`**
- **Description**: Publishes zero velocities to stop robot
- **Parameters**: None
- **Returns**: None
- **Use Case**: Emergency stop or cleanup

#### Configuration Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `linear_speed` | float | 0.5 | Maximum linear velocity (m/s) |
| `angular_speed` | float | 0.5 | Maximum angular velocity (rad/s) |

## Testing

### Unit Testing

The package includes standard ROS 2 Python tests:

```bash
cd ~/robot_project_ws
colcon test --packages-select robot_controller
colcon test-result --verbose
```

### Integration Testing

#### Test 1: Topic Publication Verification

```bash
# Terminal 1: Start node
ros2 run robot_controller status_publisher

# Terminal 2: Verify publication
ros2 topic hz /robot/status
# Expected: ~0.5 Hz
```

#### Test 2: Command Processing Verification

```bash
# Terminal 1: Start subscriber
ros2 run robot_controller command_subscriber

# Terminal 2: Send test commands
ros2 topic pub /robot/command std_msgs/String "data: 'start'" --once
ros2 topic pub /robot/command std_msgs/String "data: 'stop'" --once

# Verify: Check Terminal 1 for state transitions
```

#### Test 3: Velocity Control Verification

```bash
# Terminal 1: Start publisher
ros2 run robot_controller velocity_publisher

# Terminal 2: Monitor output
ros2 topic echo /cmd_vel

# Verify: Check sinusoidal pattern in values
```

### System Testing

#### Full System Test

```bash
# Launch all nodes
ros2 run robot_controller status_publisher &
ros2 run robot_controller command_subscriber &
ros2 run robot_controller velocity_publisher &

# Verify all nodes active
ros2 node list

# Verify all topics active
ros2 topic list

# Test communication
ros2 topic pub /robot/command std_msgs/String "data: 'start'" --once
```

### Performance Metrics

| Metric | Expected Value | Tolerance |
|--------|---------------|-----------|
| Status Rate | 0.5 Hz | ±0.05 Hz |
| Velocity Rate | 1.0 Hz | ±0.05 Hz |
| Command Latency | <10 ms | - |
| CPU Usage (per node) | <1% | - |
| Memory Usage (per node) | <50 MB | - |

## Troubleshooting

### Common Issues

#### Issue: Package Not Found

**Symptom:**
```
Package 'robot_controller' not found
```

**Solution:**
```bash
cd ~/robot_project_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select robot_controller
source install/setup.bash
```

---

#### Issue: Module Import Error

**Symptom:**
```
ModuleNotFoundError: No module named 'robot_controller'
```

**Solution:**
```bash
# Verify __init__.py exists
ls ~/robot_project_ws/src/robot_controller/robot_controller/__init__.py

# If missing, create it
touch ~/robot_project_ws/src/robot_controller/robot_controller/__init__.py

# Rebuild
cd ~/robot_project_ws
colcon build --packages-select robot_controller --symlink-install
```

---

#### Issue: Permission Denied

**Symptom:**
```
Permission denied: 'status_publisher.py'
```

**Solution:**
```bash
chmod +x ~/robot_project_ws/src/robot_controller/robot_controller/*.py
```

---

#### Issue: Topics Not Appearing

**Symptom:**
```
ros2 topic list
# Missing expected topics
```

**Solution:**
```bash
# Verify nodes are running
ros2 node list

# Check for errors
ros2 wtf

# Restart nodes with verbose logging
ros2 run robot_controller status_publisher --ros-args --log-level DEBUG
```

---

#### Issue: Build Failures

**Symptom:**
```
--- stderr: robot_controller
SetuptoolsDeprecationWarning
```

**Solution:**
```bash
# Clean workspace
cd ~/robot_project_ws
rm -rf build/ install/ log/

# Update dependencies
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Rebuild
colcon build --packages-select robot_controller
```

### Debug Mode

Enable detailed logging for troubleshooting:

```bash
ros2 run robot_controller status_publisher --ros-args --log-level DEBUG
```

### Getting Help

1. Check ROS 2 Jazzy documentation: https://docs.ros.org/en/jazzy/
2. Review ROS Answers: https://answers.ros.org/
3. Consult package issues: [Create an issue]

## Contributing

### Development Setup

```bash
# Fork and clone repository
git clone <your-fork-url>
cd robot_controller

# Create feature branch
git checkout -b feature/your-feature-name

# Make changes and test
colcon build --packages-select robot_controller
colcon test --packages-select robot_controller

# Commit with conventional commits
git commit -m "feat: add new feature"

# Push and create pull request
git push origin feature/your-feature-name
```

### Code Standards

- **Style Guide**: PEP 8 compliance required
- **Documentation**: All public APIs must be documented
- **Testing**: Minimum 80% code coverage
- **Type Hints**: Required for all function signatures
- **Linting**: Must pass flake8 and pep257

### Testing Requirements

```bash
# Run linters
colcon test --packages-select robot_controller --ctest-args tests test_flake8
colcon test --packages-select robot_controller --ctest-args tests test_pep257

# Run all tests
colcon test --packages-select robot_controller
colcon test-result --verbose
```

## License

This package is licensed under the Apache License 2.0. See LICENSE file for details.

```
Copyright 2024 [Your Name/Organization]

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
```

## Citation

If you use this package in your research, please cite:

```bibtex
@software{robot_controller,
  title = {Robot Controller: ROS 2 Publisher-Subscriber Implementation},
  author = {Your Name},
  year = {2024},
  url = {https://github.com/yourusername/robot_controller}
}
```

## Acknowledgments

- ROS 2 Development Team
- Open Robotics
- ROS Community Contributors

## Maintainers

- **Your Name** - *Initial work* - [your.email@example.com]

## Project Status

**Status:** Active Development  
**Version:** 0.0.1  
**Last Updated:** November 2024

---

**For detailed testing procedures, see [TESTING.md](TESTING.md)**  
**For contribution guidelines, see [CONTRIBUTING.md](CONTRIBUTING.md)**  
**For API documentation, see [API.md](API.md)**
