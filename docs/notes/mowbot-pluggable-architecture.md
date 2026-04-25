# Mowbot Pluggable Architecture: Robot Models & Sensor Kits

## Overview

The mowbot codebase implements a **sophisticated pluggable architecture** that enables dynamic selection of robot models and sensor kits at runtime through configuration, not code changes. This allows for flexible deployment configurations and true plug-and-play between different robot and sensor combinations.

## Core Architecture

### Dynamic Resolution Process
```
Your Choice → mowbot_launch → robot.xacro → Plugin Resolution → base_link Integration
     ↓              ↓              ↓              ↓                    ↓
robot_model   →  launch args  →  xacro args  →  robot plugin    →  robot components
sensor_model  →  launch args  →  xacro args  →  sensor plugin   →  sensor components
```

### Key Integration Point
- **`base_link`** serves as the **universal integration point**
- All robot components and sensors attach to this common reference frame
- Enables true plug-and-play between any robot model and any sensor kit

## Entry Point

### Main Orchestrator
```xml
<!-- src/core/mowbot_sdk/launch/mowbot_robot_launch/urdf/robot.xacro -->
<xacro:arg name="robot_model" default=""/>
<xacro:arg name="sensor_model" default=""/>

<!-- Dynamic includes -->
<xacro:include filename="$(find ${robot_model_property}_description)/urdf/robot.xacro"/>
<xacro:include filename="$(find ${sensor_model_property}_description)/urdf/sensors.xacro"/>
```

## Available Plugins

### Robot Models
- `mowbot_gazebo_modelv3` - Simulated robot (fully implemented)
- `mowbot_modelv3` - Real robot (minimal implementation)

### Sensor Kits
- `mowbot_gazebo_sensor_kit` - Simulated sensors (fully implemented)
- `mowbot_default_sensor_kit` - Real sensors (minimal implementation)

## Configuration System

### Two-Level Calibration
1. **`sensors_calibration.yaml`** - High-level sensor kit positioning relative to `base_link`
2. **`sensor_kit_calibration.yaml`** - Individual sensor fine-tuning within the kit

### Usage Example
```python
launch_arguments={
    'model': 'mowbot_gazebo_modelv3',           # Robot choice
    'sensor_model': 'mowbot_gazebo_sensor_kit', # Sensor choice
    'config_dir': PathJoinSubstitution([
        FindPackageShare('mowbot_gazebo_sensor_kit_description'), 
        'config'
    ])
}
```

## How to Add New Plugins

### Step 1: Create Plugin Structure

#### New Robot Model
```
src/launcher/mowbot_launch/robot/mowbot_my_new_robot/
├── mowbot_my_new_robot_description/
│   ├── urdf/robot.xacro          # ← REQUIRED: Must define base_link
│   ├── config/robot_info.param.yaml
│   ├── mesh/my_new_robot.stl
│   ├── CMakeLists.txt
│   └── package.xml
└── mowbot_my_new_robot_launch/
    ├── launch/robot_interface.launch.py
    ├── CMakeLists.txt
    └── package.xml
```

#### New Sensor Kit
```
src/launcher/mowbot_launch/sensor_kit/mowbot_my_new_sensors/
├── mowbot_my_new_sensors_description/
│   ├── urdf/
│   │   ├── sensors.xacro         # ← REQUIRED: Must use sensor_kit_macro
│   │   └── sensor_kit.xacro      # ← REQUIRED: Must attach to base_link
│   ├── config/
│   │   ├── sensors_calibration.yaml
│   │   └── sensor_kit_calibration.yaml
│   ├── CMakeLists.txt
│   └── package.xml
└── mowbot_my_new_sensors_launch/
    ├── launch/sensing.launch.py
    ├── CMakeLists.txt
    └── package.xml
```

### Step 2: Follow Interface Requirements

#### Robot Model Must
- ✅ Define `base_link` (integration point)
- ✅ Follow existing robot structure
- ✅ Include robot-specific components

#### Sensor Kit Must
- ✅ Use `sensor_kit_macro` with `parent="base_link"`
- ✅ Include calibration YAML files
- ✅ Follow calibration-driven positioning

### Step 3: Use Your New Plugins
```python
launch_arguments={
    'model': 'mowbot_my_new_robot',           # ← YOUR NEW ROBOT
    'sensor_model': 'mowbot_my_new_sensors',  # ← YOUR NEW SENSORS
    'config_dir': PathJoinSubstitution([
        FindPackageShare('mowbot_my_new_sensors_description'), 
        'config'
    ])
}
```

## Implementation Status

### ✅ Fully Implemented
- Gazebo robot model (sophisticated track vehicle with physics)
- Gazebo sensor kit (dual GNSS + IMU with calibration)
- Configuration system and launch integration
- Dynamic xacro resolution mechanism

### 📝 Placeholder Status
- Real robot model (minimal implementation)
- Default sensor kit (minimal implementation)

## Key Benefits

1. **🔌 True Plug-and-Play** - Any robot works with any sensor kit
2. **🧩 Modular Development** - Independent robot and sensor evolution
3. **⚡ Configuration-Driven** - No code changes for deployment variants
4. **🎯 Environment-Specific** - Optimized for simulation vs. real hardware
5. **🔧 Calibration-Aware** - Precise sensor positioning through YAML
6. **🔄 Backward Compatibility** - Existing configurations continue working
7. **🧪 Flexible Testing** - Mix real and simulated components

## Usage Examples

```bash
# Full simulation
ros2 launch mowbot_launch gazebo_sim_raceway.launch.py

# Real deployment (when implemented)
ros2 launch mowbot_launch mowbot.launch.py robot_model:=mowbot_modelv3 sensor_model:=mowbot_default_sensor_kit

# Mixed testing
ros2 launch mowbot_launch mowbot.launch.py robot_model:=mowbot_gazebo_modelv3 sensor_model:=mowbot_default_sensor_kit
```

## Source Files Summary

- **56 total source files** participate in the pluggable system
- **Core orchestrator**: `mowbot_robot_launch/urdf/robot.xacro`
- **Configuration hub**: `mowbot_launch/` directory
- **Plugin packages**: Robot and sensor model variants in launcher system

## Architecture Quality

This is a **professional-grade, industry-standard architecture** that demonstrates:
- **Advanced ROS2 practices**
- **Modular software design**
- **Production deployment considerations**
- **Scalable plugin architecture**

The system successfully separates concerns, enables flexible deployment, and provides a solid foundation for autonomous robot development!

## Quick Reference

### Current Working Configuration
```python
# In gazebo_sim_raceway.launch.py
'robot_model': 'mowbot_gazebo_modelv3'
'sensor_model': 'mowbot_gazebo_sensor_kit'
```

### Extension Pattern
```
Create Plugin → Follow Interface → Configure Launch → Dynamic Resolution
     ↓              ↓                    ↓                    ↓
Plugin structure   Required files    Launch arguments    Automatic resolution
```

### Key Files to Remember
- `src/core/mowbot_sdk/launch/mowbot_robot_launch/urdf/robot.xacro` - Main orchestrator
- `src/launcher/mowbot_launch/mowbot_launch/launch/gazebo_sim_raceway.launch.py` - Working example
- `src/launcher/mowbot_launch/robot/` - Robot model plugins
- `src/launcher/mowbot_launch/sensor_kit/` - Sensor kit plugins
