# Qt5 SerialPort

This role installs the Qt5 SerialPort development package (`libqt5serialport5-dev`) for serial/UART communication in Qt applications (e.g. witmotion IMU UART tools).

## Purpose

- Qt5 SerialPort library and headers for building applications that use serial ports
- Required by packages such as witmotion-uart-qt and other Qt-based serial tools

## Inputs

| Name | Required | Default | Description |
|------|----------|---------|-------------|
| qt5_serialport_package | No | `libqt5serialport5-dev` | Package name to install |
| qt5_serialport_state | No | `latest` | Package state (e.g. `latest`, `present`) |
| qt5_serialport_verify_installation | No | `true` | Whether to verify installation |

## Manual Installation

```bash
sudo apt-get update
sudo apt-get install -y libqt5serialport5-dev
```

## Usage

### In a playbook

```yaml
- name: Install Qt5 SerialPort
  hosts: localhost
  roles:
    - role: mowbot.dev_env.qt5_serialport
```

### With options

```yaml
- role: mowbot.dev_env.qt5_serialport
  vars:
    qt5_serialport_verify_installation: false
```
