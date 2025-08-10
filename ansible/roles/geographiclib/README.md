# GeographicLib

This role installs the GeographicLib development package for GNSS coordinate transformations in robotics applications.

## Purpose

GeographicLib is a C++ library for solving geodesic problems, which is essential for:
- GNSS coordinate transformations
- Converting between different coordinate systems (WGS84, local Cartesian, etc.)
- Precise geodetic calculations for autonomous navigation
- Mowbot GNSS fuser component

## Inputs

| Name | Required | Default | Description |
|------|----------|---------|-------------|
| geographiclib_package | No | `libgeographic-dev` | Package name to install |
| geographiclib_version | No | `latest` | Package version to install |
| geographiclib_verify_installation | No | `true` | Whether to verify installation |

## Manual Installation

```bash
# Update package lists
sudo apt-get update

# Install GeographicLib development package
sudo apt-get install -y libgeographic-dev

# Verify installation
pkg-config --exists GeographicLib
```

## Usage

### In a playbook
```yaml
- name: Install GeographicLib
  hosts: localhost
  roles:
    - role: mowbot.dev_env.geographiclib
```

### With custom variables
```yaml
- name: Install GeographicLib with custom settings
  hosts: localhost
  roles:
    - role: mowbot.dev_env.geographiclib
      vars:
        geographiclib_version: "1.52-1"
        geographiclib_verify_installation: true
```

## Dependencies

This role has no dependencies and can be run independently.

## Features

- **Smart installation**: Checks if GeographicLib is already installed before attempting installation
- **Configurable package**: Customizable package name and version
- **Optional verification**: Can disable installation verification if needed
- **Clear feedback**: Provides success/warning messages based on installation status

## Verification

The role automatically verifies the installation using `pkg-config` and provides feedback on success or failure. Verification can be disabled by setting `geographiclib_verify_installation: false`.
