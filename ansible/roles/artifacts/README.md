# Artifacts

This role manages the download and configuration of mowbot artifacts, including ONNX models and configuration files.

## Purpose

The artifacts role provides a framework for downloading and managing:
- **ONNX models** - Neural network models for perception and inference
- **Configuration files** - Settings and calibration data
- **Other assets** - Any additional files needed for mowbot operation

This role is designed to be:
- **Configurable** - Easy to add new artifacts as they become available
- **Verifiable** - Checks download integrity with checksums
- **Flexible** - Supports different types of artifacts and sources

## Current Status

**No artifacts are currently available** - The role is set up and ready for when artifacts become available in the future.

## Variables

| Name | Default | Description |
|------|---------|-------------|
| `artifacts_dir` | `"~/mowbot_data"` | Directory to store artifacts |
| `download_onnx_models` | `false` | Whether to download ONNX models |
| `download_config_files` | `false` | Whether to download configuration files |
| `artifacts_verify_downloads` | `true` | Whether to verify download integrity |
| `artifacts_create_backup` | `false` | Whether to create backups of existing files |
| `onnx_models` | `[]` | List of ONNX models to download |
| `config_files` | `[]` | List of configuration files to download |

## Artifact Structure

### ONNX Models
```yaml
onnx_models:
  - name: "model_name.onnx"
    url: "https://example.com/models/model_name.onnx"
    checksum: "sha256:actual_checksum_here"
```

### Configuration Files
```yaml
config_files:
  - name: "config_file.yaml"
    url: "https://example.com/configs/config_file.yaml"
    checksum: "sha256:actual_checksum_here"
```

## Installation Process

The role performs the following steps:

1. **Expand directory path** - Converts `~` to actual home directory
2. **Create artifacts directory** - Creates the target directory if it doesn't exist
3. **Download ONNX models** - Downloads neural network models (if enabled)
4. **Download configuration files** - Downloads config files (if enabled)
5. **Verify downloads** - Checks file integrity and existence
6. **Display status** - Provides feedback on download success

## Usage Examples

### Basic setup (creates directory only)
```yaml
- hosts: localhost
  roles:
    - artifacts
```

### Custom artifacts directory
```yaml
- hosts: localhost
  roles:
    - artifacts
  vars:
    artifacts_dir: "/opt/mowbot/artifacts"
```

### Future usage with artifacts
```yaml
- hosts: localhost
  roles:
    - artifacts
  vars:
    download_onnx_models: true
    download_config_files: true
    onnx_models:
      - name: "perception_model.onnx"
        url: "https://example.com/models/perception.onnx"
        checksum: "sha256:actual_checksum"
    config_files:
      - name: "perception_config.yaml"
        url: "https://example.com/configs/perception.yaml"
        checksum: "sha256:actual_checksum"
```

## Manual Setup

If you need to set up manually:

```bash
# Create artifacts directory
mkdir -p ~/mowbot_data

# Set permissions
chmod 755 ~/mowbot_data

# Verify directory
ls -la ~/mowbot_data
```

## Adding New Artifacts

When artifacts become available, you can add them by:

1. **Update defaults/main.yaml**:
   ```yaml
   onnx_models:
     - name: "new_model.onnx"
       url: "https://example.com/models/new_model.onnx"
       checksum: "sha256:actual_checksum"
   ```

2. **Enable downloads**:
   ```yaml
   download_onnx_models: true
   ```

3. **Run the role**:
   ```bash
   ansible-playbook playbooks/main.yaml -e "prompt_download_artifacts=y"
   ```

## Testing

### Verify directory creation
```bash
# Check if directory exists
ls -la ~/mowbot_data

# Check permissions
stat ~/mowbot_data
```

### Test with custom directory
```bash
# Test with different directory
ansible-playbook -i localhost, -c local test-artifacts.yaml \
  -e "artifacts_dir=/tmp/test_artifacts"
```

## Troubleshooting

### Permission denied
```bash
# Check directory permissions
ls -la ~/mowbot_data

# Fix permissions if needed
chmod 755 ~/mowbot_data
```

### Download failures
```bash
# Check network connectivity
curl -I https://example.com/models/test.onnx

# Check disk space
df -h ~/mowbot_data
```

### Checksum verification
```bash
# Verify file checksum
sha256sum ~/mowbot_data/model.onnx

# Compare with expected checksum
echo "expected_checksum model.onnx" | sha256sum -c
```

## Future Enhancements

When artifacts become available, this role can be extended with:

- **Progressive downloads** - Resume interrupted downloads
- **Version management** - Track artifact versions
- **Automatic updates** - Check for newer versions
- **Compression support** - Handle compressed artifacts
- **Mirror support** - Multiple download sources
- **Caching** - Local artifact cache

## Prerequisites

- **Internet connection** (for downloading artifacts)
- **Sufficient disk space** (for storing artifacts)
- **Write permissions** (for artifacts directory)

## Dependencies

- **Ansible builtin modules** (file, get_url, stat)
- **curl/wget** (for downloads, handled by get_url)

This role provides a solid foundation for artifact management when they become available! 📦