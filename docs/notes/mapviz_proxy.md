# Mapviz Google Maps Proxy Setup Guide

## Overview
This guide shows how to replace discontinued Bing Maps in Mapviz with Google Maps using a MapProxy Docker container.

## Prerequisites
- Docker installed
- Mapviz running in ROS2
- Internet connection for initial tile loading

## Step 1: Start MapProxy Container

```bash
# Run the MapProxy container (one-time setup)
sudo docker run -p 8080:8080 -d -t --restart=always danielsnider/mapproxy

# Verify the container is running
docker ps | grep mapproxy

# Test the proxy is working
curl http://localhost:8080/wmts/gm_layer/gm_grid/1/0/0.png
```

## Step 2: Configure Mapviz

### Option A: Via GUI
1. Open Mapviz
2. Add **Tile Map** plugin
3. In tile map settings:
   - Select **"Custom WMTS Source"**
   - **Base URL**: `http://localhost:8080/wmts/gm_layer/gm_grid/{level}/{x}/{y}.png`
   - **Max Zoom**: `19`
   - **Name**: `Google Maps via Proxy`
4. Save configuration

### Option B: Via Config File
Replace your tile_map display section with:

```yaml
displays:
  - type: mapviz_plugins/tile_map
    name: new display
    config:
      visible: true
      collapsed: false
      custom_sources:
        - base_url: "http://localhost:8080/wmts/gm_layer/gm_grid/{level}/{x}/{y}.png"
          max_zoom: 19
          name: "Google Maps via Proxy"
      source: "Google Maps via Proxy"
```

## Step 3: Verify Setup

- **Maps should load**: High-quality satellite/terrain imagery
- **Caching works**: Tiles load faster on subsequent views
- **Offline capability**: Previously viewed areas work without internet

## Troubleshooting

### Container Not Working
```bash
# Check container logs
docker logs $(docker ps -q --filter ancestor=danielsnider/mapproxy)

# Restart container
docker restart $(docker ps -q --filter ancestor=danielsnider/mapproxy)
```

### Maps Not Loading
```bash
# Test proxy endpoint directly
curl -I http://localhost:8080/wmts/gm_layer/gm_grid/1/0/0.png

# Check Mapviz console for errors
# Ensure base_url format is correct (with {level}/{x}/{y})
```

### Port Conflicts
```bash
# Use different port if 8080 is occupied
sudo docker run -p 8081:8080 -d -t --restart=always danielsnider/mapproxy

# Update base_url accordingly:
# http://localhost:8081/wmts/gm_layer/gm_grid/{level}/{x}/{y}.png
```

## Benefits

- ✅ **High-quality imagery**: Google Maps satellite and terrain data
- ✅ **Cached tiles**: Faster loading and offline capability  
- ✅ **No API keys**: Free to use
- ✅ **Reliable**: Proven solution in robotics community
- ✅ **Easy setup**: Single Docker command

## Alternative: OpenStreetMap (Free)

For a completely free alternative without Docker:

```yaml
custom_sources:
  - base_url: "https://tile.openstreetmap.org/{level}/{x}/{y}.png"
    max_zoom: 19
    name: "OpenStreetMap"
``` 

## Container Management

```bash
# Stop the container
docker stop $(docker ps -q --filter ancestor=danielsnider/mapproxy)

# Remove the container
docker rm $(docker ps -aq --filter ancestor=danielsnider/mapproxy)

# Container auto-starts on system reboot (--restart=always)
```

---

**Note**: This solution uses cached Google Maps tiles via MapProxy, which provides excellent map quality for robotics visualization while maintaining good performance and offline capability.