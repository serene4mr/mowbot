# Off-Road AMR Simulation Settings for Baylands Terrain

## Overview
Optimized Gazebo simulation parameters for off-road autonomous mobile robot operation in the Baylands environment (grass, dirt, uneven terrain).

## Changes Applied

### 1. Friction Parameters (track.xacro)
**Purpose**: Simulate realistic grass/dirt traction with controlled slip

```xml
<!-- BEFORE: Indoor smooth surface -->
<mu>5.0</mu>     <!-- Too grippy for off-road -->
<mu2>3.0</mu2>

<!-- AFTER: Off-road grass/dirt -->
<mu>1.2</mu>     <!-- Primary friction: grass/dirt contact -->
<mu2>0.8</mu2>   <!-- Lateral friction: allows realistic slip -->
<slip1>0.1</slip1>   <!-- Longitudinal slip: traction dynamics -->
<slip2>0.15</slip2>  <!-- Lateral slip: turning behavior -->
```

**Effect**:
- ✅ Realistic wheel slip on grass
- ✅ Natural turning behavior
- ✅ Prevents unrealistic "locked" traction
- ✅ Better power delivery simulation

---

### 2. Contact Physics (track.xacro)
**Purpose**: Simulate soft terrain compliance and bump absorption

```xml
<!-- BEFORE: Hard surface (asphalt/concrete) -->
<kp>10000.0</kp>  <!-- Very stiff -->
<kd>10.0</kd>     <!-- Low damping -->
<min_depth>0.001</min_depth>
<max_vel>0.01</max_vel>

<!-- AFTER: Soft terrain (grass/dirt) -->
<kp>1500.0</kp>   <!-- Compliant: terrain deformation -->
<kd>100.0</kd>    <!-- High damping: shock absorption -->
<min_depth>0.005</min_depth>  <!-- Allow penetration -->
<max_vel>1.0</max_vel>         <!-- Realistic velocity -->
```

**Effect**:
- ✅ Wheels "sink" slightly into soft terrain
- ✅ Better shock absorption over bumps
- ✅ More stable on uneven ground
- ✅ Prevents bouncing and instability

---

### 3. Bounce Properties (track.xacro)
**Purpose**: Minimize bouncing on rough terrain

```xml
<!-- NEW: Low restitution for off-road -->
<bounce>
  <restitution_coefficient>0.1</restitution_coefficient>
  <threshold>0.1</threshold>
</bounce>
```

**Effect**:
- ✅ Wheels don't bounce on rocks/bumps
- ✅ Maintains ground contact
- ✅ More stable motion

---

### 4. Joint Damping (track.xacro)
**Purpose**: Add rotational resistance for stability on rough terrain

```xml
<!-- NEW: Joint dynamics for stability -->
<dynamics damping="0.5" friction="0.1"/>
```

**Effect**:
- ✅ Reduces oscillations over bumps
- ✅ Smoother wheel rotation
- ✅ Better energy dissipation
- ✅ Prevents wheel "spinning out"

---

### 5. Wheel Mass (robot.xacro)
**Purpose**: Increase rotational inertia for better stability

```xml
<!-- BEFORE -->
<xacro:property name="robot_track_wheel_mass" value="1" />

<!-- AFTER -->
<xacro:property name="robot_track_wheel_mass" value="3.0" />
```

**Effect**:
- ✅ Higher rotational inertia
- ✅ Better momentum on rough terrain
- ✅ More realistic mass distribution
- ✅ Improved physics stability (310kg robot needs heavier wheels)

---

## Parameter Reference Table

| Parameter | Indoor/Smooth | Off-Road/Grass | Purpose |
|-----------|---------------|----------------|---------|
| **Friction (mu)** | 5.0 | 1.2 | Primary traction |
| **Friction (mu2)** | 3.0 | 0.8 | Lateral traction |
| **Slip1** | - | 0.1 | Longitudinal slip |
| **Slip2** | - | 0.15 | Lateral slip |
| **Contact Stiffness (kp)** | 10000 | 1500 | Terrain compliance |
| **Contact Damping (kd)** | 10 | 100 | Shock absorption |
| **Min Depth** | 0.001 | 0.005 | Penetration allowance |
| **Max Velocity** | 0.01 | 1.0 | Velocity limit |
| **Restitution** | - | 0.1 | Bounce coefficient |
| **Joint Damping** | - | 0.5 | Rotational damping |
| **Joint Friction** | - | 0.1 | Rotational friction |
| **Wheel Mass (kg)** | 1.0 | 3.0 | Rotational inertia |

---

## Terrain-Specific Tuning Guide

### For Different Terrains:

#### **Muddy Terrain**
```xml
<mu>0.6</mu>          <!-- Lower grip -->
<mu2>0.4</mu2>
<slip1>0.3</slip1>    <!-- More slip -->
<slip2>0.4</slip2>
<kp>800.0</kp>        <!-- Very soft -->
```

#### **Gravel/Rocky**
```xml
<mu>0.9</mu>          <!-- Medium grip -->
<mu2>0.7</mu2>
<kp>2500.0</kp>       <!-- Medium stiff -->
<kd>80.0</kd>
<restitution_coefficient>0.2</restitution_coefficient>  <!-- Some bounce -->
```

#### **Sand**
```xml
<mu>0.8</mu>          <!-- Low grip -->
<mu2>0.5</mu2>
<slip1>0.4</slip1>    <!-- High slip */
<kp>500.0</kp>        <!-- Very compliant */
<min_depth>0.01</min_depth>  <!-- Deep penetration */
```

#### **Asphalt/Concrete (Indoor)**
```xml
<mu>4.0</mu>          <!-- High grip -->
<mu2>3.0</mu2>
<slip1>0.01</slip1>   <!-- Minimal slip */
<kp>10000.0</kp>      <!-- Very stiff */
<kd>10.0</kd>
```

---

## Expected Behavior Changes

### Before (Indoor Settings)
- ❌ Unrealistic "locked" traction on grass
- ❌ Robot bounces over small bumps
- ❌ Too responsive/twitchy controls
- ❌ Wheels feel "glued" to ground
- ❌ Unstable physics on uneven terrain

### After (Off-Road Settings)
- ✅ Natural wheel slip on grass
- ✅ Smooth motion over bumps
- ✅ Realistic control response
- ✅ Progressive traction behavior
- ✅ Stable physics on terrain variations

---

## Testing Recommendations

### 1. **Traction Test**
```bash
# Accelerate on flat grass - should see gradual acceleration with slight slip
ros2 topic pub /joy_cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}}"
```
**Expected**: Smooth acceleration, no jerking

### 2. **Turning Test**
```bash
# Sharp turn - wheels should slip slightly
ros2 topic pub /joy_cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 1.0}}"
```
**Expected**: Slight drift/slip in turn, natural behavior

### 3. **Hill Climbing**
- Drive up slopes in Baylands
**Expected**: Wheels maintain contact, some slip on steep grades

### 4. **Rough Terrain**
- Drive over bumpy areas
**Expected**: Smooth motion, minimal bouncing, good stability

---

## Performance Notes

- **Physics Update Rate**: 250 Hz (world setting)
- **Step Size**: 0.004s (4ms)
- **Real-time Factor**: 1.0
- **Solver**: ODE Quick with 10 iterations

These world settings are already optimized for off-road. No changes needed.

---

## Troubleshooting

### Robot slides too much:
- Increase `mu` to 1.5-2.0
- Decrease `slip1` to 0.05

### Robot bounces on bumps:
- Decrease `kp` to 1000
- Increase `kd` to 150
- Check `restitution_coefficient` is low (0.1)

### Wheels "spin out" unrealistically:
- Increase joint damping to 1.0
- Increase `mu` slightly

### Robot feels sluggish:
- Decrease joint damping to 0.3
- Decrease `kd` to 50

### Physics instability/jittering:
- Check wheel mass (should be 2-5 kg)
- Verify `min_depth` is not too large (max 0.01)
- Ensure solver iterations ≥ 10 in world file

---

## Files Modified

1. ✅ `/workspaces/mowbot/src/launcher/mowbot_launch/robot/mowbot_gazebo_robot_modelv3/mowbot_gazebo_modelv3_description/urdf/track.xacro`
   - Friction parameters
   - Contact physics
   - Bounce properties
   - Joint dynamics

2. ✅ `/workspaces/mowbot/src/launcher/mowbot_launch/robot/mowbot_gazebo_robot_modelv3/mowbot_gazebo_modelv3_description/urdf/robot.xacro`
   - Wheel mass property

---

## References

- **Baylands Coordinates**: 37.413534°N, -121.996561°W
- **Terrain Type**: Grass, dirt, some gravel paths
- **Robot Mass**: 310 kg total
- **Wheel Configuration**: 6 wheels (3 per side), tracked vehicle

---

## Validation Checklist

- [x] Friction coefficients appropriate for grass/dirt
- [x] Contact stiffness allows terrain compliance
- [x] Damping prevents bouncing
- [x] Slip parameters enable realistic traction
- [x] Wheel mass appropriate for robot size
- [x] Joint damping adds stability
- [x] Bounce coefficient minimized
- [x] All 6 wheels configured identically

**Status**: ✅ Ready for off-road testing in Baylands!

