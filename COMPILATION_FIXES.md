# Fast-Planner Compilation Fixes and Mockamap Integration

## Fixed Issues

### 1. GCC 5+ Compilation Problems

**Problem**: The repository had compilation errors with GCC 5 and later versions due to improper smart pointer initialization.

**Files Fixed**:
- `uav_simulator/Utils/rviz_plugins/src/aerialmap_display.cpp`
- `uav_simulator/Utils/rviz_plugins/src/probmap_display.cpp`

**Fix Applied**:
```cpp
// Before (caused compilation errors):
//! @bug cannot compile @gcc-5 or later, material_(0)

// After (GCC 5+ compatible):
, material_()
```

**Technical Explanation**: 
In modern C++ (GCC 5+), initializing smart pointers (`Ogre::MaterialPtr`) with raw pointers like `(0)` is deprecated and causes compilation errors. Smart pointers should be default-initialized using `()` or left uninitialized, as they automatically initialize to null.

## Mockamap Simulation Integration

### 2. Added Mockamap Package

**Location**: `uav_simulator/mockamap/`

**What is Mockamap?**
Mockamap is a ROS-based simulation map generator that creates various types of procedural maps for UAV path planning testing.

**Features**:
- **Perlin Noise 3D**: Realistic terrain-like obstacles
- **2D Maze Generation**: Structured environments for navigation testing  
- **Customizable Obstacles**: Various shapes and densities
- **Real-time Generation**: Dynamic map updates
- **ROS Integration**: Publishes point clouds directly

### Usage

#### Quick Start
```bash
# Launch mockamap with default Perlin noise
roslaunch mockamap mockamap.launch

# Launch with Fast-Planner integration
roslaunch mockamap fast_planner_with_mockamap.launch

# Launch specific map types
roslaunch mockamap perlin3d.launch
roslaunch mockamap maze2d.launch
```

#### Map Type Configuration

**Perlin Noise 3D (type=1)**:
- `complexity`: Noise frequency (0.0-0.5, higher = more complex)
- `fill`: Obstacle density (0.0-0.5, higher = more obstacles)
- `fractal`: Detail level (1-5, higher = more detail)
- `attenuation`: Fractal dampening (0.0-0.5)

**Example Configuration**:
```xml
<param name="type" type="int" value="1"/>
<param name="complexity" type="double" value="0.05"/>
<param name="fill" type="double" value="0.25"/>
<param name="fractal" type="int" value="2"/>
<param name="attenuation" type="double" value="0.15"/>
```

### Integration with Fast-Planner

The mockamap publishes point clouds that can be directly used by Fast-Planner for:
- Obstacle detection and mapping
- Path planning algorithm testing
- UAV simulation in complex environments

## Verification

### Compilation Test
The fixed code should now compile successfully with GCC 5+ without any smart pointer initialization errors.

### Runtime Test
1. Launch mockamap: `roslaunch mockamap mockamap.launch`
2. Verify point cloud publishing: `rostopic echo /map_generator/global_cloud`
3. Visualize in RViz: The launch file automatically opens RViz with proper configuration

## Files Changed

- **Fixed**: `uav_simulator/Utils/rviz_plugins/src/aerialmap_display.cpp`
- **Fixed**: `uav_simulator/Utils/rviz_plugins/src/probmap_display.cpp` 
- **Added**: `uav_simulator/mockamap/` (complete package)
- **Updated**: `.gitignore` (exclude build directories)
- **Added**: `uav_simulator/mockamap/launch/fast_planner_with_mockamap.launch`

## Notes

- All changes are minimal and surgical - only fixing the specific compilation issues
- Mockamap is a complete, standalone package that doesn't modify existing Fast-Planner code
- The integration provides additional simulation capabilities without breaking existing functionality