# Fast-Planner with MPPI Integration

A robust and efficient quadrotor path planning system that combines topology-guided planning with Model Predictive Path Integral (MPPI) control for fast autonomous flight in complex environments.

## Overview

This enhanced version of Fast-Planner integrates MPPI (Model Predictive Path Integral) control with the original B-spline trajectory optimization system. The planner uses sampling-based trajectory generation for robust collision avoidance while maintaining smooth and dynamically feasible paths.

**Key Features:**
- **MPPI-based Local Planning**: Sampling-based trajectory optimization with importance sampling
- **Topology-guided Path Planning**: Uses topological paths as guidance for MPPI trajectory generation  
- **B-spline Trajectory Representation**: Maintains compatibility with existing systems while using MPPI optimization
- **Collision Avoidance**: Penalty-based cost functions with real-time obstacle detection
- **Configurable Parameters**: Extensive tuning options for different flight scenarios
- **Dual Planning Modes**: Support for both MPPI and traditional B-spline optimization

## Why B-spline Code is Retained

The B-spline trajectory representation is **NOT** deleted because it serves a crucial role in the system architecture:

1. **Trajectory Representation Format**: B-splines provide a smooth, continuous trajectory representation that's compatible with the existing Fast-Planner ecosystem
2. **Interface Compatibility**: The trajectory server, visualization tools, and controller interfaces expect B-spline format
3. **Mathematical Properties**: B-splines ensure C² continuity (smooth acceleration) which is essential for quadrotor control
4. **Conversion Bridge**: MPPI generates discrete waypoints which are converted to B-spline format via `convertMPPIToTrajectory()`

**MPPI handles optimization, B-spline handles representation** - this hybrid approach gets the best of both worlds.

## System Architecture

```
Topology Planning → MPPI Optimization → B-spline Conversion → Trajectory Execution
       ↑                    ↑                    ↑                    ↑
    Path Search        Sampling-based       Smooth Representation   Controller
    & Guidance         Optimization         & Continuity            Interface
```

## Installation and Dependencies

### Prerequisites
- Ubuntu 18.04/20.04 with ROS Melodic/Noetic
- C++14 compiler
- Eigen3, PCL, NLopt, Armadillo

### Install Dependencies
```bash
# Install NLopt v2.7.1
git clone -b v2.7.1 https://github.com/stevengj/nlopt.git
cd nlopt && mkdir build && cd build
cmake .. && make && sudo make install

# Install other dependencies
sudo apt-get install libarmadillo-dev
```

### Build the Project
```bash
# Clone and build
cd ${YOUR_WORKSPACE}/src
git clone <this-repository-url>
cd ..
catkin_make

# Source the workspace
source devel/setup.bash
```

## Configuration

### MPPI Parameters

Key MPPI parameters in `launch/topo_algorithm.xml`:

```xml
<!-- MPPI Controller Parameters -->
<param name="mppi/num_samples" value="1000" type="int"/>          <!-- Number of trajectory samples -->
<param name="mppi/horizon_length" value="20" type="int"/>         <!-- Planning horizon steps -->
<param name="mppi/dt" value="0.1" type="double"/>                 <!-- Time step -->
<param name="mppi/lambda" value="1.0" type="double"/>             <!-- Temperature parameter -->

<!-- Noise parameters (exploration vs exploitation) -->
<param name="mppi/sigma_pos" value="0.2" type="double"/>          <!-- Position noise std -->
<param name="mppi/sigma_vel" value="0.5" type="double"/>          <!-- Velocity noise std -->
<param name="mppi/sigma_acc" value="1.0" type="double"/>          <!-- Acceleration noise std -->

<!-- Cost function weights -->
<param name="mppi/w_collision" value="1000.0" type="double"/>     <!-- Collision avoidance -->
<param name="mppi/w_goal" value="10.0" type="double"/>            <!-- Goal reaching -->
<param name="mppi/w_smoothness" value="1.0" type="double"/>       <!-- Trajectory smoothness -->
<param name="mppi/w_path_following" value="20.0" type="double"/>  <!-- Topology path following -->

<!-- Physical constraints -->
<param name="mppi/max_vel" value="3.0" type="double"/>            <!-- Max velocity -->
<param name="mppi/max_acc" value="2.5" type="double"/>            <!-- Max acceleration -->
<param name="mppi/safe_distance" value="0.3" type="double"/>      <!-- Min obstacle distance -->
```

### Parameter Tuning Guidelines

- **`num_samples`**: Higher values (1000-2000) improve trajectory quality but increase computation
- **`lambda`**: Higher values make the controller more deterministic (less exploration)
- **`sigma_*`**: Control exploration - higher values increase randomness
- **`w_collision`**: Should be much higher than other weights for safety
- **`w_path_following`**: Important when using topology guidance

## Running Test Scenarios

### Environment Selection

The Fast-Planner now supports **two different simulation environments** that you can choose from:

#### Environment 1: Random Forest (Default)
Traditional Fast-Planner environment with randomly generated cylindrical obstacles.

#### Environment 2: Mockamap Procedural 
Advanced procedural environment using Perlin noise for more realistic terrain-like obstacles.

### Quick Start with Environment Selection

```bash
# Terminal 1: Launch RViz visualization
roslaunch plan_manage rviz.launch

# Terminal 2: Choose your environment
# Environment 1 (Random Forest - Default)
roslaunch plan_manage topo_replan_environments.launch environment:=1

# OR Environment 2 (Mockamap Procedural)  
roslaunch plan_manage topo_replan_environments.launch environment:=2

# Test procedure:
# 1. Wait for the map to generate (red obstacles in RViz)
# 2. Use "2D Nav Goal" tool in RViz to set a goal point
# 3. Observe multiple topology paths being generated and visualized
# 4. Watch the drone select the best path and execute smooth trajectory
# 5. Try multiple goal points to see different scenarios
```

### Expected Behavior

**Topology Path Visualization:**
- **Yellow/Orange lines**: Filtered topology paths (shows 4+ path options)
- **Colored lines**: Selected topology paths (best candidates)
- **Green trajectory**: Final optimized B-spline trajectory
- **Blue arrow**: Current drone position and orientation

**Console Output:**
```
[Topo]: Visualizing 4 filtered topology paths
[Topo]: Visualizing 2 selected topology paths  
[MPPI] Testing 2 topology paths for best trajectory
[MPPI] Testing topology path 0 with 12 points
[MPPI]: Successfully selected best trajectory from 2 candidates with cost: 45.234
```

### Scenario 1: Basic MPPI Planning with Simple Obstacles

This scenario tests MPPI planning in a structured environment with fixed obstacles.

```bash
# Terminal 1: Start visualization
roslaunch plan_manage rviz.launch

# Terminal 2: Launch basic MPPI planning (Environment 1)
roslaunch plan_manage topo_replan_environments.launch environment:=1

# Test procedure:
# 1. Wait for the random map to generate (red obstacles in RViz)
# 2. Use "2D Nav Goal" tool in RViz to set a goal point
# 3. Observe the drone planning and following MPPI-optimized trajectory
# 4. Try multiple goal points to test different scenarios

# Expected behavior:
# - Yellow/Orange lines show multiple topology path options (4+ paths)
# - Colored lines show selected best topology paths  
# - Green trajectory shows the final planned path
# - Drone smoothly avoids obstacles using MPPI optimization
# - Console shows topology path generation and MPPI cost information
```

### Scenario 2: Advanced Procedural Environment

Test MPPI planning in complex procedural terrain.

```bash  
# Terminal 1: Start visualization
roslaunch plan_manage rviz.launch

# Terminal 2: Launch procedural environment (Environment 2)
roslaunch plan_manage topo_replan_environments.launch environment:=2

# Expected behavior:
# - More complex terrain-like obstacles from Perlin noise
# - Multiple topology paths adapting to complex geometry
# - MPPI controller optimizing through narrow passages
# - Smooth trajectory following in challenging environments
```

### Manual Environment Launch (Advanced)

For more control, you can launch environments manually:

```bash
# Traditional approach (Environment 1 only)
roslaunch plan_manage topo_replan.launch

# With mockamap directly  
roslaunch mockamap fast_planner_with_mockamap.launch

### Scenario 2: MPPI with Flyable Obstacles (Advanced)

This scenario tests MPPI planning with low-height obstacles that can be flown over.

```bash
# Terminal 1: Start visualization  
roslaunch plan_manage rviz.launch

# Terminal 2: Launch MPPI with flyable obstacles
roslaunch plan_manage topo_replan.launch flyable_obstacles:=true

# Test procedure:
# 1. Notice obstacles are now lower height (1.5m instead of 3.0m)
# 2. Set goals that require flying over or around obstacles
# 3. Observe how MPPI finds optimal paths (over vs around)
# 4. Test goals at different heights (0.5m, 1.0m, 2.0m, 2.5m)

# Expected behavior:
# - Drone may choose to fly over low obstacles when efficient
# - Higher goals should show more over-obstacle trajectories
# - Lower goals should show more around-obstacle paths
# - Smooth transitions between different altitude levels
```

### Scenario 3: Comparison Mode (Optional)

Compare MPPI vs traditional B-spline optimization:

```bash
# Run traditional B-spline mode (disable MPPI)
# Edit launch/topo_algorithm.xml: set use_mppi to false
<param name="fsm/use_mppi" value="false" type="bool"/>

# Then run same scenarios and compare:
# - Trajectory smoothness
# - Computational time  
# - Obstacle avoidance behavior
# - Goal reaching accuracy
```

## Understanding the Output

### Console Logs

**MPPI Planning Logs:**
```
[MPPI] Generating trajectory: start=[x,y,z], goal=[x,y,z]
[MPPI] Cost range: min=X.XXX, best_idx=N
[MPPI] Generated trajectory with cost: X.XXX, final goal distance: X.XXX
```

**Planner Manager Logs:**
```
[MPPI]: Searching topology paths  
[MPPI] Using topology guidance path with N points
[FastPlannerManager] Created B-spline with N control points, dt=X.XXX
```

### RViz Visualization

- **Red cubes**: Obstacles in the environment
- **Green line**: Planned trajectory  
- **Blue arrow**: Current drone position and orientation
- **Yellow spheres**: Topology path points (when available)
- **Purple markers**: MPPI sample trajectories (if visualization enabled)

### Performance Metrics

Monitor these values for system performance:
- **Planning time**: Should be < 50ms for real-time operation
- **Trajectory cost**: Lower values indicate better trajectories  
- **Goal distance**: Final distance to target goal
- **Collision margins**: Minimum distance to obstacles

## Troubleshooting

### Common Issues and Solutions

**Issue**: `[MPPI] All trajectories have very high cost`
- **Solution**: Reduce collision weight or increase safe distance, check for impossible goals

**Issue**: `[MPPI] MPPI controller not initialized!`  
- **Solution**: Verify MPPI parameters are loaded correctly in launch files

**Issue**: Drone moves slowly or stops frequently
- **Solution**: Increase `num_samples`, reduce `w_collision`, tune `sigma_*` parameters

**Issue**: Trajectories are too jerky or unstable
- **Solution**: Increase `w_smoothness`, reduce noise parameters, check B-spline conversion timing

**Issue**: Planning takes too long
- **Solution**: Reduce `num_samples`, decrease `horizon_length`, optimize cost function weights

### Parameter Debugging

Enable detailed logging by setting in launch file:
```xml
<param name="logging/level" value="debug" type="string"/>
```

### Performance Optimization

For better performance:
1. Start with `num_samples=500` and increase gradually
2. Use shorter horizons (`horizon_length=15`) for faster environments  
3. Tune `lambda` parameter for exploration/exploitation balance
4. Adjust cost weights based on your specific environment

## Technical Details

### MPPI Algorithm Overview

1. **Sample Generation**: Generate N random noise sequences
2. **Trajectory Rollout**: Forward-simulate dynamics with noise
3. **Cost Evaluation**: Evaluate each trajectory using multi-objective cost function
4. **Importance Weighting**: Assign weights based on trajectory costs
5. **Weighted Average**: Compute optimal trajectory as weighted average
6. **B-spline Conversion**: Convert discrete trajectory to smooth B-spline

### Cost Function Components

- **Collision Cost**: Exponential penalty based on distance to obstacles
- **Goal Cost**: Quadratic distance to target with time weighting
- **Smoothness Cost**: Penalizes high acceleration changes  
- **Path Following Cost**: Encourages following topology guidance
- **Velocity/Acceleration Constraints**: Soft constraints on dynamic limits

## Publications and References

If you use this implementation, please cite:
- Original Fast-Planner papers (see original README)
- MPPI: "Model Predictive Path Integral Control" by Williams et al.

## Contributing

This is a research implementation. For issues or improvements:
1. Check parameter tuning first
2. Review console logs for error details
3. Test with different scenarios
4. Document reproducible issues with launch parameters

## License

Same as original Fast-Planner: GPLv3 license.