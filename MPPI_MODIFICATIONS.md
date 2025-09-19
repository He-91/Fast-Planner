# Fast-Planner MPPI Modifications

This is a modified version of Fast-Planner that implements the following key changes:

## 1. MPPI-Based Local Planning

**Original**: Used B-spline trajectory optimization for local planning
**Modified**: Replaced with Model Predictive Path Integral (MPPI) controller

### Key Features:
- Sampling-based trajectory generation with importance sampling
- Collision avoidance through penalty-based cost functions
- Path following guidance from topology planning
- Configurable noise parameters for exploration vs exploitation

### Usage:
Set `use_mppi: true` in the FSM configuration to enable MPPI-based local planning.

```xml
<param name="fsm/use_mppi" value="true" type="bool"/>
```

## 2. Minimum Cost Path Selection

**Original**: Selected multiple topological paths for optimization
**Modified**: Selects only the single path with minimum cost

### Changes:
- Modified `TopologyPRM::selectShortPaths()` to return only one path
- Added `pathCost()` method considering both length and obstacle clearance
- Cost function includes path length, collision penalties, and clearance violations

### Benefits:
- Reduced computational overhead
- More focused optimization on the best available path
- Better integration with MPPI controller

## 3. Flyable Obstacles Environment

**Original**: Fixed obstacle heights up to ceiling
**Modified**: Optional low-height obstacles that can be flown over

### Usage:
Enable flyable obstacles in launch files:

```xml
<arg name="flyable_obstacles" value="true"/>
```

This reduces obstacle height from 3.0m to 1.5m, allowing the drone to fly over obstacles when beneficial.

## Configuration

### MPPI Parameters

Key parameters can be configured in the launch files:

```xml
<!-- MPPI Controller Parameters -->
<param name="mppi/num_samples" value="1000" type="int"/>
<param name="mppi/horizon_length" value="20" type="int"/>
<param name="mppi/dt" value="0.1" type="double"/>
<param name="mppi/lambda" value="1.0" type="double"/>

<!-- Noise parameters -->
<param name="mppi/sigma_pos" value="0.2" type="double"/>
<param name="mppi/sigma_vel" value="0.5" type="double"/>
<param name="mppi/sigma_acc" value="1.0" type="double"/>

<!-- Cost weights -->
<param name="mppi/w_collision" value="1000.0" type="double"/>
<param name="mppi/w_goal" value="10.0" type="double"/>
<param name="mppi/w_smoothness" value="1.0" type="double"/>
```

### Parameter Tuning Guidelines:

- **num_samples**: Higher values improve trajectory quality but increase computation time
- **lambda**: Higher values make the controller more deterministic (less exploration)
- **sigma_* parameters**: Control the exploration vs exploitation trade-off
- **Cost weights**: Adjust relative importance of different objectives

## Running the Modified System

1. **Traditional Mode** (original B-spline planning):
```bash
roslaunch plan_manage topo_replan.launch
```

2. **MPPI Mode** with flyable obstacles:
```bash
roslaunch plan_manage topo_replan.launch flyable_obstacles:=true
```

The system will automatically use MPPI for local planning while maintaining the original global planning.

## File Structure

### New Files:
- `fast_planner/plan_manage/include/plan_manage/mppi_controller.h`
- `fast_planner/plan_manage/src/mppi_controller.cpp`
- `fast_planner/plan_manage/config/mppi_params.yaml`

### Modified Files:
- `fast_planner/plan_manage/src/planner_manager.cpp` - MPPI integration
- `fast_planner/path_searching/src/topo_prm.cpp` - Path selection modification
- `fast_planner/plan_manage/src/topo_replan_fsm.cpp` - FSM updates
- Launch files - Configuration parameters

## Performance Considerations

- MPPI computational cost scales with `num_samples * horizon_length`
- Recommended to start with lower sample counts for real-time performance
- Path following cost helps MPPI converge faster when topology guidance is available
- System gracefully falls back to direct planning when topology planning fails

## Validation

The modified system maintains compatibility with the original Fast-Planner interfaces while providing enhanced local planning capabilities through MPPI and improved environment simulation options.