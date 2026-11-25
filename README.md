# UAV Navigation in Known Indoor Environments

### MATLAB-Based Simulation of A* Path Planning, PD Control, UAV Dynamics, and 360° LiDAR Sensing

**Research areas:** Autonomous Robotics · Robot Navigation · Motion Planning · Path Planning · Perception and Sensing · LiDAR-Based Systems · Feedback Control · Aerial Robotics · Ground and Surface Robotics

---

## 1. Project Overview

This project develops a MATLAB-based simulation framework for autonomous navigation of a UAV in a known, two-dimensional indoor environment.

The framework integrates:

- Occupancy-grid environment representation.
- A* global path planning.
- PD-based waypoint tracking.
- Simplified planar UAV dynamics.
- Simulated 360° LiDAR sensing.
- Obstacle detection and LiDAR point-cloud generation.
- Trajectory visualization and performance evaluation.

The primary objective is to investigate the interaction between global path planning, feedback control, and simplified UAV motion dynamics in structured indoor environments.

The UAV navigates through a predefined environment containing static circular obstacles. A* generates a collision-avoiding path using a known occupancy grid, while a PD controller generates acceleration commands to track the planned waypoints.

LiDAR sensing is also simulated to generate range measurements and point clouds. In the current implementation, LiDAR operates as a sensing and visualization subsystem; its measurements are not used to construct the planning map or control the UAV.

The project is intended as a foundational simulation study that can be extended toward navigation in partially known or unknown environments.

## 2. Research Objectives

The main objectives are:

1. Develop a two-dimensional occupancy-grid representation of an indoor environment.
2. Implement A* for global path planning.
3. Implement waypoint tracking using a PD controller.
4. Model simplified UAV motion using planar translational dynamics.
5. Simulate 360° LiDAR measurements through geometric ray casting.
6. Evaluate navigation performance across multiple environment configurations.
7. Establish a baseline for future development involving mapping, localization, and autonomous navigation in unknown environments.

## 3. System Architecture

The current framework consists of two principal processing pipelines.

### 3.1 Navigation pipeline

```text
Known Environment
       |
       v
Environment Generation
       |
       v
Occupancy Grid
       |
       v
Obstacle Inflation
       |
       v
A* Global Path Planning
       |
       v
Waypoint Generation
       |
       v
PD Controller
       |
       v
UAV Dynamics
       |
       v
Position and Velocity Update
       |
       v
Trajectory and Collision Evaluation
```

The occupancy grid is generated from the predefined environment. Obstacles are inflated to account for the UAV's effective radius and an additional safety margin.

A* computes a global path through the inflated occupancy grid. The resulting path is used as a sequence of waypoints for the PD controller.

The controller generates acceleration commands, which are applied to the simplified UAV dynamics model.

### 3.2 LiDAR sensing pipeline

```text
UAV Position and Orientation
             |
             v
      360° LiDAR Simulation
             |
             v
       Ray Casting
             |
             v
     Range Measurements
             |
             v
       Hit Detection
             |
             v
     LiDAR Point Cloud
             |
             v
       Visualization
```

The LiDAR subsystem simulates range measurements against the physical obstacles and room boundaries.

**Important distinction:** In this version, LiDAR is passive with respect to navigation. The planner uses the predefined occupancy grid rather than a map constructed from LiDAR measurements. Therefore, the current system is a known-map navigation framework, not an SLAM-based or LiDAR-driven autonomous navigation system.

## 4. Mathematical and Computational Methodology

### 4.1 Environment representation

The simulated indoor environment is a square room of dimensions:

\[
10\,\text{m}\times10\,\text{m}
\]

The environment is represented using a two-dimensional occupancy grid with a resolution of \(0.1\,\text{m}\).

This produces a grid of:

\[
100\times100\text{ cells}
\]

The occupancy values are:

| Value | Meaning |
|---|---|
| 0 | Free space |
| 1 | Occupied space |

Obstacles are represented as circles.

### 4.2 Obstacle inflation

To account for the UAV's physical size and maintain a safety margin, obstacles are inflated before path planning.

The inflation radius is:

\[
R_{\mathrm{inflated}}
=
R_{\mathrm{UAV}}+d_{\mathrm{safety}}
\]

where:

- \(R_{\mathrm{UAV}}=0.25\,\text{m}\) is the UAV radius.
- \(d_{\mathrm{safety}}=0.15\,\text{m}\) is the additional safety margin.

Therefore:

\[
R_{\mathrm{inflated}}=0.40\,\text{m}
\]

The inflated obstacles are used for planning, while the physical obstacle geometry is retained for environment representation and LiDAR ray casting.

### 4.3 A* global path planning

A* is used to find a path between the specified start and goal positions on the occupancy grid.

The evaluation function is:

\[
f(n)=g(n)+h(n)
\]

where:

- \(g(n)\) is the accumulated path cost from the start node.
- \(h(n)\) is the estimated cost from the current node to the goal.
- \(f(n)\) is the total estimated path cost.

The implementation uses an eight-connected neighbourhood:

- Four cardinal directions, with movement cost \(1\).
- Four diagonal directions, with movement cost \(\sqrt{2}\).

The heuristic is based on Euclidean distance.

Diagonal corner cutting is prevented to avoid paths passing diagonally between blocked cells.

### 4.4 Simplified UAV dynamics

The UAV is modelled as a planar translational system.

The continuous-time equations are:

\[
\dot{\mathbf{p}}=\mathbf{v}
\]

\[
\dot{\mathbf{v}}=\mathbf{a}
\]

where:

- \(\mathbf{p}\) is the two-dimensional position.
- \(\mathbf{v}\) is the velocity.
- \(\mathbf{a}\) is the commanded acceleration.

The numerical integration uses a semi-implicit Euler update:

\[
\mathbf{v}_{k+1}
=
\mathbf{v}_k+\mathbf{a}_k\Delta t
\]

\[
\mathbf{p}_{k+1}
=
\mathbf{p}_k+\mathbf{v}_{k+1}\Delta t
\]

Acceleration and velocity magnitudes are constrained by predefined maximum values.

This is a simplified planar model. It does not represent the complete six-degree-of-freedom flight dynamics of a multirotor UAV.

### 4.5 PD waypoint tracking

A PD controller is used to track the waypoints generated by A*.

The position error is:

\[
\mathbf{e}(t)
=
\mathbf{p}_{\mathrm{target}}(t)
-
\mathbf{p}(t)
\]

The acceleration command is:

\[
\mathbf{a}_{\mathrm{cmd}}
=
K_p\mathbf{e}
-
K_d\mathbf{v}
\]

The derivative term is expressed using the negative current velocity, assuming a stationary target waypoint.

Although the controller function supports an integral term, the current configuration uses \(K_i=0\). Therefore, the implemented controller operates as a PD controller.

### 4.6 Simulated 360° LiDAR

A two-dimensional LiDAR sensor is simulated using geometric ray casting.

| Parameter | Configuration |
|---|---:|
| Field of view | 360° |
| Angular resolution | 1° |
| Number of beams | 360 |
| Maximum range | 5 m |
| Initial orientation | 0° |

For each beam, the ray direction is calculated from its angular position.

The ray is checked against:

- The four room boundaries.
- Circular obstacles.

For circular obstacles, ray-circle intersection is calculated using the quadratic intersection equation. The nearest valid non-negative intersection distance is selected.

If no intersection is detected within the maximum sensor range, the ray endpoint is placed at the maximum range and is marked as having no obstacle hit.

The current LiDAR model is idealized. It does not include measurement noise, missed detections, or sensor uncertainty.

## 5. Simulation Configuration

| Parameter | Value |
|---|---:|
| Environment | 10 m × 10 m |
| Occupancy-grid resolution | 0.1 m |
| Grid dimensions | 100 × 100 |
| UAV radius | 0.25 m |
| Safety margin | 0.15 m |
| Obstacle inflation radius | 0.40 m |
| Obstacle geometry | Circular |
| Path planner | A* |
| A* neighbourhood | 8-connected |
| Path-tracking controller | PD |
| UAV dynamics | Planar translational model |
| LiDAR field of view | 360° |
| LiDAR angular resolution | 1° |
| LiDAR maximum range | 5 m |

## 6. Experimental Scenarios

Five scenarios are used to evaluate the navigation framework under different environment configurations.

| Scenario | Description |
|---|---|
| E1 – Reference Environment | Baseline indoor environment |
| E2 – Relocated Obstacles | Obstacles are placed at different locations |
| E3 – Narrow Passage | Navigation through a narrow passage |
| E4 – Cluttered Environment | Navigation in an environment with multiple obstacles |
| E5 – Different Start and Goal | Navigation with a modified start-goal configuration |

These scenarios are designed to examine how changes in obstacle placement, environmental complexity, and start-goal configuration affect path planning and trajectory tracking.

## 7. Experimental Results

The following results were obtained from the five simulation scenarios.

### 7.1 Navigation performance

| Performance metric | E1 | E2 | E3 | E4 | E5 |
|---|---:|---:|---:|---:|---:|
| A* expanded nodes | 1943 | 2170 | 70 | 1468 | 2365 |
| Global path length (m) | 10.720 | 10.661 | 7.029 | 10.661 | 11.087 |
| Trajectory length (m) | 10.429 | 10.410 | 6.982 | 10.360 | 10.745 |
| Final position error (m) | 0.02749 | 0.02749 | 0.02932 | 0.02749 | 0.03221 |
| Final speed (m/s) | 0.04793 | 0.04793 | 0.04856 | 0.04793 | 0.04971 |
| Simulation time (s) | 86.05 | 85.55 | 63.45 | 85.35 | 90.15 |
| Goal reached | Yes | Yes | Yes | Yes | Yes |
| Collision | No | No | No | No | No |

### 7.2 Result interpretation

The simulation results indicate that:

- The A* planner generated paths for all five tested scenarios.
- The UAV reached the specified goal in all five scenarios.
- No collision was reported in the five recorded runs.
- Final position errors ranged from approximately \(0.0275\,\text{m}\) to \(0.0322\,\text{m}\).
- The narrow-passage scenario required fewer expanded A* nodes than the other tested scenarios.

These results demonstrate successful operation of the implemented navigation pipeline for the selected test environments.

They should not be interpreted as a statistical performance guarantee or as evidence of successful navigation in arbitrary environments. The current evaluation is based on five designed simulation scenarios.

### 7.3 Generated outputs

The simulation generates scenario-wise results, including:

- Environment and obstacle information.
- Occupancy-grid data.
- Initial and final LiDAR point clouds.
- Planned paths.
- UAV trajectories.
- Simulation parameters.
- Navigation performance metrics.
- Visualization figures.

A combined summary is also maintained in:

`Results/combined_summary.csv`

The individual scenario result folders are:

- `Results/E1_Reference/`
- `Results/E2_Relocated_Obstacles/`
- `Results/E3_Narrow_Passage/`
- `Results/E4_Cluttered_Environment/`
- `Results/E5_Different_Start_Goal/`

## 8. Repository Structure

The project is organized into functional MATLAB directories.

```text
Indoor_UAV_Navigation/
│
├── README.md
├── main.m
├── run_navigation_simulation.m
│
├── Control/
│   ├── README.txt
│   └── pid_controller.m
│
├── Dynamics/
│   ├── README.txt
│   └── update_uav.m
│
├── Environment/
│   ├── create_environment.m
│   ├── create_occupancy_grid.m
│   ├── create_test_scenarios.m
│   └── plot_environment.m
│
├── Localization/
│   └── README.txt
│
├── Planning/
│   ├── README.txt
│   └── astar.m
│
├── Sensors/
│   ├── animate_lidar.m
│   ├── animate_navigation.m
│   ├── detect_obstacles.m
│   ├── process_lidar_scan.m
│   └── simulate_lidar.m
│
├── Results/
│   ├── README.txt
│   ├── combined_summary.csv
│   ├── E1_Reference/
│   ├── E2_Relocated_Obstacles/
│   ├── E3_Narrow_Passage/
│   ├── E4_Cluttered_Environment/
│   └── E5_Different_Start_Goal/
│
└── Tests/
    ├── test_astar_path_tracking.m
    ├── test_coupled_navigation.m
    ├── test_pid_dynamics.m
    └── test_scenario_validation.m
```

The `Localization` directory currently contains documentation only. A localization algorithm has not yet been implemented.

## 9. Software Requirements

- MATLAB.
- MATLAB-supported operating system.
- Project source files.

The framework is implemented in MATLAB and does not require ROS 2, Gazebo, PX4, or ArduPilot for its current simulation workflow.

The exact MATLAB release and any additional toolbox dependencies should be confirmed against the source code before publication.

## 10. Installation and Execution

### 10.1 Clone the repository

After creating the GitHub repository, clone it using:

```bash
git clone <YOUR_GITHUB_REPOSITORY_URL>
```

Navigate to the project directory:

```bash
cd Indoor_UAV_Navigation
```

### 10.2 Open the project in MATLAB

Open MATLAB and set the project root directory as the current working directory.

Ensure that the project folders are available on the MATLAB path.

### 10.3 Run the simulation

Execute the main entry-point script:

```matlab
main
```

The main script initiates the simulation workflow.

Refer to the source code for the available scenario configuration and execution options.

## 11. Current Scope and Limitations

The current implementation is intentionally limited to a known-map, two-dimensional navigation problem.

The main limitations are:

1. **Known environment:** The occupancy grid is generated from predefined environment information.
2. **No autonomous mapping:** LiDAR measurements are not currently used to construct or update the planning map.
3. **No SLAM:** Simultaneous localization and mapping has not been implemented.
4. **Assumed position availability:** The UAV position is assumed to be available to the navigation and control system. No localization algorithm is currently implemented.
5. **Simplified dynamics:** The UAV is represented using planar translational dynamics rather than a complete six-degree-of-freedom multirotor model.
6. **Static obstacles:** Dynamic obstacle detection and avoidance are outside the current scope.
7. **Idealized LiDAR:** Sensor noise, uncertainty, and missed detections are not modelled.
8. **Limited experimental scenarios:** Evaluation is currently based on five predefined simulation scenarios.

These limitations define the boundary of the current baseline implementation and provide clear directions for subsequent research.

## 12. Future Research Direction

The current project establishes a baseline for extending UAV navigation from known environments toward unknown and partially known environments.

Potential future developments include:

- LiDAR-based environment perception and mapping.
- Incremental occupancy-grid construction.
- Localization using onboard sensor measurements.
- SLAM-based navigation.
- Replanning in response to newly detected obstacles.
- Dynamic obstacle avoidance.
- More realistic UAV motion and flight-control models.
- Evaluation across a larger set of environments and repeated trials.

These extensions are proposed research directions and are not part of the current implementation.

## 13. Research Relevance and Broader Applicability

Although this project focuses on indoor UAV navigation, several of its underlying computational methods are relevant to other robotic platforms.

Occupancy-grid representation, graph-based path planning, collision checking, waypoint tracking, and sensor-based environment perception are also important in ground and autonomous surface robotic systems.

The methods developed here may provide a starting point for such applications, subject to appropriate modifications to the platform dynamics, sensor configuration, environmental representation, and control architecture.

The current implementation, however, is specifically a planar UAV simulation and does not claim to implement ground or surface robotic navigation.

## 14. Author

**Prashank Mishra**  
Junior Research Fellow  
Centre for Drone Technology  
Indian Institute of Technology Guwahati, India

## 15. Acknowledgements

[Add the relevant project, funding, supervisor, and institutional acknowledgements here after confirming the exact wording and permissions.]

## 16. License

[Specify the selected open-source license and include the corresponding `LICENSE` file before publishing the repository.]

---

**Project status:** MATLAB-based simulation of autonomous UAV navigation in known two-dimensional indoor environments.

**Development approach:** The current repository represents the foundational navigation stage. Subsequent research stages will be maintained separately to preserve a clear distinction between the baseline implementation and future extensions.
