# Path Planning Project writeup

The goal of the project was to develop software for safe driving a car around a virtual highway.
This writeup describes the approach taken to complete the project.

## Design

The design was choosen with respect to the giagram shown in the classroom.

![text](img/components.png)

The projects has three main components listed below:
- Prediction
- Behavior Planner
- Trajectory Generator

The Localization, Sensor Fusion and Motion Control components were not implemented as they were already provided as parts of the Term3 simulator.

## Data flow (inputs/outputs)

The developed software runs an infinite loop; at each iteration it takes in the output of the simulator and produces a trajectory for ego-car which is sent back to the simulator.

A highlevel data-flow diagram is presented below:

![data flow](img/data-flow.png)

## Project structure

The `src/` directory of the repository contains the project's source code.
Here you can find descriptions of each file:

| File(s) | Purpose |
|------|---------|
| `main.cpp` | The entrypoint the project. |
| `utils.h` | Contains helper functions used in different places in the project. |
| `vehicle.h` and `vehicle.cpp` | Describe a vehicle's state such as x, y, s, d coordinates, velocity and yaw. |
| `road.h` and `road.cpp` | Contains a road definition and useful helper methods such as `get_lane_center()`. |
| `spline.h` | Spline interpolation implementation provided in the classroom. |
| `json.hpp` | A module for working with JSONs |
| `fsm.h` | Defines the Finite State Machine used in the project. |
| `cost.h` | Contains the cost functions used in the behavior module. |
| `prediction.h` and `prediction.cpp` | Implementation of the Prediction module. |
| `behavior.h` and `behavior.cpp` | Implementation of the Behavior module. |
| `traj_gen.h` and `traj_gen.cpp` | Implementation of the Trajectory Generator module. |

## Prediction

## Behavior Planner

### FSM and its states

## Trajectory Generator

### Smoothing

## Further improvements

what can be improved?