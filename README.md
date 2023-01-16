# rrt-mpc-quad
Repository for PDM project - Group 11

## Setup
```
git clone git@github.com:nikhil-sethi/rrt-mpc-quad.git
cd rrt-mpc-quad
chmod +x ./tooling/install.sh
./tooling/install.sh
```

## Run
General run command (while in repository folder):
```
python3 main.py [<options>]
```
This runs the planner and the simulation. Six maps have been implemented (numbered 0 to 5) and 5 different planner variants can be chosen:
| Planner | Argument |
| ------- | -------- |
| RRT | `rrt` |
| Informed RRT | `inf_rrt` |
| RRT with recycling | `rec_rrt` |
| RRT* | `rrt_star` |
| Informed RRT* | `inf_rrt_star` |

For example, running informed RRT* on map 2: 
```
python3 main.py --planner='inf_rrt_star' --map_number=2
```

Reference the following table for all run options (ordered by relevancy):


| Option argument | Argument Type | Default | Explanation |
| --------------- | ------------- | ------- | ---------- |
| `--planner` | see table above | `rrt_star` | Which planner to use
| `--map_number` | 0-5 | 1 | Which map to use 
| `--min_snap` | True/False | True | Trajectory optimization flag
| `--corridor` | True/False | False | Equality corridor for trajectory optimization
| `--plot` | True/False | False | Plots control graphs at the end
| `--plot_all` | True/False | False | To plot all nodes and lines
| `--seed` | int | None | Seed value for reproducibility
| `--duration_sec` | int | 60 | Duration of the simulation 
| `--output_folder` | str | `results/logs/` | Folder for logs
| `--record_video` | True/False | False | Whether to record a video 
| `--gui` | True/False | True | Whether to use PyBullet GUI 
| `--simulation_freq_hz` | int | 240 | Simulation frequency in Hz 
| `--control_freq_hz` | int | 48 | Control frequency in Hz 

## Class Diagram
<img src="media/class_diagram.png" alt="Class Diagram"> 

## Maps
### Map 0
<img src="media/Map0.png" alt="Map 0" width="450"> 

### Map 1
<img src="media/Map1.png" alt="Map 1" width="450"> 

### Map 2
<img src="media/Map2.png" alt="Map 2" width="450"> 

### Map 3
<img src="media/Map3.png" alt="Map 3" width="450"> 

### Map 4
<img src="media/Map4.png" alt="Map 4" width="450"> 

### Map 5
<img src="media/Map5.png" alt="Map 5" width="450"> 

### Map 6
<img src="media/Map6.png" alt="Map 6" width="450"> 

## MPC (under development)

There is also a development branch called `feat/mpc`. This branch consists of an initial setup to working with mpc as a local planner in combination with one of the sampling based global planners.

