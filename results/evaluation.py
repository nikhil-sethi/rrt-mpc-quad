import sys
import os
import numpy as np

#to include subdirs as modules
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.dirname(SCRIPT_DIR))

from main import run

def run_stat(n_evals = 1):
    ## metrics to evalutate. add whatever you want here
    planner_ct = []
    planner_nodes = []
    planner_nodes_gc = []
    trajectory_generator_time = []
    for seed in range(n_evals):
        res = run(seed = seed, gui=True, map_number=1, planner='rrt', min_snap=False)
        planner_ct.append(res["global_planner"]["metrics"]["time"])
        planner_nodes.append(res["global_planner"]["metrics"]["nodes_wo_gc"])
        planner_nodes_gc.append(res["global_planner"]["metrics"]["nodes_w_gc"])
        trajectory_generator_time.append(res["traj_opt"]["metrics"]["time"])
        

    print( f"========== {n_evals} Evaluations ==========")
    print(f"[Planner] Mean and STD of Computation Time: {np.around(np.average(planner_ct), decimals=2)}, {np.around(np.std(planner_ct), decimals=2)}")
    print(f"[Planner] Mean and STD of Number of Nodes: {np.around(np.average(planner_nodes), decimals=2)}, {np.around(np.std(planner_nodes), decimals=2)}")
    print(f"[Planner] Mean and STD of Number of Garbage Collected Nodes: {np.around(np.average(planner_nodes_gc), decimals=2)}, {np.around(np.std(planner_nodes_gc), decimals=2)}")
    print(f"[Traj Opt] Mean and STD of Computation Time Trajectory Optimization: {np.around(np.average(trajectory_generator_time), decimals=5)}, {np.around(np.std(trajectory_generator_time), decimals=5)}")
    print("=============================================")

def plot_graphs():
    pass

if __name__=="__main__":
    run_stat()