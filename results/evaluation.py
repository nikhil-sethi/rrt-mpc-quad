import sys
import os
import numpy as np

#to include subdirs as modules
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.dirname(SCRIPT_DIR))

from main import run

def run_stat(n_evals = 3):
    planner = 'inf_rrt_star'
    map_number = 3
    ## metrics to evalutate. add whatever you want here
    planner_ct = []
    planner_nodes = []
    planner_nodes_gc = []
    path_distances = []
    for seed in range(n_evals):
        print(f"Beginning Evaluation {seed}/{n_evals}")
        res = run(seed = seed, gui=False, map_number=map_number, planner=planner)
        planner_ct.append(res["global_planner"]["metrics"]["time"])
        planner_nodes.append(res["global_planner"]["metrics"]["nodes_wo_gc"])
        planner_nodes_gc.append(res["global_planner"]["metrics"]["nodes_w_gc"])
        path_distances.append(res["global_planner"]["metrics"]["dist"])


    print( f"========== {n_evals} Evaluations ==========")
    print( f"========== Planner: {planner} ==========")
    print( f"========== Map Number: {map_number} ==========")
    print(f"[Planner] Mean and STD of Final Path Distance: {np.around(np.average(path_distances), decimals=2)}, {np.around(np.std(path_distances), decimals=2)}")
    print(f"[Planner] Mean and STD of Computation Time: {np.around(np.average(planner_ct), decimals=2)}, {np.around(np.std(planner_ct), decimals=2)}")
    print(f"[Planner] Mean and STD of Number of Nodes: {np.around(np.average(planner_nodes), decimals=2)}, {np.around(np.std(planner_nodes), decimals=2)}")
    print(f"[Planner] Mean and STD of Number of Garbage Collected Nodes: {np.around(np.average(planner_nodes_gc), decimals=2)}, {np.around(np.std(planner_nodes_gc), decimals=2)}")
    print("=============================================")

def plot_graphs():
    pass

if __name__=="__main__":
    run_stat()