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
    for seed in range(n_evals):
        res = run(seed = seed+10, gui=True, map_number=4, planner='rrt_star')
        planner_ct.append(res["global_planner"]["metrics"]["time"])
        planner_nodes.append(res["global_planner"]["metrics"]["nodes_wo_gc"])
        planner_nodes_gc.append(res["global_planner"]["metrics"]["nodes_w_gc"])


    print( f"========== {n_evals} Evaluations ==========")
    print(f"[Planner] Mean and STD of Computation Time: {np.around(np.average(planner_ct), decimals=2)}, {np.around(np.std(planner_ct))}")
    print(f"[Planner] Mean and STD of Number of Nodes: {np.around(np.average(planner_nodes))}, {np.around(np.std(planner_nodes))}")
    print(f"[Planner] Mean and STD of Number of Garbage Collected Nodes: {np.around(np.average(planner_nodes_gc))}, {np.around(np.std(planner_nodes_gc))}")
    print("=============================================")

def plot_graphs():
    pass

if __name__=="__main__":
    run_stat()