import sys
import os

#to include subdirs as modules
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.dirname(SCRIPT_DIR))

from main import run

def run_stat(n_evals = 10):
    ## metrics to evalutate. add whatever you want here
    planner_ct = 0
    planner_nodes = 0
    planner_nodes_gc = 0
    for seed in range(n_evals):
        res = run(seed = seed, gui=False, map_number=2)
        planner_ct += res["global_planner"]["metrics"]["time"]
        planner_nodes += res["global_planner"]["metrics"]["nodes_wo_gc"]
        planner_nodes_gc += res["global_planner"]["metrics"]["nodes_w_gc"]


    print( f"========== {n_evals} Evaluations ==========")
    print("[Planner] Average computation time: ", planner_ct/n_evals)
    print("[Planner] Average nodes: ", planner_nodes/n_evals)
    print("[Planner] Average garbage collected nodes: ", planner_nodes_gc/n_evals)
    print("=============================================")

def plot_graphs():
    pass

if __name__=="__main__":
    run_stat()