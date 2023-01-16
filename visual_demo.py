import sys
import os
import numpy as np
import pybullet as p

#to include subdirs as modules
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.dirname(SCRIPT_DIR))

from main import run

def run_stat(n_evals = 1):
    planner_list = ['rrt', 'inf_rrt', 'rec_rrt', 'rrt_star', 'inf_rrt_star']
    map_number = 3
    gui = True
    plot_all = True
    planner_results = {
        'rrt': {
            'avg_time_m_std' : {},
            'avg_dist_m_std' : {},
            'avg_iter_num_m_std' : {}
        },
        'inf_rrt': {
            'avg_time_m_std' : {},
            'avg_dist_m_std' : {},
            'avg_iter_num_m_std' : {}
        },
        'rec_rrt': {
            'avg_time_m_std' : {},
            'avg_dist_m_std' : {},
            'avg_iter_num_m_std' : {}
        },
        'rrt_star': {
            'avg_time_m_std' : {},
            'avg_dist_m_std' : {},
            'avg_iter_num_m_std' : {}
        },
        'inf_rrt_star': {
            'avg_time_m_std' : {},
            'avg_dist_m_std' : {},
            'avg_iter_num_m_std' : {}
        }
    }
    for planner in planner_list:
        ## metrics to evalutate. add whatever you want here
        planner_ct = []
        path_distances = []
        planner_iterations = []
        seed = 1
        print(f"Beginning Map {map_number} Demonstration for the {planner} planner!")
        res = run(seed = seed, gui=gui, map_number=map_number, planner=planner, plot_all=plot_all)
        planner_ct.append(res["global_planner"]["metrics"]["time"])
        path_distances.append(res["global_planner"]["metrics"]["dist"])
        planner_iterations.append(res["global_planner"]["metrics"]["iter_num"])
        
        planner_results[planner]['avg_time_m_std'][f"map_{map_number}"] = [np.around(np.average(planner_ct), decimals=2), np.around(np.std(planner_ct), decimals=2)]
        planner_results[planner]['avg_dist_m_std'][f"map_{map_number}"] = [np.around(np.average(path_distances), decimals=2), np.around(np.std(path_distances), decimals=2)]
        planner_results[planner]['avg_iter_num_m_std'][f"map_{map_number}"] = [np.around(np.average(planner_iterations), decimals=2), np.around(np.std(planner_iterations), decimals=2)]

        print( f"========== {n_evals} Evaluations ==========")
        print( f"========== Map Number: {map_number} ==========")
        print( f"========== Planner: {planner} ==========")
        p_m_time = planner_results[planner]['avg_time_m_std'][f"map_{map_number}"]
        p_m_dist = planner_results[planner]['avg_dist_m_std'][f"map_{map_number}"]
        p_m_iter = planner_results[planner]['avg_iter_num_m_std'][f"map_{map_number}"]
        print(f"[Planner] Mean and STD of Computation Time: {p_m_time}")
        print(f"[Planner] Mean and STD of Final Path Distance: {p_m_dist}")
        print(f"[Planner] Mean and STD of Number of Iterations: {p_m_iter}")
        print("=============================================\n\n")
    
    print( f"========== Final Results for All Planners ==========")
    print( f"========== {n_evals} Evaluations ==========")
    print( f"========== Map Number: {map_number} ==========")
    for planner in planner_list:
        print( f"========== Planner: {planner} ==========")
        p_m_time = planner_results[planner]['avg_time_m_std'][f"map_{map_number}"]
        p_m_dist = planner_results[planner]['avg_dist_m_std'][f"map_{map_number}"]
        p_m_iter = planner_results[planner]['avg_iter_num_m_std'][f"map_{map_number}"]
        print(f"[Planner] Mean and STD of Computation Time: {p_m_time}")
        print(f"[Planner] Mean and STD of Final Path Distance: {p_m_dist}")
        print(f"[Planner] Mean and STD of Number of Iterations: {p_m_iter}")
        print("=============================================\n")

def plot_graphs():
    pass

if __name__=="__main__":
    run_stat()