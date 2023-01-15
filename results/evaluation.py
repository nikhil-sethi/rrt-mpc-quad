import sys
import os
import numpy as np
import pybullet as p

#to include subdirs as modules
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.dirname(SCRIPT_DIR))

from utils import printC
from utils.color import PrintColor
from main import run

def run_stat(n_evals = 1):
    planner_list = ['rrt', 'inf_rrt', 'rrt_star', 'inf_rrt_star']
    map_number_list = [0,1,2,3,4,5,6]
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
        for map_number in map_number_list:
            ## metrics to evalutate. add whatever you want here
            planner_ct = []
            path_distances = []
            planner_iterations = []
            for i in range(n_evals):
                seed = 10*i + 777
                printC(f"Beginning Evaluation {i+1}/{n_evals} for the {planner} planner on map number {map_number}!", color=PrintColor.BOLD+PrintColor.OKGREEN)
                res = run(seed = seed, gui=False, map_number=map_number, planner=planner, plot_all=False)
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
    for planner in planner_list:
        print( f"========== Planner: {planner} ==========")
        for map_number in map_number_list:
            print( f"========== Map Number: {map_number} ==========")
            p_m_time = planner_results[planner]['avg_time_m_std'][f"map_{map_number}"]
            p_m_dist = planner_results[planner]['avg_dist_m_std'][f"map_{map_number}"]
            p_m_iter = planner_results[planner]['avg_iter_num_m_std'][f"map_{map_number}"]
            print(f"[Planner] Mean and STD of Computation Time: {p_m_time}")
            print(f"[Planner] Mean and STD of Final Path Distance: {p_m_dist}")
            print(f"[Planner] Mean and STD of Number of Iterations: {p_m_iter}")
        print("=============================================\n")

    f = open(f'{SCRIPT_DIR}/results.txt', 'w')
    f.write( f"========== Final Results for All Planners ==========\n")
    f.write( f"========== {n_evals} Evaluations ==========\n")
    for planner in planner_list:
        f.write( f"========== Planner: {planner} ==========\n")
        for map_number in map_number_list:
            f.write( f"========== Map Number: {map_number} ==========\n")
            p_m_time = planner_results[planner]['avg_time_m_std'][f"map_{map_number}"]
            p_m_dist = planner_results[planner]['avg_dist_m_std'][f"map_{map_number}"]
            p_m_iter = planner_results[planner]['avg_iter_num_m_std'][f"map_{map_number}"]
            f.write(f"[Planner] Mean and STD of Computation Time: {p_m_time}\n")
            f.write(f"[Planner] Mean and STD of Final Path Distance: {p_m_dist}\n")
            f.write(f"[Planner] Mean and STD of Number of Iterations: {p_m_iter}\n")
        f.write("=============================================\n\n")
    f.write( f"========== RAW DATA ==========\n")    
    f.write( f"========== {n_evals} Evaluations ==========\n")
    f.write(f"{'Planner'}\t{'Map Number'}\t{'Average Time'}\t{'STD Time'}\t{'Average Dist'}\t{'STD Dist'}\t{'Average Iter'}\t{'STD Iter'}\n")
    for planner in planner_list:
        for map_number in map_number_list:
            f.write( f"{planner}\t")
            f.write( f"{map_number}\t")
            p_m_time = planner_results[planner]['avg_time_m_std'][f"map_{map_number}"]
            p_m_dist = planner_results[planner]['avg_dist_m_std'][f"map_{map_number}"]
            p_m_iter = planner_results[planner]['avg_iter_num_m_std'][f"map_{map_number}"]
            f.write(f"{p_m_time[0]}\t{p_m_time[1]}\t")
            f.write(f"{p_m_dist[0]}\t{p_m_dist[1]}\t")
            f.write(f"{p_m_iter[0]}\t{p_m_iter[1]}\t\n")
        # f.write("\n")
    f.close()


def plot_graphs():
    pass

if __name__=="__main__":
    run_stat()