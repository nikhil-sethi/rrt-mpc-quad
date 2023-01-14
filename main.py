""" =========== main.py ============
Main script to start environment and planner

Example
-------
In a terminal, run as:

	$ python3 main.py --map_number 2 --planner "rrt_star" --min_snap True

"""
import time
import argparse
import numpy as np
import sys
import os
import random


# to include subdirs as modules
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.dirname(SCRIPT_DIR))

from gym_pybullet_drones.utils.enums import DroneModel
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
from gym_pybullet_drones.utils.Logger import Logger
from gym_pybullet_drones.utils.utils import sync, str2bool
from gym_pybullet_drones.utils.enums import DroneModel, Physics

from utils.color import Color
from utils import printRed
from environment import Env


def run(
		drone=DroneModel("cf2x"),
		num_drones=1,
		gui=True,
		record_video=False,
		plot=False,
		user_debug_gui=False,
		aggregate=True,
		obstacles=True,
		simulation_freq_hz=240,
		control_freq_hz=48,
		duration_sec=60,
		output_folder="results/logs/",
		map_number = 1,
		planner = "inf_rrt_star",
		min_snap = True,
		seed = None,
		plot_all = False,
		corridor=False
		):

	random.seed(seed)
	np.random.seed(seed)

	# dictionary which stores all evaluations + info. should be passed around to objects to collect data
	# some fields can be empty depending on cmdline options
	result = {
		"seed":seed,
		"global_planner":{
			"name":planner,
			"metrics":{
				"dist" : 0
			}
		},
		"traj_opt":{
			"state":min_snap,
			"metrics":{

			}
		},
		"pos_error_mean": 0,
		"pos_error_std": 0,
		"text_output": "\n ---- \n",
		"map": map_number
	}
	## Initialize the simulation
	# setup states
	init_att = [0, 0,  (np.pi/2)]

	AGGR_PHY_STEPS = int(simulation_freq_hz/control_freq_hz) if aggregate else 1
	
	#### Create the environment with or without video capture ##

	env = Env(
		map_number = map_number,
		freq=simulation_freq_hz,
		aggregate_phy_steps=AGGR_PHY_STEPS,
		gui=gui,
		record=record_video,
		obstacles=obstacles,
		user_debug_gui=user_debug_gui,
		result = result,
		plot_all = plot_all
		)

	import cProfile
	import pstats
	# with cProfile.Profile() as pr:
	plan = env.plan(method=planner, min_snap=min_snap, corridor=corridor)
	# stats = pstats.Stats(pr)
	# stats.sort_stats(pstats.SortKey.TIME)
	# stats.print_stats()
	NUM_WP = plan.shape[0]
	wp_counter = 0

	if gui:
		env.plot_plan(plan, WPS=False)

	## Initialize the logger 
	logger = Logger(logging_freq_hz=int(simulation_freq_hz/AGGR_PHY_STEPS),
					num_drones=num_drones,
					output_folder=output_folder,
					colab=False
					)

	## Initialize the controller
	controller = DSLPIDControl(drone_model=drone)

	## Run the simulation 
	CTRL_EVERY_N_STEPS = int(np.floor(env.SIM_FREQ/control_freq_hz))
	action = {"0": np.array([0,0,0,0])}
	
	START = time.time()
	pos_err = []
	for i in range(0, int(duration_sec*env.SIM_FREQ), AGGR_PHY_STEPS):

		## Step the simulation 
		# action = {str(i): np.array([0,0,0,0]) for i in range(num_drones)}
		
		obs, reward, done, info = env.step(action)

		#### Compute control at the desired frequency ##############
		if i%CTRL_EVERY_N_STEPS == 0:

			## Compute control for the current way point 
			action["0"], _, _ = controller.computeControlFromState(control_timestep=CTRL_EVERY_N_STEPS*env.TIMESTEP,
																	state=obs["0"]["state"],
																	target_pos=plan[wp_counter, 0:3],
																	target_rpy=init_att
																	)
			pos = obs["0"]["state"][:3]
			
			if gui:								
				env.plot_point(pos, color=Color.BLUE)
			
			# Go to the next way point and loop 
			# Changed "else 0" to "else wp_counters[j]" to keep drone at endpoint
			if wp_counter<(NUM_WP-1):
				wp_counter += 1
				count = i + 150*CTRL_EVERY_N_STEPS	# num steps to wait after reaching the goal

				# record error in trajectory. Only record upto
				pos_err.append(np.linalg.norm(pos - plan[wp_counter, 0:3]))	
				
			else: 
				wp_counter = wp_counter
				if count%i ==0:
					break
		# Log the simulation 
		logger.log(drone=0,
				timestamp=i/env.SIM_FREQ,
				state=obs["0"]["state"],
				control=np.hstack([plan[wp_counter, 0:2], env.map.starting_pos[2], init_att, np.zeros(6)])
				# control=np.hstack([INIT_XYZS[j, :]+TARGET_POS[wp_counters[j], :], INIT_RPYS[j, :], np.zeros(6)])
				)	
		## Sync the simulation 
		if gui:
			sync(i, START, env.TIMESTEP)

	# compile metrics
	result['pos_error_mean'] = np.mean(pos_err)
	result['pos_error_std'] = np.std(pos_err)

	## Close the environment 
	env.close()

	## Save the simulation results 
	logger.save()
	# logger.save_as_csv("pid") # Optional CSV save

	## Plot the simulation results 
	if plot:
		logger.plot()
	
	return result

if __name__ == "__main__":
	## Define and parse (optional) arguments for the script ##
	parser = argparse.ArgumentParser(description='Helix flight script using CtrlAviary or VisionAviary and DSLPIDControl')
	parser.add_argument('--num_drones',         default=1,          		  type=int,           help='Number of drones (default: 3)', metavar='')
	parser.add_argument('--gui',                default=True,       		  type=str2bool,      help='Whether to use PyBullet GUI (default: True)', metavar='')
	parser.add_argument('--record_video',       default=False,     			  type=str2bool,      help='Whether to record a video (default: False)', metavar='')
	parser.add_argument('--plot',               default=False,      		  type=str2bool,      help='Whether to plot the simulation results (default: True)', metavar='')
	parser.add_argument('--simulation_freq_hz', default=240,        		  type=int,           help='Simulation frequency in Hz (default: 240)', metavar='')
	parser.add_argument('--control_freq_hz',    default=48,         		  type=int,           help='Control frequency in Hz (default: 48)', metavar='')
	parser.add_argument('--duration_sec',       default=60,         		  type=int,           help='Duration of the simulation in seconds (default: 5)', metavar='')
	parser.add_argument('--output_folder',     	default='results/logs/', 	type=str,           help='Folder where to save logs (default: "results")', metavar='')
	parser.add_argument('--map_number',         default=3, 					      type=int,           help='Map number (default: "Map 0")', metavar='')
	parser.add_argument('--planner',            default="inf_rrt_star", 	type=str,           help='Planner (default: "rrt_star")', metavar='')
	parser.add_argument('--min_snap',           default=True, 				    type=str2bool,      help='Min Snap (default: False)', metavar=''),
	parser.add_argument('--corridor',           default=False, 		        type=str2bool,      help='Corridor constraints (default: False)', metavar=''),
	parser.add_argument('--seed',              	default=None, 		        type=int,           help='Seed (default: None)', metavar=''),
	parser.add_argument('--plot_all',           default=True, 		        type=str2bool,      help='Will plot all nodes and connections (default: False)', metavar='')
  
	ARGS = parser.parse_args()

	result = run(**vars(ARGS))

	result["text_output"] += " -----"
	printRed("============== Results ==============")
	for key in result:
		printRed(f"{key} -> {result[key]}")
	printRed("=====================================")