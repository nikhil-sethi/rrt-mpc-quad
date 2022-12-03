"""Script demonstrating the joint use of simulation and control.

The simulation is run by a `CtrlAviary` or `VisionAviary` environment.
The control is given by the PID implementation in `DSLPIDControl`.

Example
-------
In a terminal, run as:

	$ python main.py

Notes
-----
The drones move, at different altitudes, along cicular trajectories 
in the X-Y plane, around point (0, -.3).

"""
import time
import argparse
import numpy as np
from maps import load_map, MAPS


from gym_pybullet_drones.utils.enums import DroneModel, Physics
from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
from gym_pybullet_drones.utils.Logger import Logger
from gym_pybullet_drones.utils.utils import sync, str2bool

import numpy as np
from gym import spaces

from gym_pybullet_drones.envs.BaseAviary import BaseAviary
from gym_pybullet_drones.utils.enums import DroneModel, Physics

class PlanAviary(CtrlAviary):
	"""Multi-drone environment class for control applications."""

	################################################################################

	def __init__(self,
				 drone_model: DroneModel=DroneModel.CF2X,
				 num_drones: int=1,
				 neighbourhood_radius: float=np.inf,
				 initial_xyzs=None,
				 initial_rpys=None,
				 physics: Physics=Physics.PYB,
				 freq: int=240,
				 aggregate_phy_steps: int=1,
				 gui=False,
				 record=False,
				 obstacles=False,
				 user_debug_gui=True,
				 output_folder='results',
				 map = 1
				 ):
		"""Initialization of an aviary environment for control applications.

		Parameters
		----------
		drone_model : DroneModel, optional
			The desired drone type (detailed in an .urdf file in folder `assets`).
		num_drones : int, optional
			The desired number of drones in the aviary.
		neighbourhood_radius : float, optional
			Radius used to compute the drones' adjacency matrix, in meters.
		initial_xyzs: ndarray | None, optional
			(NUM_DRONES, 3)-shaped array containing the initial XYZ position of the drones.
		initial_rpys: ndarray | None, optional
			(NUM_DRONES, 3)-shaped array containing the initial orientations of the drones (in radians).
		physics : Physics, optional
			The desired implementation of PyBullet physics/custom dynamics.
		freq : int, optional
			The frequency (Hz) at which the physics engine steps.
		aggregate_phy_steps : int, optional
			The number of physics steps within one call to `BaseAviary.step()`.
		gui : bool, optional
			Whether to use PyBullet's GUI.
		record : bool, optional
			Whether to save a video of the simulation in folder `files/videos/`.
		obstacles : bool, optional
			Whether to add obstacles to the simulation.
		user_debug_gui : bool, optional
			Whether to draw the drones' axes and the GUI RPMs sliders.

		"""
		self.map = map
		super().__init__(drone_model=drone_model,
						 num_drones=num_drones,
						 neighbourhood_radius=neighbourhood_radius,
						 initial_xyzs=initial_xyzs,
						 initial_rpys=initial_rpys,
						 physics=physics,
						 freq=freq,
						 aggregate_phy_steps=aggregate_phy_steps,
						 gui=gui,
						 record=record,
						 obstacles=obstacles,
						 user_debug_gui=user_debug_gui,
						 output_folder=output_folder,
						 )
		


	def _addObstacles(self):
		"""Add obstacles to the environment.
		"""
		load_map(MAPS[self.map], self.CLIENT)

					   
					   
DEFAULT_DRONES = DroneModel("cf2x")
DEFAULT_NUM_DRONES = 1
DEFAULT_PHYSICS = Physics("pyb")
DEFAULT_VISION = False
DEFAULT_GUI = True
DEFAULT_RECORD_VISION = False
DEFAULT_PLOT = True
DEFAULT_USER_DEBUG_GUI = False
DEFAULT_AGGREGATE = True
DEFAULT_OBSTACLES = True
DEFAULT_SIMULATION_FREQ_HZ = 240
DEFAULT_CONTROL_FREQ_HZ = 48
DEFAULT_DURATION_SEC = 12
DEFAULT_OUTPUT_FOLDER = 'results'
DEFAULT_COLAB = False
DEFAULT_MAP = 1
def run(
		drone=DEFAULT_DRONES,
		num_drones=DEFAULT_NUM_DRONES,
		physics=DEFAULT_PHYSICS,
		vision=DEFAULT_VISION,
		gui=DEFAULT_GUI,
		record_video=DEFAULT_RECORD_VISION,
		plot=DEFAULT_PLOT,
		user_debug_gui=DEFAULT_USER_DEBUG_GUI,
		aggregate=DEFAULT_AGGREGATE,
		obstacles=DEFAULT_OBSTACLES,
		simulation_freq_hz=DEFAULT_SIMULATION_FREQ_HZ,
		control_freq_hz=DEFAULT_CONTROL_FREQ_HZ,
		duration_sec=DEFAULT_DURATION_SEC,
		output_folder=DEFAULT_OUTPUT_FOLDER,
		colab=DEFAULT_COLAB,
		map = DEFAULT_MAP
		):
	#### Initialize the simulation #############################
	H = .1
	H_STEP = .05
	R = .3
	INIT_XYZS = np.array([[0.6,-0.6,0.5] for i in range(num_drones)])
	
	INIT_RPYS = np.array([[0, 0,  i * (np.pi/2)/num_drones] for i in range(num_drones)])
	AGGR_PHY_STEPS = int(simulation_freq_hz/control_freq_hz) if aggregate else 1

	#### Initialize a circular trajectory ######################
	PERIOD = 10
	NUM_WP = control_freq_hz*PERIOD 
	TARGET_POS = np.zeros((NUM_WP,3))

	for i in range(NUM_WP):
		TARGET_POS[i, :] = R*np.cos((i/NUM_WP)*(2*np.pi)+np.pi/2)+INIT_XYZS[0, 0], R*np.sin((i/NUM_WP)*(2*np.pi)+np.pi/2)-R+INIT_XYZS[0, 1], 0
	wp_counters = np.array([int((i*NUM_WP/6)%NUM_WP) for i in range(num_drones)])

	# for i in range(NUM_WP):
	# 	if i>NUM_WP//2:
	# 		pass
	# 	else:
	# 		TARGET_POS[i, :] = 
	#### Debug trajectory ######################################
	#### Uncomment alt. target_pos in .computeControlFromState()
	# INIT_XYZS = np.array([[.3 * i, 0, .1] for i in range(num_drones)])
	# INIT_RPYS = np.array([[0, 0,  i * (np.pi/3)/num_drones] for i in range(num_drones)])
	# NUM_WP = control_freq_hz*15
	# TARGET_POS = np.zeros((NUM_WP,3))
	# for i in range(NUM_WP):
	#     if i < NUM_WP/6:
	#         TARGET_POS[i, :] = (i*6)/NUM_WP, 0, 0.5*(i*6)/NUM_WP
	#     elif i < 2 * NUM_WP/6:
	#         TARGET_POS[i, :] = 1 - ((i-NUM_WP/6)*6)/NUM_WP, 0, 0.5 - 0.5*((i-NUM_WP/6)*6)/NUM_WP
	#     elif i < 3 * NUM_WP/6:
	#         TARGET_POS[i, :] = 0, ((i-2*NUM_WP/6)*6)/NUM_WP, 0.5*((i-2*NUM_WP/6)*6)/NUM_WP
	#     elif i < 4 * NUM_WP/6:
	#         TARGET_POS[i, :] = 0, 1 - ((i-3*NUM_WP/6)*6)/NUM_WP, 0.5 - 0.5*((i-3*NUM_WP/6)*6)/NUM_WP
	#     elif i < 5 * NUM_WP/6:
	#         TARGET_POS[i, :] = ((i-4*NUM_WP/6)*6)/NUM_WP, ((i-4*NUM_WP/6)*6)/NUM_WP, 0.5*((i-4*NUM_WP/6)*6)/NUM_WP
	#     elif i < 6 * NUM_WP/6:
	#         TARGET_POS[i, :] = 1 - ((i-5*NUM_WP/6)*6)/NUM_WP, 1 - ((i-5*NUM_WP/6)*6)/NUM_WP, 0.5 - 0.5*((i-5*NUM_WP/6)*6)/NUM_WP
	# wp_counters = np.array([0 for i in range(num_drones)])

	#### Create the environment with or without video capture ##

	env = PlanAviary(drone_model=drone,
						num_drones=num_drones,
						initial_xyzs=INIT_XYZS,
						initial_rpys=INIT_RPYS,
						physics=physics,
						neighbourhood_radius=10,
						freq=simulation_freq_hz,
						aggregate_phy_steps=AGGR_PHY_STEPS,
						gui=gui,
						record=record_video,
						obstacles=obstacles,
						user_debug_gui=user_debug_gui,
						map=map
						)

	#### Obtain the PyBullet Client ID from the environment ####
	PYB_CLIENT = env.getPyBulletClient()

	#### Initialize the logger #################################
	logger = Logger(logging_freq_hz=int(simulation_freq_hz/AGGR_PHY_STEPS),
					num_drones=num_drones,
					output_folder=output_folder,
					colab=colab
					)

	#### Initialize the controllers ############################
	ctrl = [DSLPIDControl(drone_model=drone) for i in range(num_drones)]

	#### Run the simulation ####################################
	CTRL_EVERY_N_STEPS = int(np.floor(env.SIM_FREQ/control_freq_hz))
	action = {str(i): np.array([0,0,0,0]) for i in range(num_drones)}
	
	START = time.time()
	for i in range(0, int(duration_sec*env.SIM_FREQ), AGGR_PHY_STEPS):

		#### Step the simulation ###################################
		action = {str(i): np.array([0,0,0,0]) for i in range(num_drones)}
		obs, reward, done, info = env.step(action)

		#### Compute control at the desired frequency ##############
		if i%CTRL_EVERY_N_STEPS == 0:

			#### Compute control for the current way point #############
			for j in range(num_drones):
				action[str(j)], _, _ = ctrl[j].computeControlFromState(control_timestep=CTRL_EVERY_N_STEPS*env.TIMESTEP,
																	   state=obs[str(j)]["state"],
																	   target_pos=np.hstack([TARGET_POS[wp_counters[j], 0:2], INIT_XYZS[j, 2]]),
																	   # target_pos=INIT_XYZS[j, :] + TARGET_POS[wp_counters[j], :],
																	   target_rpy=INIT_RPYS[j, :]
																	   )
								

			#### Go to the next way point and loop #####################
			for j in range(num_drones): 
				wp_counters[j] = wp_counters[j] + 1 if wp_counters[j] < (NUM_WP-1) else 0

		#### Log the simulation ####################################
		for j in range(num_drones):
			logger.log(drone=j,
					   timestamp=i/env.SIM_FREQ,
					   state=obs[str(j)]["state"],
					   control=np.hstack([TARGET_POS[wp_counters[j], 0:2], INIT_XYZS[j, 2], INIT_RPYS[j, :], np.zeros(6)])
					   # control=np.hstack([INIT_XYZS[j, :]+TARGET_POS[wp_counters[j], :], INIT_RPYS[j, :], np.zeros(6)])
					   )

		#### Printout ##############################################
		if i%env.SIM_FREQ == 0:
			env.render()
			
		#### Sync the simulation ###################################
		if gui:
			sync(i, START, env.TIMESTEP)

	#### Close the environment #################################
	env.close()

	#### Save the simulation results ###########################
	logger.save()
	logger.save_as_csv("pid") # Optional CSV save

	#### Plot the simulation results ###########################
	if plot:
		logger.plot()

if __name__ == "__main__":
	#### Define and parse (optional) arguments for the script ##
	parser = argparse.ArgumentParser(description='Helix flight script using CtrlAviary or VisionAviary and DSLPIDControl')
	parser.add_argument('--drone',              default=DEFAULT_DRONES,     type=DroneModel,    help='Drone model (default: CF2X)', metavar='', choices=DroneModel)
	parser.add_argument('--num_drones',         default=DEFAULT_NUM_DRONES,          type=int,           help='Number of drones (default: 3)', metavar='')
	parser.add_argument('--physics',            default=DEFAULT_PHYSICS,      type=Physics,       help='Physics updates (default: PYB)', metavar='', choices=Physics)
	parser.add_argument('--vision',             default=DEFAULT_VISION,      type=str2bool,      help='Whether to use VisionAviary (default: False)', metavar='')
	parser.add_argument('--gui',                default=DEFAULT_GUI,       type=str2bool,      help='Whether to use PyBullet GUI (default: True)', metavar='')
	parser.add_argument('--record_video',       default=DEFAULT_RECORD_VISION,      type=str2bool,      help='Whether to record a video (default: False)', metavar='')
	parser.add_argument('--plot',               default=DEFAULT_PLOT,       type=str2bool,      help='Whether to plot the simulation results (default: True)', metavar='')
	parser.add_argument('--user_debug_gui',     default=DEFAULT_USER_DEBUG_GUI,      type=str2bool,      help='Whether to add debug lines and parameters to the GUI (default: False)', metavar='')
	parser.add_argument('--aggregate',          default=DEFAULT_AGGREGATE,       type=str2bool,      help='Whether to aggregate physics steps (default: True)', metavar='')
	parser.add_argument('--obstacles',          default=DEFAULT_OBSTACLES,       type=str2bool,      help='Whether to add obstacles to the environment (default: True)', metavar='')
	parser.add_argument('--simulation_freq_hz', default=DEFAULT_SIMULATION_FREQ_HZ,        type=int,           help='Simulation frequency in Hz (default: 240)', metavar='')
	parser.add_argument('--control_freq_hz',    default=DEFAULT_CONTROL_FREQ_HZ,         type=int,           help='Control frequency in Hz (default: 48)', metavar='')
	parser.add_argument('--duration_sec',       default=DEFAULT_DURATION_SEC,         type=int,           help='Duration of the simulation in seconds (default: 5)', metavar='')
	parser.add_argument('--output_folder',     default=DEFAULT_OUTPUT_FOLDER, type=str,           help='Folder where to save logs (default: "results")', metavar='')
	parser.add_argument('--colab',              default=DEFAULT_COLAB, type=bool,           help='Whether example is being run by a notebook (default: "False")', metavar='')
	parser.add_argument('--map',              default=DEFAULT_MAP, type=int,           help='Map number")', metavar='')
	ARGS = parser.parse_args()

	run(**vars(ARGS))
