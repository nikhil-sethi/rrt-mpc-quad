import numpy as np
import pybullet as p
import time

from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.utils.enums import DroneModel, Physics

from planner.sample_based import RRT, Informed_RRT, Recycle_RRT
from planner.sample_based import RRT_Star, Informed_RRT_Star
from planner.spaces import Space
from planner.graph import Node
from planner.trajectory import MinVelAccJerkSnapCrackPop,MinVelAccJerkSnapCrackPopCorridor

from maps import Map
from utils.color import Color
from utils import printRed

class Env(CtrlAviary):
	"""Multi-drone environment class for control applications."""

	################################################################################

	def __init__(self,
				 map_number = 0,
				 drone_model: DroneModel=DroneModel.CF2X,
				 num_drones: int=1,
				 neighbourhood_radius: float=np.inf,
				 initial_rpys = (0, 0,  (np.pi/2)),
				 physics: Physics=Physics.PYB,
				 freq: int=240,
				 aggregate_phy_steps: int=1,
				 gui=False,
				 record=False,
				 obstacles=False,
				 user_debug_gui=True,
				 output_folder='results',
				 result = {},
				 plot_all = False
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
		# Initiate Map
		self.map = Map(map_number=map_number)
		self.result = result
		self.plot_all = plot_all
		self.gui = gui
		super().__init__(drone_model=drone_model,
						 num_drones=num_drones,
						 neighbourhood_radius=neighbourhood_radius,
						 initial_xyzs=np.array([self.map.starting_pos]),
						 initial_rpys=np.array([initial_rpys]),
						 physics=physics,
						 freq=freq,
						 aggregate_phy_steps=aggregate_phy_steps,
						 gui=gui,
						 record=record,
						 obstacles=obstacles,
						 user_debug_gui=user_debug_gui,
						 output_folder=output_folder,
						 )
		
		# Pause to set camera
		# time.sleep(15)	
	
	@staticmethod
	def plot_point(position, color:Color = Color.WHITE, pointSize=2):
		"""Used to plot trajectories"""
		id = p.addUserDebugPoints(pointPositions = [position], pointColorsRGB = [color.value[:-1]], pointSize=pointSize)
		return id

	@staticmethod
	def plot_line(from_pos, to_pos, color:Color = Color.WHITE, lineWidth=1.4):
		"""Used to plot trajectories"""
		node_id = p.addUserDebugLine(lineFromXYZ = from_pos, lineToXYZ = to_pos, lineColorRGB=list(color.value[:-1]), lineWidth=lineWidth)
		return node_id

	@staticmethod
	def remove_line(lineID):
		p.removeUserDebugItem(lineID)

	def _addObstacles(self):
		"""Add obstacles to the environment.
		"""
		# Dilate obstacles to drone radius plus margin also equal to drone radius 
		self.map.load_map(self.CLIENT, dilate=True, dilation=2.1*self.L)
		
		# # uncomment below snippet to plot extents for obstacles
		# for obs in self.map.obstacles:
		# 	for i in range(2): 
		# 		self.plot_point(obs.extent[i])
		# 	self.plot_line(obs.extent[0], obs.extent[1])

	def plan(self, method="rrt_star", min_snap=False, d=100):	
		start = Node(pos = self.map.starting_pos)
		goal = Node(pos = self.map.goal_pos)
		ws = Space(low = self.map.ws_ll, high = self.map.ws_ul)
		if method == 'rrt':
			planner = RRT(space=ws, start=start, goal=goal, map=self.map.obstacles, env=self, result = self.result, map_number=self.map.map_number)
		elif method == 'inf_rrt':
			planner = Informed_RRT(space=ws, start=start, goal=goal, map=self.map.obstacles, env=self, result = self.result, map_number=self.map.map_number)
		elif method == 'rec_rrt':
			planner = Recycle_RRT(space=ws, start=start, goal=goal, map=self.map.obstacles, env=self, result = self.result, map_number=self.map.map_number)
		elif method == 'rrt_star':
			planner = RRT_Star(space=ws, start=start, goal=goal, map=self.map.obstacles, env=self, result = self.result, map_number=self.map.map_number)
		elif method == 'inf_rrt_star':
			planner = Informed_RRT_Star(space=ws, start=start, goal=goal, map=self.map.obstacles, env=self, result = self.result, map_number=self.map.map_number)
		else:
			raise NotImplementedError()
		printRed(f"Begin waypoint planning with {method}...")
		self.result["text_output"] += f" Begin waypoint planning with {method}...\n"
		start_time = time.perf_counter()
		wps = planner.run()
		elapsed_time_planner = time.perf_counter() - start_time
		self.result["global_planner"]["metrics"]["time"] = elapsed_time_planner

		printRed(f"Planning complete. Elapsed time: {elapsed_time_planner} seconds")
		printRed("---")
		self.result["text_output"] += f" Planning complete. Elapsed time: {elapsed_time_planner} seconds\n ---\n"

		if min_snap:
			printRed(f"Begin trajectory optimization with Minimum Snap")
			self.result["text_output"] += f" Begin trajectory optimization with Minimum Snap\n"
		else:
			printRed(f"Begin trajectory optimization with Linear Discretization")
			self.result["text_output"] += f" Begin trajectory optimization with Linear Discretization\n"
		
		plan_dist = planner.fastest_route_to_end # total distance covered by waypoints
		self.result["global_planner"]["metrics"]["dist"] = plan_dist

		num_pts = int(plan_dist//0.01) # trajectory seems to work best with current control system when discretisation is around 1 cm. Weird but ok.

		# some preprocessing on waypoints
		# convert to numpy array
		wps = planner.nodes_to_array(wps)
		
		# making sure waypoints are unique. RRT can glitch sometimes
		wps_sorted, _, idx = np.unique(wps, axis=0, return_index=True, return_inverse=True)
		# print(wps_sorted, idx)
		if len(idx)!=len(wps_sorted):
			
			wps = wps_sorted[idx[:-1],:] # np unique returns sorted values for some reason. undo that shit
		else: # if everything was already unique. Need this stuff coz np unique gives wrong vals sometimes
			wps = wps_sorted[idx,:]

		if self.gui:
			self.plot_plan(wps, WPS=True)
		# path optimization 
		if min_snap:
			try:
				start_time = time.perf_counter()
				traj_opt = MinVelAccJerkSnapCrackPop(order=2, waypoints = wps.T, time=8)	# don't worry about time argument tooplanner much. it's all relative
				plan = traj_opt.optimize(num_pts=num_pts)
				elapsed_time_opt = time.perf_counter() - start_time
				self.result["traj_opt"]["metrics"]["time"] = elapsed_time_opt 
				
				printRed(f"Optimization complete. Elapsed time: {elapsed_time_opt} seconds")
				self.result["text_output"] += f" Optimization complete. Elapsed time: {elapsed_time_opt} seconds\n"

				# traj_opt.plot(plan) # uncomment to plot plan in matplotlib. but note that --gui must be False
			except: # because min snap fails sometimes because of rank errors. still to debug
				# in that case, just go ahead with original waypoints and discretisation
				printRed("Minimum Snap failed. Resorting to linear discretization.")
				self.result["text_output"] += " Minimum Snap failed. Resorting to linear discretization.\n"
				start_time = time.perf_counter()
				plan = planner.discretize_path(wps, num_steps=int(num_pts/len(wps)))
				elapsed_time_opt = time.perf_counter() - start_time
				self.result["traj_opt"]["metrics"]["time"] = elapsed_time_opt 		
				
				printRed(f"Optimization complete. Elapsed time: {elapsed_time_opt} seconds")
				self.result["text_output"] += f" Optimization complete. Elapsed time: {elapsed_time_opt} seconds\n"
		else: # discretise the plan 
			start_time = time.perf_counter()
			plan = planner.discretize_path(wps, num_steps=int(num_pts/len(wps)))
			elapsed_time_opt = time.perf_counter() - start_time
			self.result["traj_opt"]["metrics"]["time"] = elapsed_time_opt 		
			
			printRed(f"Optimization complete. Elapsed time: {elapsed_time_opt} seconds")
			self.result["text_output"] += f" Optimization complete. Elapsed time: {elapsed_time_opt} seconds\n"
		
		return plan
					   
	def plot_plan(self, plan, WPS=False, Nodes=False, color:Color = Color.WHITE):
		# return
		num_lines_formed = 0
		if WPS:
			prev_pos = plan[0]
			for pos in plan:
				self.plot_point(pos, color = color, pointSize=4)
				self.plot_line(prev_pos, pos, color = color, lineWidth=8)
				num_lines_formed += 1
				prev_pos = pos
		elif Nodes:
			prev_pos = plan[0].pos
			for node in plan:
				# self.plot_point(node.pos, color = color, pointSize=2)
				self.plot_line(prev_pos, node.pos, color = color, lineWidth=8)
				num_lines_formed += 1
				prev_pos = node.pos
		else:
			for pos in plan:
				self.plot_point(pos, color = Color.RED, pointSize=5)
		return num_lines_formed
