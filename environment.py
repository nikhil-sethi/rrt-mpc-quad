import numpy as np
import pybullet as p

from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.utils.enums import DroneModel, Physics

from planner.sample_based import RRT
from planner.sample_based import RRT_Star
from planner.spaces import Space
from planner.graph import Node
from planner.trajectory import MinVelAccJerkSnapCrackPop

from maps import load_map, MAPS
from utils import Color, discretize_path

class Env(CtrlAviary):
	"""Multi-drone environment class for control applications."""

	################################################################################

	def __init__(self,
				 start, 
				 goal,
				 map = 1,
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
		self.start = start
		self.goal = goal
		# print(self.start, self.goal)
		print(map)
		self.map = MAPS[map]
		super().__init__(drone_model=drone_model,
						 num_drones=num_drones,
						 neighbourhood_radius=neighbourhood_radius,
						 initial_xyzs=np.array([start]),
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
	
	@staticmethod
	def plot_point(position, color:Color = Color.BLUE):
		"""Used to plot trajectories"""
		p.addUserDebugPoints(pointPositions = [position], pointColorsRGB = [color.value[:-1]])

	@staticmethod
	def plot_line(from_pos, to_pos, color:Color = Color.BLUE):
		"""Used to plot trajectories"""
		p.addUserDebugLine(lineFromXYZ = from_pos, lineToXYZ = to_pos, lineColorRGB = [color.value[:-1]])

	def _addObstacles(self):
		"""Add obstacles to the environment.
		"""
		# Dilate obstacles to drone radius plus margin also equal to drone radius 
		load_map(self.map, self.CLIENT, dilate=True, dilation=2*self.L)
		
		## uncomment below snippet to plot extents for obstacles
		# for obs in self.map:
		# 	for i in range(2): 
		# 		self.plot_point(obs.extent[i])
		# 	self.plot_line(obs.extent[0], obs.extent[1])

	def plan(self, method="rrt_star", min_snap=False, d=100):	
		start = Node(pos = self.start)
		goal = Node(pos = self.goal)
		ws = Space(low=[-1, -1, 0], high=[1, 1, 1])
		if method == 'rrt':
			planner = RRT(space=ws, start=start, goal=goal, map=self.map)
		elif method == 'rrt_star':
			planner = RRT_Star(space=ws, start=start, goal=goal, map=self.map)
		else:
			raise NotImplementedError()
		wps = planner.run()

		plan_dist = planner.fastest_route_to_end # total distance covered by waypoints

		num_pts = int(plan_dist//0.01) # trajectory seems to work best with current control system when discretisation is around 1 cm. Weird but ok.

		# some preprocessing on waypoints
		# convert to numpy array
		wps = planner.nodes_to_array(wps)

		# making sure waypoints are unique. RRT can be glitch sometimes
		wps, idx = np.unique(wps, axis=0, return_index=True)
		wps = wps[idx,:]	# np unique returns sorted values for some reason. undo that shit

		# path optimization 
		if min_snap:
			try:
				traj_opt = MinVelAccJerkSnapCrackPop(order=2, waypoints = wps.T, time=8)	# don't worry about time argument too much. it's all relative
				plan = traj_opt.optimize(num_pts=num_pts)
			except: # because min snap fails sometimes because of rank errors. still to debug
				# in that case, just go ahead with original waypoints and discretisation
				plan = discretize_path(wps, num_steps=int(num_pts/len(wps)))		
				# traj_opt.plot(plan)
		else: # discretise the plan 
			plan = discretize_path(wps, num_steps=int(num_pts/len(wps)))	
		return plan
					   
	def plot_plan(self, plan):
		prev_pos = plan[0]
		for pos in plan:
			self.plot_point(pos)
			# try:
			self.plot_line(prev_pos, pos)
			prev_pos = pos