""" File to debug and plot maps etc."""
import sys
import os
import pybullet as p

import time
# to include subdirs as modules
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.dirname(SCRIPT_DIR))

from maps import Map

map = Map(map_number=1)
map.view_map()

while True:
    p.stepSimulation()
    time.sleep(1)