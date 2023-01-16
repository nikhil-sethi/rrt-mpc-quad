# MPC
# Introduction
This branch includes an implementation of MPC. This implementation however is still in development and is therefore not pushed to the main branch.
It does include all the required features, but it is not able to fly properly due to some issues. One issue is that the physics model from pybullet 
and the incorporated dynamics model in the mpc seem to use a different order of the rotors. But also a second model which was used to try and resemble the dynamics model 
in the gym pybullet drones environment better, did not make the drone fly properly in the simulation environment.
There have been successful test runs outside the simulation environment, but that does not use any form of feedback loop.
Also this does not work perfectly and leads to some bad exitflags from the solver, meaning that the way the problem is in general is also not yet fully up to the standards that it should be for the MPC to work.

## How to set it up?
The solver that is used is FORCES Pro. This is a licensed solver. If you have a license for FORCES Pro, you can use and create your own solvers. To do this, you need to make sure that 
you are using a python-3.9 or before, since FORCES Pro is not yet compatible with 3.10 and 3.11. Also to run it, you need to make sure that you installed your FORCES-Pro client and add that to your pythonpath, like this:
```commandline
export PYTHONPATH=/path/to/forces_pro_client:/$PYTHONPATH
```
For more information, like the required dependencies, you can check out the [manual](https://forces.embotech.com/Documentation/) provided for FORCES Pro.
## How to run?
To run the MPC in the simulation environment, you have to run the following command:
```commandline
python3 main.py --use_mpc True
```
To run the MPC without the simulation environment, so with no feedback (Here it will just follow a trajectory in the shape of a circle)
```commandline
python3 test_mpc.py
```

## The code
The code for the MPC is split up into four. First there is `solver.py`. This is where the model is defined and based on that, the solver is created. Also in the function `compute_action`, this created solver is used
to compute the next states and the speeds of the rotors that need to be fed to the simulation environment. Then there is `dynamics.py`. Here are all the equality constraints defined that are used in the solver. This file includes two different implementations
of the dynamics that have been tested with. Third of all, there is `objectives.py`. This includes the objectives that the solver optimizes for. This includes a normal cost and a terminal cost.
These cost functions now only serve the purpose of tracking a trajectory and keeping a cost the rotor speeds. And finally, there is `util.py`. This includes some functions to extract the next path points which are needed to set a trajectory, which can be fed to the objectives.