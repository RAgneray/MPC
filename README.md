# MPC Simulation (current version 2.3)
# Files used for the simulation of a Model Predictive Control and developped on Matlab environnement.
# To lauch a simulation, execute MAIN.m
# You can change the simulation parameters in PARAMETERS.m
# Now fully able to simulate 'R' plane robot (one_link), 'RR' plane robot (two_link) and PUMA 560, and to work with CVX solver and OSQP solver.
# The Robotics Toolbox developped by Peter Corke, CVX-academic license (http://cvxr.com/cvx/academic/) and OSQP (http://osqp.readthedocs.io/en/latest/index.html) are needed to make the scripts work. CVX has to installed and the 'rvctools' folder and 'MPC' folder has to be in the same folder. OSQP has to install every time Matlab is launched, and the 'osqp' folder and 'MPC' folder has to be in the same folder.
# To know about the trajectory generation, please check : Konstantinos J. Kyriakopoulos and George N. Saridis. Minimum jerk path generation. In Robotics and Automation, 1988 IEEE International Conference on , volume 1, pages 364-369, 1988.
# Variables have been named following the files where they are set up (para from PARAMETERS.m, main from MAIN.m, mpc from MPC.m, cons from CONSTRAINTS.m, plot from PLOTANIM.m, save from SAVEDATA.m, task from TASK.m, traj from TRAJECTORY.m and upda from UPDATE.m). If thess naming components are in caps, variables are set to be global.
