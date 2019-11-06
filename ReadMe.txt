MPC_MAIN_navigation.py 
To run the main program file with A* planning open MPC_MAIN_navigation.py and run.
	Tunable options for user:
	*Set save_simulation=True if you want to save simulation
	*set feasible start point and goal point (default (10,2) and (20,47))	
	*Horizon length (default=6)
	*desired_speed (default=5)
	*cost function weights W1, W2, W3 
	*default: W1 = np.array([0.01, 0.01])  # input weightage
		  W2 = np.array([5.0, 5.0, 0.5, 5])  # state error weightage
		  W3 = np.array([0.01, 1.0])  # rate of input change weightage
	*To add more obstacles in path use lines 486 to 493 as an example and repeat the procedure.


mpc_test.py
To test navigation through moving obstacles using potential fields and MPC set run_code=1 	 
	Tunable  options for user:
	*Set save_simulation=True if you want to save simulation
	*set start point and goal point (default (0,-5) and (50,30))	
	*Horizon length (default=5)
	*desired_speed (default=5)
	*cost function weights W1, W2, W3 
	*default: W1 = np.array([0.01, 0.01])  # input weightage
		  W2 = np.array([2.0, 2.0, 0.5, 0.5])  # state error weightage
		  W3 = np.array([0.01, 0.1])  # rate of input change weightage
	*NUM_OF_OBSTACLES (default = 12)

To test only path following using MPC set run_code=2
	Tunable  options for user:
	*Set save_simulation=True if you want to save simulation
	*set start point (default (0,-5))	
	*Horizon length (default=5)
	*desired_speed (default=5)
	*cost function weights W1, W2, W3 
	*default: W1 = np.array([0.01, 0.01])  # input weightage
		  W2 = np.array([2.0, 2.0, 0.5, 0.5])  # state error weightage
		  W3 = np.array([0.01, 0.1])  # rate of input change weightage
	*To change path uncomment any one of the paths given on lines 521 to 523		
		521.. #path_x,path_y,path_yaw = get_right_turn(dist_step)
    		522.. path_x,path_y,path_yaw = get_forward_course(dist_step)
    		523.. #path_x,path_y,path_yaw = get_straight_course(dist_step)

 
To test only potential field set run_code=3
	Tunable  options for user:
	*Set save_simulation=True if you want to save simulation
	*set start point and goal point (default (0,-5) and (50,30))	
	*NUM_OF_OBSTACLES (default = 12)

