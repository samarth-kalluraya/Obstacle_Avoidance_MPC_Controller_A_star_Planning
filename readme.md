# Obstacle avoidance simulation (A* + potential fields + MPC)
![](https://github.com/samarth-kalluraya/Obstacle_Avoidance_MPC_Controller_A_star_Planning/blob/master/mpc_sim.gif)

# To Run the Main Program File with A* Planning
Open MPC_MAIN_navigation.py and run.
	
### Tunable options for user:
1. Set save_simulation=True if you want to save simulation
2. set feasible start point and goal point (default (10,2) and (20,47))	
3. Horizon length (default=6) [Describe a little more]
4. desired_speed (default=5)
5. cost function weights W1, W2, W3 
6. default: W1 = np.array([0.01, 0.01])  # input weightage
7. W2 = np.array([5.0, 5.0, 0.5, 5])  # state error weightage
8. W3 = np.array([0.01, 1.0])  # rate of input change weightage
9. To add more obstacles in path use lines 486 to 493 as an example and repeat the procedure.



# To Test Navigation through Moving Obstacles using Potential Fields and MPC 
Run mpc_test.py set run_code=1

### Tunable  options for user:
1. Set save_simulation=True if you want to save simulation
2. Set start point and goal point (default (0,-5) and (50,30))	
3. orizon length (default=5)
4. desired_speed (default=5)
5. cost function weights W1, W2, W3 
6. default: W1 = np.array([0.01, 0.01])  # input weightage
7. W2 = np.array([2.0, 2.0, 0.5, 0.5])  # state error weightage
8. W3 = np.array([0.01, 0.1])  # rate of input change weightage
	*NUM_OF_OBSTACLES (default = 12)

# To Test only Path Following Using MPC
Run mpc_test.py set run_code=2
	
### Tunable  options for user:
1. Set save_simulation=True if you want to save simulation
2. set start point (default (0,-5))	
3. Horizon length (default=5)
4. desired_speed (default=5)
5. cost function weights W1, W2, W3 
6. default: 
    * W1 = np.array([0.01, 0.01])  # input weightage
    *  W2 = np.array([2.0, 2.0, 0.5, 0.5])  # state error weightage
	*  W3 = np.array([0.01, 0.1])  # rate of input change weightage

#### To change path uncomment any one of the paths given on lines 521 to 523		
	521. path_x,path_y,path_yaw = get_right_turn(dist_step)
	522. path_x,path_y,path_yaw = get_forward_course(dist_step)
	523. path_x,path_y,path_yaw = get_straight_course(dist_step)

 
# To Test only Potential Field 
set run_code=3
	
### Tunable  options for user:

1. Set save_simulation=True if you want to save simulation
2. set start point and goal point (default (0,-5) and (50,30))	
3. NUM_OF_OBSTACLES (default = 12)
