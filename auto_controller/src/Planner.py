from audioop import avg
import numpy as np
import rospy
import utils

def angular_difference(start_config, end_config):         
    th1 = start_config[2]; th2 = end_config[2]
    ang_diff = th1-th2
    ang_diff = ((ang_diff + np.pi) % (2*np.pi)) - np.pi 
    ang_diff = ang_diff*(180.0/np.pi) # convert to degrees
    return ang_diff

def compute_distance(start_config, end_config):  
    scale1 = 1#50*0.05
    scale2= 1

    ang_diff = np.abs(angular_difference(start_config, end_config))/180.0
    e_to_s = start_config[:2] - end_config[:2] # Shape: [2 x n]
    euclidean_distance = np.linalg.norm(e_to_s) # Shape: [n]
    e_to_s = e_to_s / float(euclidean_distance)
    e_vec = np.array([np.cos(end_config[2]), np.sin(end_config[2])])
    alignment = 1 - np.abs(e_vec.dot(e_to_s)) # Shape: [n]
    #alignment = 1 - e_vec.dot(e_to_s)
    # alignment is in [0,1], euclidean_distance can be large, ang_diff is between [0,1]

    #print(euclidean_distance)
    return scale1*alignment + euclidean_distance + scale2*ang_diff, alignment, euclidean_distance, ang_diff



class My_planner:
    def __init__(self, map_msg, map_info, motion_params, goal_list, control_span):
        #self.map = np.array(map_msg.data).reshape(
        #    (map_msg.info.height, map_msg.info.width)
        #)
        self.map = map_msg
        self.step = 0
        self.map_info = map_info
        self.map_resolution = map_info.resolution #map_msg.info.resolution
        self.motion_params = motion_params
        self.goal_list = goal_list
        self.current_goal_index = 0
        self.t_span = control_span + 3
        self.t_collision = control_span + 1

        self.old_delta = 0
        self.collision_scale = 1000
        self.smooth_scale = 0
        self.dis_scale = 12
        self.V_limit = 0.4

        self.T = int(round(self.t_span/(0.04/self.V_limit)))
        self.T_collision = int(round(self.t_collision/(0.04/self.V_limit )))
        print('T', self.T, 'T_collision: ',  self.T_collision)
        self.K = 20

    def check_goal_list(self, state):
        if self.current_goal_index == len(self.goal_list) - 1:
            return
        current_goal = self.goal_list[self.current_goal_index]
        next_goal = self.goal_list[self.current_goal_index + 1]

        dis_now = np.linalg.norm(current_goal[:2] - state[:2])
        dis_next = np.linalg.norm(next_goal[:2] - state[:2])
        dis_goal = np.linalg.norm(next_goal[:2] - current_goal[:2])

        if dis_now < 0.4 or dis_next < dis_goal:
            self.current_goal_index = self.current_goal_index + 1
            rospy.logdebug("update to goal"+ str(self.current_goal_index) +"at step: "+str(self.step ))
        return

    def check_dis_from_wall(self, state):
        pixel_x = round(state[0]/self.map_resolution)
        pixel_y = round(state[1]/self.map_resolution)
        
        r_to_check = [1, 2, 3, 4, 5, 6]
        #for r in range(1, 10):
        closest_wall = -1

        for r in r_to_check:
            area = self.map[pixel_x-r:pixel_x+r+1, pixel_y-r:pixel_y+r+1]
            if np.sum(area) > 0:
                closest_wall = r
                break
        if closest_wall == -1: return 0
        else:  return 1.0/(closest_wall + 1)


    def check_dis_from_wall2(self, state):
        #real_x = state[0]/self.map_resolution
        #real_y = state[1]/self.map_resolution
          
        new_state = utils.world_to_map_single(state, self.map_info)
        new_state = utils.convert_to_pltaxis(new_state)
        
        
        real_x = new_state[0]
        real_y = new_state[1]
        pixel_x = int(round(new_state[0]/self.map_resolution))
        pixel_y = int(round(new_state[1]/self.map_resolution))

        r_to_check = 10
        #for r in range(1, 10):
        closest_wall = -1

        area = self.map[pixel_x-r_to_check:pixel_x+r_to_check+1, pixel_y-r_to_check:pixel_y+r_to_check+1]

        if np.sum(area) > 0:
            min_dis = 100000
            for i in range(pixel_x-r_to_check, pixel_x+r_to_check+1):
                for j in range(pixel_y-r_to_check , pixel_y+r_to_check+1):
                    if(self.map[i, j] == 1):
                        r = np.linalg.norm( [real_x - i, real_y - j])
                        if r < min_dis:
                            min_dis = r
            return 1.0/min_dis
        else:
            return 0


    def goal_criterion(self, config, goal_config):
        if np.linalg.norm(config[:2] - goal_config[:2]) < 1: #and np.abs(self.angular_difference(config, goal_config)) < 5:
            rospy.logdebug('Goal reached!')
            #print(f'Goal reached! State: {config}, Goal state: {goal_config}')
            #print(f'xy_diff: {np.linalg.norm(config[:2,:] - goal_config[:2,:]):.03f}, '\
            #        f'ang_diff: {np.abs(self.angular_difference(config, goal_config))[0]:.03f}')
            return True
        else:
            return False

    def motion_simulation(self, rollouts, t_span, linear_vel, deltas):
        K, T, _ = rollouts.shape
        dt = float(t_span)/T

        L = self.motion_params["L"]
        for t in range(1, T):
            rollouts[:,t, 0] = rollouts[:,t-1, 0] + linear_vel * np.cos(rollouts[:,t-1, 2]) * dt
            rollouts[:,t, 1] = rollouts[:,t-1, 1] + linear_vel * np.sin(rollouts[:,t-1, 2]) * dt
            rollouts[:,t, 2] = rollouts[:,t-1, 2] + (linear_vel/L) * np.tan(deltas) * dt
            rollouts[:,t, 2] = rollouts[:,t, 2] % (2*np.pi)
        #print(rollouts[:, T-1, :2]*20)
        #raise KeyboardInterrupt
        return rollouts

    def check_collision(self, path_point):
        collision_times = 0
        #print(path_point.shape)
        for t in range(0, path_point.shape[0]):
            new_state = utils.world_to_map_single(path_point[t, :2], self.map_info)
            new_state = utils.convert_to_pltaxis(new_state)
            pixel_x = int(round(new_state[0]))
            pixel_y = int(round(new_state[1]))

            if(self.map[pixel_x, pixel_y] == 1):
                collision_times += 1
        return collision_times

    def check_lidar(self, delta, observe):
        ob_ranges, ob_angles = observe
        ob_ranges = ob_ranges[:-1]
        ob_angles = ob_angles[:-1]
        tolerance = 0.44
        dises = []
        L = self.motion_params["L"]
        #delta_new = delta + (self.V_limit/L) * np.tan(delta) * (self.t_collision/3.0)
        delta_new = delta
        #print(delta, delta_new)
        for i in range(0, ob_angles.shape[0]):
            if abs(ob_angles[i] - delta_new) < tolerance:
                dises.append(ob_ranges[i])
        
        if(len(dises) == 0):
            return 1000
            
        dises = np.array(dises)
        short_num = np.sum(dises < 0.7)
        #print(delta, short_num, dises)
        
        scale = 5
        avg_dis = np.mean(dises)
        #print(dises)
        if avg_dis > 4:
            return 0 + short_num*scale
        elif avg_dis <= 0.3:
            return 1000 + short_num*scale
        else:
            return 1.0/avg_dis + short_num*scale
            
        return 
        #print(delta, dises)
        #print(ob_angles)


    def check_lidar2(self, delta, observe):
        ob_ranges, ob_angles = observe
        ob_ranges = ob_ranges[:-1]
        ob_angles = ob_angles[:-1]
        tolerance = 0.12
        dises = []
        L = self.motion_params["L"]
        #delta_new = delta + (self.V_limit/L) * np.tan(delta) * (self.t_collision/3.0)
        delta_new = delta
        #print(delta, delta_new)
        for i in range(0, ob_angles.shape[0]):
            if abs(ob_angles[i] - delta_new) < tolerance:
                dises.append(ob_ranges[i])

        #print(delta, dises)
        #print(ob_angles)
        if(len(dises) == 0):
            return 1000
        avg_dis = np.mean(dises)
        #print(dises)
        if avg_dis > 3:
            return 0
        elif avg_dis <= 0.2:
            return 1000
        else:
            return 1.0/avg_dis


    def motion_cost(self, path_point, goal, delta):
        # cost of end point to goal 
        
        cost_to_goal, alignment, euclidean_distance, ang_diff = compute_distance(path_point[-1, :], goal)
        cost_to_obstacle = self.check_collision(path_point)
        cost_for_smooth  = (np.abs(delta - self.old_delta)/np.pi)
        cost_for_wall_dis = self.check_dis_from_wall2(path_point[-1, :])
        '''
        if self.step == 31:
            print("Step",self.step, "ANGLE", delta)
            print(np.round(path_point[0, :2]/self.map_resolution), path_point[0, 2]/np.pi*180)
            print(np.round(path_point[-1, :2]/self.map_resolution), path_point[-1, 2]/np.pi*180)
            #print(alignment, euclidean_distance, ang_diff)
            print(cost_to_goal, self.dis_scale*cost_for_wall_dis , self.collision_scale*cost_to_obstacle)
        '''
        return cost_to_goal + self.dis_scale*cost_for_wall_dis + self.collision_scale*cost_to_obstacle + self.smooth_scale*cost_for_smooth, cost_to_obstacle


    def motion_cost_with_lidar(self, path_point, goal, delta, observe):
        # cost of end point to goal 
        # print("Pos", delta, path_point[-1, :])
        cost_to_goal, alignment, euclidean_distance, ang_diff = compute_distance(path_point[-1, :], goal)
        cost_to_obstacle = self.check_collision(path_point[:self.T_collision, :])
        cost_to_lidar = self.check_lidar(delta, observe)
        '''
        if self.step == 31:
            print("Step",self.step, "ANGLE", delta)
            print(np.round(path_point[0, :2]/self.map_resolution), path_point[0, 2]/np.pi*180)
            print(np.round(path_point[-1, :2]/self.map_resolution), path_point[-1, 2]/np.pi*180)
            #print(alignment, euclidean_distance, ang_diff)
            print(cost_to_goal, self.dis_scale*cost_for_wall_dis , self.collision_scale*cost_to_obstacle)
        '''
        #print(delta, cost_to_goal, cost_to_obstacle , cost_to_lidar)

        return cost_to_goal + self.collision_scale*cost_to_obstacle + self.dis_scale*cost_to_lidar, cost_to_obstacle


    def motion_planning(self, state, lidar_ob):
        T = self.T 
        K = self.K

        if self.goal_criterion(state, self.goal_list[-1, :]):
            print('Goal reached!!!!!!')
            return None, None

        self.check_goal_list(state)
        self.step += 1

        max_delta = self.motion_params["max_delta"]
        deltas = np.linspace(-max_delta, max_delta, K)


        # emergency 
        ob_ranges, ob_angles = lidar_ob
        ob_ranges = ob_ranges[:-3]
        ob_angles = ob_angles[:-3]
        range_angle = 0.1
        for i in range(0, ob_angles.shape[0]):
            if abs(ob_angles[i] - 0) < range_angle and ob_ranges[i] < 0.3:
                print("Emergency Stop!!!!!!")
                return [0, 0], None


        rollouts = np.zeros((K, T, 3))

        for k in range(0, K):
            rollouts[k, 0, :] = state

        rollouts = self.motion_simulation(rollouts, self.t_span, self.V_limit, deltas)

        min_cost = 100000
        min_k = -1
        cost_list = []
        for k in range(0, K):
            cost, collision_time = self.motion_cost_with_lidar(rollouts[k,...], self.goal_list[self.current_goal_index], deltas[k], lidar_ob)
            cost_list.append(cost)
            #print(cost, collision_time)
            if cost < min_cost and collision_time < 1:
                min_cost = cost
                min_k = k

        if min_k == -1:
            rospy.logdebug("ALL path is collided")
            return None, None
        else:
            #print(cost_list[::2])
            #print(self.step, min_cost, deltas[min_k])
            return [self.V_limit, deltas[min_k]], rollouts[min_k,...]


