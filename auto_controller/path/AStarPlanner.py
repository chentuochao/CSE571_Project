import numpy as np

class AStarPlanner(object):    
    def __init__(self, planning_env, epsilon):
        self.env = planning_env
        self.nodes = {}
        self.epsilon = epsilon
        self.visited = np.zeros(self.env.map.shape)
        
            
    def Plan(self, start_config, goal_config):
        # TODO: YOUR IMPLEMENTATION HERE

        plan = []
        plan.append(start_config)
        plan.append(goal_config)

        state_count = 0
        cost = 0
        #self.env.goal = goal_config #since we use function: self.env.h(new_config). Already wrapped in run.py
        
        action_dict = {'up':np.array([[0],[1]]), 'down':np.array([[0],[-1]]), 'left':np.array([[-1],[0]]),'right':
                           np.array([[1],[0]]), 'up-left': np.array([[-1],[1]]), 'up-right':np.array([[1],[1]]),
                           'down-left': np.array([[-1],[-1]]), 'down-right':np.array([[1],[-1]])}
        
        #obstacle = np.argwhere(self.env.map == 1)
        g=0
        #print('eps',self.epsilon)
        #print('env.eps',self.env.epsilon)
        self.env.epsilon = self.epsilon
        h=self.env.h(start_config)
        cost=g+h
        openlist = [(cost, start_config)]
        closelist = []
 
        self.nodes[str(start_config.ravel())]= (cost, g, h, start_config)

        while len(openlist) >0 and str(goal_config.ravel()) not in closelist:
            _, current_config = openlist.pop(0)
            
            closelist.append(str(current_config.ravel()))
#             print('clist',closelist)

            if str(goal_config.ravel()) in closelist:
                break


            for action in action_dict:
                new_config = current_config+action_dict[action]
                
                if self.env.state_validity_checker(new_config):
                    h=self.env.h(new_config)
                    g = self.env.compute_distance(current_config, new_config) + self.nodes[str(current_config.ravel())][1]
                    cost = g+h
                    if self.visited[int(new_config[0]), int(new_config[1])]:
                        continue
                    # set the nodes to visited
                    self.visited[int(new_config[0]), int(new_config[1])] = 1
      
                    pair = (cost, new_config)
                    openlist.append(pair)
                    self.nodes[str(new_config.ravel())] =(cost, g, h, current_config)
                    openlist = sorted(openlist, key=lambda pair: pair[0])
#             print('olist',openlist)

#             if self.env.goal_criterion(current_config, goal_config):
#                 break

        child_config = goal_config
#         child_config = current_config
        
        while np.any(self.nodes[str(child_config.ravel())][3]!=start_config):
#             plan.insert(1, child_node)
            parent_node=self.nodes[str(child_config.ravel())][3]
            child_config = parent_node
            plan.insert(1, parent_node)
        
        #print('plan', plan)
        np.save('plan.npy', plan)
        state_count = len(closelist)
        cost = self.nodes[str(goal_config.ravel())][1]
#         cost = self.nodes[str(current_config.ravel())][1] + self.env.compute_distance(current_config, goal_config)
        print("States Expanded: %d" % state_count)
        print("Cost: %f" % cost)

        return np.concatenate(plan, axis=1)

