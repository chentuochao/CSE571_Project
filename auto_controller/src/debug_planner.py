import numpy as np
from Planner import My_planner
from matplotlib import pyplot as plt
import copy

L = 0.3

def init_visualizer(map, traj, plan):
    """ Initialize visualizer
    """
    radius = L/0.05

    fig = plt.figure()
    ax1 = fig.add_subplot(1, 1, 1)

    # Plot img
    visit_map = 1 - np.copy(map) # black is obstacle, white is free space
    ax1_img = ax1.imshow(visit_map, interpolation="nearest", cmap="gray")

    ax = plt.gca()



    for i in range(0, traj.shape[0]):
        # Now plot a line for the direction of the car
        config = traj[i, :]

        circle1 = plt.Circle(config[:2][::-1], radius, fill=True, facecolor='w')
        circle2 = plt.Circle(config[:2][::-1], radius, fill=False, color='k')
        ax.add_artist(circle1)
        ax.add_artist(circle2)

        theta = config[2]
        ed = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]]) @ np.array([[radius*1.5, 0]]).T
        ed = ed[:,0]
        ax.plot([config[1], config[1]+ed[1]], [config[0], config[0]+ed[0]], 'b-', linewidth=3)

    for idx in range(plan.shape[0]-1):
        econfig = plan[idx, :, 0]
        sconfig = plan[idx+1, :, 0]
        x = [sconfig[0], econfig[0]]
        y = [sconfig[1], econfig[1]]
        ax.plot(y, x, 'r')



def result_visualizer(map,traj, plan, plan_noise):
    """ Initialize visualizer
    """
    radius = L/0.05

    fig = plt.figure()
    ax1 = fig.add_subplot(1, 1, 1)

    # Plot img
    visit_map = 1 - np.copy(map) # black is obstacle, white is free space
    ax1_img = ax1.imshow(visit_map, interpolation="nearest", cmap="gray")

    ax = plt.gca()

    for i in range(0, traj.shape[0]):
        # Now plot a line for the direction of the car
        config = traj[i, :]

        circle1 = plt.Circle(config[:2][::-1], radius, fill=True, facecolor='w')
        circle2 = plt.Circle(config[:2][::-1], radius, fill=False, color='k')
        ax.add_artist(circle1)
        ax.add_artist(circle2)

        theta = config[2]
        ed = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]]) @ np.array([[radius*1.5, 0]]).T
        ed = ed[:,0]
        ax.plot([config[1], config[1]+ed[1]], [config[0], config[0]+ed[0]], 'b-', linewidth=3)


    for idx in range(len(plan)-1):
        econfig = plan_noise[idx]
        sconfig = plan_noise[idx+1]
        x = [round(sconfig[0]/resolution), round(econfig[0]/resolution)]
        y = [round(sconfig[1]/resolution), round(econfig[1]/resolution)]

        econfig2 = plan[idx]
        sconfig2 = plan[idx+1]

        x2 = [round(sconfig2[0]/resolution), round(econfig2[0]/resolution)]
        y2 = [round(sconfig2[1]/resolution), round(econfig2[1]/resolution)]
        ax.plot(y, x, 'r')
        ax.scatter(y2, x2, c='g')
        ax.text(y[1], x[1], str(idx + 1))


def car_forward(ip, t_span, linear_vel, delta):
    T = 20 
    dt = t_span/T
    #print(linear_vel, delta)
    #print(ip)
    for t in range(1, T):
        ip[0] = ip[0] + linear_vel * np.cos(ip[2]) * dt
        ip[1]  = ip[1]  + linear_vel * np.sin(ip[2]) * dt
        ip[2]  = ip[2]  + (linear_vel/L) * np.tan(delta) * dt
        ip[2] = ip[2] % (2*np.pi)
    #print(ip)
    return ip



map = np.load('../s_150_150_g300_450_allen1/allen1_safeguard_1.npy')
map = map[1850:2350,1900:2500]
plan = np.load('../s_150_150_g300_450_allen1/plan2.npy')

'''
pos_list = np.load('../s_150_150_g300_450_allen1/desample_plan.npy')
angle_list = np.load('../s_150_150_g300_450_allen1/desample_theta.npy')
print(pos_list.shape)
print(angle_list.shape)
'''

traj =np.load('../s_150_150_g300_450_allen1/desample_traj.npy')

#init_visualizer(map, traj, plan)
#plt.show()


motion_params = {}
motion_params["L"] = 0.3
motion_params["max_delta"] = 0.34

resolution = 0.05
t_span = 0.5


traj2 = copy.deepcopy(traj)
traj2[:, :2] = traj2[:, :2] *resolution
planner = My_planner(map, motion_params, traj2, t_span, resolution)



start_point = np.array([150*resolution, 150*resolution, 0])
ip = start_point
history = []
history_noise = []
np.random.seed(4)

while 1:
    ctrl = planner.motion_planning(ip)
    if ctrl is None:
        break
    ip = car_forward(ip, t_span, ctrl[0], ctrl[1])
    save_ip = copy.deepcopy(ip)

    ip[0] = ip[0] + np.random.normal(0, 0.05)
    ip[1] = ip[1] + np.random.normal(0, 0.05)
    ip[2] = ip[2] + np.random.normal(0, 0.1)

    save_ip2 = copy.deepcopy(ip)

    history.append(save_ip)
    history_noise.append(save_ip2)

print(len(history))
result_visualizer(map, traj, history, history_noise)
plt.show()