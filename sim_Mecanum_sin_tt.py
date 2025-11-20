import numpy as np
from matplotlib import pyplot as plt
from math import cos,sin,pi,sqrt,atan2
import math
import json
import os

from Model.Mecanum import Mecanum
from Plotter.Error_plot import Error_plot
from Plotter.Mecanum_trajectory import Mecanum_trajectory
from Plotter.Mecanum_animation import Mecanum_animation

def Saturation(x,bound):
    x = np.array(x)
    x_m = np.linalg.norm(x)
    if x_m > bound:
        return x/x_m*bound
    else: 
        return x
    
def tanh(x):
    return [math.tanh(y) for y in x]

def Sgn(x):
    x = np.array(x)
    y = []
    for number in x:
        if number > 0:
            y.append(1)
        elif number < 0:
            y.append(-1)
        else:
            y.append(0)
    return np.array(y)






def run_motion_simulation(sim_type,sim_name,sim_time):
    # agent basic parameter
    num_agent = 6
    dim_state = 3
    dim_control = 3

    # sim time para
    total_seconds = sim_time
    time_step = 1e-4
    total_step = int(total_seconds/time_step)
    frequency = int(1/time_step)

    # create agent class and data_recorder 
    agent=[ Mecanum(dt=time_step) for i in range(num_agent) ]

    # 3d data list: trajectory_data [agent][state_dimension][value along time]
    trajectory_data = [ [ [  ] for i in range(dim_state) ] for i in range(num_agent) ]
    
    # sensor graph definition    
    adjencency_matrix = [[0,0,0,0,0,0],
                         [1,0,1,1,1,1],
                         [1,1,0,0,0,1],
                         [1,1,0,0,1,0],
                         [0,1,0,1,0,1],
                         [0,1,1,0,1,0],  ] # a_ij , i->j means i keep the distance to j

    # formation shape and rigidity matrix (five ploygon)
    bian = 5
    bianxin = 5*0.5/cos(54*pi/180)
    distance_matrix = [ [0,0,0,0,0,0],
                        [bianxin,0,bianxin,bianxin,bianxin,bianxin],
                        [bian,bianxin,0,0,0,bian],
                        [bian,bianxin,0,0,bian,0],
                        [0,bianxin,0,bian,0,bian],
                        [0,bianxin,bian,0,bian,0],        ] 

    # autmatically identify the edges in the graph
    edge_list = []
    distance_error = []
    orientation_error = []
    tracking_err = []
    target_trajectory = [ [],[] ]


    for i in range(num_agent):
        for j in range(i):
            a = adjencency_matrix[i][j]
            a2 = adjencency_matrix[j][i]
            if a>0 or a2>0: # exist directed edge
                edge_list.append([min(i,j),max(i,j)])
                distance_error.append([])

    # reset pos and save data
    agent[0].reset([0,0,0])
    agent[1].reset([-6,6,0])
    agent[2].reset([-4,-5,0])
    agent[3].reset([-2,12,0])
    agent[4].reset([-12,12,0])
    agent[5].reset([-14,-2,0])

    # save initial state
    for i in range(num_agent):
        for j in range(dim_state):
            trajectory_data[i][j].append(agent[i].state[j]) # i=agent j=dim

    # save initial error
    for i in range(len(edge_list)):
        a,b = edge_list[i]
        x1 = agent[a].state[0]
        y1 = agent[a].state[1]
        x2 = agent[b].state[0]
        y2 = agent[b].state[1]
        d = sqrt( (x1-x2)**2+(y1-y2)**2 )
        err = d-max(distance_matrix[a][b],distance_matrix[b][a])
        distance_error[i].append(err)

    # orientation error recording
    def angle_transform(a):
        if a<-pi:
            return angle_transform(a+2*pi)
        elif a>=pi:
            return angle_transform(a-2*pi)
        else:
            return a

    theta = 0
    p0 = np.array(agent[0].state[:2])
    p1 = np.array(agent[1].state[:2])
    po=p0-p1 # relative position vector of orientation
    po_d = [bianxin*cos(theta),bianxin*sin(theta)]
    po_bar = po-po_d # error of p10
    orientation_error.append(np.linalg.norm(po_bar))

    print('-'*30)
    print('init done.')
    print('-'*30)

    

    # run motion simulation

    for step in range(total_step):
        # reset the r vector, r_i = sum_j(p_ij*sigma_ij)
        r = [ [0 for i in range(2)] for j in range(num_agent)]

        # current time /second
        time = step*time_step
        
        # agent i control rigid term
        for i in range(num_agent):
            # each neighbor contribution to r vector
            for j in range(num_agent):
                # if j is not a neighbor then continue    
                if i==j:
                    continue
                elif adjencency_matrix[i][j]==0:
                    continue
                else:
                    pass

                # r calculate
                p_i = np.array(agent[i].state[:2])
                p_j = np.array(agent[j].state[:2])
                d = distance_matrix[i][j] # desired distance
                p_ij= p_i-p_j 
                p_ij_m = np.linalg.norm(p_ij) # L2 norm
                sigma_ij = p_ij_m**2-d**2 # square distance error
                r_j = p_ij*sigma_ij

                r[i] = np.add(r[i],r_j)

        # target speed definition
        t = time

        # [ target trajectory design ] ------------------------------------------------
        pt = [5*t+5, -3*cos(t)]
        dpt = [5,3*sin(t)]

        dtheta = 1/((3/5*sin(t))**2+1)*3/5*cos(t)
        theta = atan2(dpt[1],dpt[0])
        # ---------------------------------------------------------------
        
        p0 = np.array(agent[0].state[:2])
        p1 = np.array(agent[1].state[:2])

        po=p0-p1 # relative position vector of orientation
        po_d = [bianxin*cos(theta),bianxin*sin(theta)]
        po_d_dot = [-bianxin*sin(theta)*dtheta,bianxin*cos(theta)*dtheta]
        po_bar = po-po_d # error of p10
        
        # [ control gain ] ------------------------------------------------
        k = 5 # gain for basic rigid term 
        beta = 15 #  gain for signal term
        alpha = 40 # gain for orientation

        lamda = 5 # tracking gain 1
        nu = 10 # tracking gain 2
        
        v_max_leader = 18
        v_max_follower = 18
        # -------------------------------------------------------------

        for i in range(num_agent):
            # u[i] = saturation_vector(u[i],50)
            kw=5
            if i==0:
                p_l = np.array(agent[i].state[:2])
                e_p = p_l - np.array(pt)
                # print(e_p,np.array(tanh_vector(nu*e_p))*(-lamda))
                u_L = np.array(tanh(nu*e_p))*(-lamda) + dpt
                u_L = Saturation(u_L,v_max_leader)
                agent[i].step(u_L)
                omega=kw*angle_transform(atan2(u_L[1],u_L[0])-agent[i].state[2])
                agent[i].step(np.append(u_L,omega))
            elif i==1:
                eta = alpha*np.dot(po_bar,po_d_dot)/(np.linalg.norm(r[i]-alpha*po_bar)**2)
                u1 = -(k-eta)*(r[i]-alpha*po_bar) + u_L
                u1 = Saturation(u1,v_max_leader)
                # u1 = tanh(u1,20)
                omega=kw*angle_transform(atan2(u1[1],u1[0])-agent[i].state[2])
                agent[i].step(np.append(u1,omega))
            else:
                u1 = -k*r[i] - beta*Sgn(r[i])
                u1 = Saturation(u1,v_max_follower)
                # u1 = tanh(u1,20)
                omega=kw*angle_transform(atan2(u1[1],u1[0])-agent[i].state[2])
                agent[i].step(np.append(u1,omega))

        # data recording by 50Hz
        interval = int(frequency/50)
        if step%(interval)==0:
            # state recording
            for i in range(num_agent):
                for j in range(dim_state):
                    trajectory_data[i][j].append(agent[i].state[j])

            # edge error recording
            for i in range(len(edge_list)):
                a,b = edge_list[i]
                x1 = agent[a].state[0]
                y1 = agent[a].state[1]
                x2 = agent[b].state[0]
                y2 = agent[b].state[1]
                d = sqrt( (x1-x2)**2+(y1-y2)**2 )
                err = d-max(distance_matrix[a][b],distance_matrix[b][a])
                distance_error[i].append(err)

            # orientation error recording
            orientation_error.append(np.linalg.norm(po_bar))

            # tracking error recording
            tracking_err.append(np.linalg.norm(e_p))

            # target trajectory
            target_trajectory[0].append(pt[0])
            target_trajectory[1].append(pt[1])
    
        # display process by 1Hz
        if step%frequency==0:
            print('Simulation Time: %.0f/%.0fs '%(time,total_seconds))

    print('-'*30)
    print('Motion Calculated.')
    print('-'*30)

    # create folder for figure
    folder_path = f"./data/{sim_type}/{sim_name}"
    os.makedirs(folder_path, exist_ok=True) 
    
    # save to json
    results = {
        "trajectory": np.array(trajectory_data).tolist(),
        "distance_error": np.array(distance_error).tolist(),
        "orientation_error": np.array(orientation_error).tolist(),
        "edge_list": np.array(edge_list).tolist(),
        "tracking_error": np.array(tracking_err).tolist(),
        "target_trajectory": np.array(target_trajectory).tolist()
    }
    
    with open(f'./data/{sim_type}/{sim_name}/motion_data.json', "w") as f:
        json.dump(results, f, indent=4)

    print('Motion data saved.')
    print('-'*30)






def plot_results(sim_type,sim_name,sim_time,
                 fig_type='.png',
                 if_trajectory_tracking = False,
                 if_animation = True):
    
    # read motion data
    with open(f'./data/{sim_type}/{sim_name}/motion_data.json', "r") as f:
        data = json.load(f)  

    # output fig name
    preffix = f'./data/{sim_type}/{sim_name}/'

    # define ploting parameter
    color_list = ['blue','green',
                  'darkgrey','darkgrey','darkgrey','darkgrey']

    label_list = ['Leader','Co-Leader','Followers','x','x','x']

    drawtime_list = [2.5,5,7.5]

    # ploting

    # 1 trajectory
    Mecanum_trajectory(data['trajectory'],
                    color_list,label_list,data['edge_list'],drawtime_list,
                    save_path=preffix+'tra'+fig_type,
                    # trajectory_width=1,point_size=0.8,edge_width=0.5,
                    target_trajectory=\
                        data['target_trajectory'] if if_trajectory_tracking else None,
                    xy_range = [-19,64,-9,24],
                    Show=False,
                    )

    # 2 distance error
    label_list = [ 'x' for a in range(len(data['distance_error'])-1) ]
    label_list.append(r'$\|p_{ij}\|-d_{ij}$')
    print(label_list)

    Error_plot(data['distance_error'],
                xy_label=['Time (s)','Distance error'],
                plt_label=label_list,
                save_path=preffix+'err_d'+fig_type,
                x_range=[0,sim_time]
                )

    # 3 normalized error 
    sigma_list = np.array([])
    distance_err = np.array(data['distance_error'])
    for i in range(len(distance_err[0])):
        err = distance_err[:,i]
        sigma_list = np.append(sigma_list, np.linalg.norm(err))
    sigma_list /= sigma_list[0]

    label = [r'$\frac{\|\sigma(t)\|}{\|\sigma(0)\|}$' ]
    print(label)
    Error_plot([sigma_list],
                xy_label=['Time (s)','Normalized error'],
                plt_label=label,
                save_path=preffix+'err_n'+fig_type,
                x_range=[0,sim_time],
                y_range=[-0.1,1.1],
                color_list=['blue'],
                )

    # 4 orientation error
    label = [r'$\left\| p_o-p^*_o(t) \right\|$']
    print(label)
    Error_plot([data['orientation_error']],
                xy_label=['Time (s)','Orientation error'],
                plt_label=label,
                save_path=preffix+'err_o'+fig_type,
                x_range=[0,sim_time],
                color_list=['blue']
                )
    
    # 5 tracking error
    if if_trajectory_tracking:
        label = [r'$\| p_1-p^*(t) \|$']
        print(label)
        Error_plot([data['tracking_error']],
                    xy_label=['Time (s)','Tracking error'],
                    plt_label=label,
                    save_path=preffix+'err_t'+fig_type,
                    x_range=[0,sim_time],
                    color_list=['blue'],
                    )

    # 6 Animation
    if if_animation:
        Mecanum_animation(data['trajectory'],color_list,data['edge_list'],
                        save_path=preffix+'ani'+'.gif',
                        # trajectory_width=0.6,point_size=0.6,
                        target_trajectory=\
                            data['target_trajectory'] if if_trajectory_tracking else None,
                        Show=False,
                        )

    print('All figs done.')










if __name__ == "__main__":
    sim_type = 'Mecanum'
    sim_name = 'sin_tt'
    sim_time = 10
    # run_motion_simulation(sim_type,sim_name,sim_time)
    plot_results(sim_type,sim_name,sim_time,
                 fig_type='.pdf',
                 if_trajectory_tracking = True,
                 if_animation = False)

