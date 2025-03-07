import numpy as np
from matplotlib import pyplot as plt
from math import cos,sin,pi,sqrt,atan2,tanh
import json
import os

from Model import Single_Integrator as SI
from Model import Unicycle 

import Ploting as pt

def Saturation(x,bound):
    x = np.array(x)
    x_m = np.linalg.norm(x)
    if x_m > bound:
        return x/x_m*bound
    else: 
        return x
    
def tanh(x,bound):
    return [bound*tanh(y) for y in x]

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

def u2vw(u,theta_agent,offset):
    m = sqrt(u[0]**2 + u[1]**2)
    theta_d = atan2(u[1],u[0])
    theta_err = theta_agent-theta_d
    V = m*cos(theta_err)
    W = -m*sin(theta_err)/offset
    return V,W

def run_motion_simulation(sim_name,sim_time,offset):
    # agent basic parameter
    num_agent = 6
    dim_state = 3
    dim_control = 2

    # sim time para
    total_seconds = sim_time
    time_step = 1e-4
    total_step = int(total_seconds/time_step)
    frequency = int(1/time_step)

    # create agent class and data_recorder 
    agent=[ Unicycle(dt=time_step) for i in range(num_agent) ]

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
        x1,y1 = agent[a].output_pos(offset)
        x2,y2 = agent[b].output_pos(offset)
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
    p0 = np.array(agent[0].output_pos(offset))
    p1 = np.array(agent[1].output_pos(offset))
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
                p_i = np.array(agent[i].output_pos(offset))
                p_j = np.array(agent[j].output_pos(offset))
                d = distance_matrix[i][j] # desired distance
                p_ij= p_i-p_j 
                p_ij_m = np.linalg.norm(p_ij) # L2 norm
                sigma_ij = p_ij_m**2-d**2 # square distance error
                r_j = p_ij*sigma_ij

                r[i] = np.add(r[i],r_j)

        # target speed definition
        t = time

        # [ v_d(t) design ] ------------------------------------------------
        # vx = 5
        # vy = 0
        # if t<3:
        #     theta = 0
        #     dtheta = 0
        # elif 3<=t<3+2*pi:
        #     theta = 0.5*(t-3)
        #     dtheta = 0.5
        # else:
        #     theta = pi
        #     dtheta = 0

        vx = 5
        vy = 3*sin(t)
        dtheta = 1/((3/5*sin(t))**2+1)*3/5*cos(t)
        theta = atan2(vy,vx)
        vd = np.array([vx,vy]) 
        
        # ---------------------------------------------------------------
        
        p0 = np.array(agent[0].output_pos(offset))
        p1 = np.array(agent[1].output_pos(offset))

        po=p0-p1 # relative position vector of orientation
        po_d = [bianxin*cos(theta),bianxin*sin(theta)]
        po_d_dot = [-bianxin*sin(theta)*dtheta,bianxin*cos(theta)*dtheta]
        po_bar = po-po_d # error of p10

        theta_list = [ agent[i].state[2] for i in range(num_agent) ]
        
        # [ control gain ] ------------------------------------------------
        k = 5 # gain for basic rigid term 
        beta = 15 #  gain for signal term
        alpha = 20 # gain for orientation
        
        v_max_coleader = 15
        v_max_follower = 15
        # -------------------------------------------------------------

        for i in range(num_agent):
            if i==0:
                # agent[i].step(vd)
                v,w = u2vw(vd,theta_list[i],offset)
                agent[i].step([v,w])
            elif i==1:
                eta = alpha*np.dot(po_bar,po_d_dot)/(np.linalg.norm(r[i]-alpha*po_bar)**2)
                u1 = -(k-eta)*(r[i]-alpha*po_bar) + vd
                u1 = Saturation(u1,v_max_coleader)
                # u1 = tanh(u1,20)
                # agent[i].step(u1)
                v,w = u2vw(u1,theta_list[i],offset)
                agent[i].step([v,w])
            else:
                u1 = -k*r[i] - beta*Sgn(r[i])
                u1 = Saturation(u1,v_max_follower)
                # u1 = tanh(u1,20)
                # agent[i].step(u1)
                v,w = u2vw(u1,theta_list[i],offset)
                agent[i].step([v,w])

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
                x1,y1 = agent[a].output_pos(offset)
                x2,y2 = agent[b].output_pos(offset)
                d = sqrt( (x1-x2)**2+(y1-y2)**2 )
                err = d-max(distance_matrix[a][b],distance_matrix[b][a])
                distance_error[i].append(err)

            # orientation error recording
            orientation_error.append(np.linalg.norm(po_bar))
    
        # display process by 1Hz
        if step%frequency==0:
            print('Simulation Time: %.0f/%.0fs '%(time,total_seconds))

    print('-'*30)
    print('Motion Calculated.')
    print('-'*30)

    # create folder for figure
    folder_path = f"./log/{sim_name}"
    os.makedirs(folder_path, exist_ok=True) 
    
    # save to json
    results = {
        "trajectory": np.array(trajectory_data).tolist(),
        "distance_error": np.array(distance_error).tolist(),
        "orientation_error": np.array(orientation_error).tolist(),
        "edge_list": np.array(edge_list).tolist()
    }
    
    with open(f'./log/{sim_name}/{sim_name}.json', "w") as f:
        json.dump(results, f, indent=4)

    print('Data saved, View <./log/'+sim_name+'> for details.')
    print('-'*30)






def plot_results(sim_name,sim_time,offset,
                 if_trajectory_tracking = False,
                 if_animation = True):
    
    # read motion data
    with open(f'./log/{sim_name}/{sim_name}.json', "r") as f:
        data = json.load(f)  

    # output fig name
    preffix = f'./log/{sim_name}/'
    suffix = ''

    # define ploting parameter
    color_list = ['blue','green',
                  'darkgrey','darkgrey','darkgrey','darkgrey']

    label_list = ['Leader','Co-Leader','Followers','x','x','x']

    drawtime_list = [2,5,7.5]

    # ploting

    # 1 trajectory
    pt.Plot_Trajectory('Unicycle',
                    data['trajectory'],
                    color_list,label_list,data['edge_list'],drawtime_list,
                    offset=offset,
                    target_trajectory=\
                        data['target_trajectory'] if if_trajectory_tracking else None,
                    pltrange_xy = [-19,54,-11,24],
                    if_label=True,
                    Show=False,
                    SavePath=preffix+'tra'+suffix+'.png' )

    # 2 distance error
    label_list = [ 'x' for a in range(len(data['distance_error'])-1) ]
    label_list.append(r'$\|p_{ij}\|-d_{ij}$')
    print(label_list)

    pt.plot_err(data['distance_error'],
             xy_label=['Time (s)','Distance error'],
             line_width=0.8,
             plt_label=label_list,
             pltrange_x=[0,sim_time],
             pltrange_y=[],
             color_list=[],
             SavePath=preffix+'err_d'+suffix+'.png')

    # 3 normalized error 
    sigma_list = np.array([])
    distance_err = np.array(data['distance_error'])
    for i in range(len(distance_err[0])):
        err = distance_err[:,i]
        sigma_list = np.append(sigma_list, np.linalg.norm(err))
    sigma_list /= sigma_list[0]

    label = [r'$\frac{\|\sigma(t)\|}{\|\sigma(0)\|}$' ]
    print(label)
    pt.plot_err([sigma_list],
             xy_label=['Time (s)','Normalized error'],
             line_width=0.8,
             plt_label=label,
             pltrange_x=[0,sim_time],
             pltrange_y=[-0.1,1.1],
             color_list=['blue'],
             SavePath=preffix+'err_n'+suffix+'.png')

    # 4 orientation error
    label = [r'$\left\| p_o-p^*_o(t) \right\|$']
    print(label)
    pt.plot_err([data['orientation_error']],
             xy_label=['Time (s)','Orientation error'],
             line_width=0.8,
             plt_label=label,
             pltrange_x=[0,sim_time],
             pltrange_y=[],
             color_list=['blue'],
             SavePath=preffix+'err_o'+suffix+'.png')
    
    # 5 tracking error
    if if_trajectory_tracking:
        label = [r'$\| p_1-p^*(t) \|$']
        print(label)
        pt.plot_err(data['tracking_error'],
                xy_label=['Time (s)','Tracking error'],
                line_width=0.8,
                plt_label=label,
                pltrange_x=[0,sim_time],
                pltrange_y=[],
                color_list=['blue'],
                SavePath=preffix+'err_t'+suffix+'.png')

    # 6 Animation
    if if_animation:
        pt.Animation_motion('Unicycle',
                        data['trajectory'],color_list,label_list,data['edge_list'],
                        offset=offset,
                        target_trajectory=\
                            data['target_trajectory'] if if_trajectory_tracking else None,
                        Show=False,
                        SavePath=preffix+'ani'+suffix+'.gif')

    print('All figs done.')










if __name__ == "__main__":
    sim_name = 'unicycle_1'
    sim_time = 10
    offset = 0.6
    # run_motion_simulation(sim_name,sim_time,offset)
    plot_results(sim_name,sim_time,offset,
                 if_trajectory_tracking = False,
                 if_animation = False)

