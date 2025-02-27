import numpy as np
from matplotlib import pyplot as plt
from math import cos,sin,pi,sqrt,atan2
import math
import sys
import json

from Model import Single_Integrator as SI
import Ploting as pt

def Saturation(x,bound):
    x = np.array(x)
    x_m = np.linalg.norm(x)
    if x_m > bound:
        return x/x_m*bound
    else: 
        return x
    
def tanh(x,bound):
    return [bound*math.tanh(y) for y in x]

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

# 1 initialize agent and data list ------------------------------------------

# agent basic para
num_agent = 6
dim_state = 2
dim_control = 2

# sim time para
total_seconds = 10
time_step = 1e-4
total_step = int(total_seconds/time_step)
frequency = int(1/time_step)

# create agent class and data_recorder 
agent=[ SI(dt=time_step) for i in range(num_agent) ]
# 3d data list: agnet+state_dimension+value along time
trajectory_data = [ [ [  ] for i in range(dim_state) ] for i in range(num_agent) ]
 
# graph definition    
adjencency_matrix = [ [0,0,0,0,0,0],
                      [1,0,1,1,1,1],
                      [1,1,0,0,0,1],
                      [1,1,0,0,1,0],
                      [0,1,0,1,0,1],
                      [0,1,1,0,1,0],  ] # a_ij , i->j means i keep the distance to j

# formation shape (five ploygon)
bian = 5
bianxin = 5*0.5/cos(54*pi/180)
distance_matrix = [   [0,0,0,0,0,0],
                      [bianxin,0,bianxin,bianxin,bianxin,bianxin],
                      [bian,bianxin,0,0,0,bian],
                      [bian,bianxin,0,0,bian,0],
                      [0,bianxin,0,bian,0,bian],
                      [0,bianxin,bian,0,bian,0],        ] 

# autmatically identify the edges in the graph
edge_list = []
distance_err = []
orientation_err = []

for i in range(num_agent):
    for j in range(i):
        a = adjencency_matrix[i][j]
        a2 = adjencency_matrix[j][i]
        if a>0 or a2>0: # exist directed edge
            edge_list.append([min(i,j),max(i,j)])
            distance_err.append([])

# reset pos and save data
# d1 = 8
# agent[0].reset([0,d1,0])
# agent[1].reset([0,0,0])
# agent[2].reset([d1,0,0])
# agent[3].reset([d1,d1,0])

agent[0].reset([0,0])
agent[1].reset([-6,6])
agent[2].reset([-4,-5])
agent[3].reset([-2,12])
agent[4].reset([-12,12])
agent[5].reset([-14,-2])

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
    distance_err[i].append(err)

# orientation error recording
def angle_transform(a):
    if a<-pi:
        return angle_transform(a+2*pi)
    elif a>=pi:
        return angle_transform(a-2*pi)
    else:
        return a

theta = 0
p0 = np.array(agent[0].state)
p1 = np.array(agent[1].state)
po=p0-p1 # relative position vector of orientation
po_d = [bianxin*cos(theta),bianxin*sin(theta)]
po_bar = po-po_d # error of p10
orientation_err.append(np.linalg.norm(po_bar))

print('-'*30)
print('init done.')
print('-'*30)











# 2 motion simulation -------------------------------------------------------------------

for step in range(total_step):
    # reset the r vector, r_i = sum_j(p_ij*sigma_ij)
    r = [ [0 for i in range(dim_control)] for j in range(num_agent)]

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

    # [ v_d(t) design ] ------------------------------------------------
    vx = 5
    vy = 0
    if t<3:
        theta = 0
        dtheta = 0
    elif 3<=t<3+2*pi:
        theta = 0.5*(t-3)
        dtheta = 0.5
    else:
        theta = pi
        dtheta = 0

    vd = np.array([vx,vy]) 
    # ---------------------------------------------------------------
    
    p0 = np.array(agent[0].state[:2])
    p1 = np.array(agent[1].state[:2])

    po=p0-p1 # relative position vector of orientation
    po_d = [bianxin*cos(theta),bianxin*sin(theta)]
    po_d_dot = [-bianxin*sin(theta)*dtheta,bianxin*cos(theta)*dtheta]
    po_bar = po-po_d # error of p10
    
    # [ control gain ] ------------------------------------------------
    k = 20 # gain for basic rigid term 
    beta = 15 #  gain for signal term
    alpha = 40 # gain for orientation
    
    v_max_coleader = 18
    v_max_follower = 18
    # -------------------------------------------------------------

    for i in range(num_agent):
        # u[i] = saturation_vector(u[i],50)
        if i==0:
            agent[i].step(vd)
        elif i==1:
            eta = alpha*np.dot(po_bar,po_d_dot)/(np.linalg.norm(r[i]-alpha*po_bar)**2)
            u1 = -(k-eta)*(r[i]-alpha*po_bar) + vd
            u1 = Saturation(u1,v_max_coleader)
            # u1 = tanh(u1,20)
            agent[i].step(u1)
        else:
            u1 = -k*r[i] - beta*Sgn(r[i])
            u1 = Saturation(u1,v_max_follower)
            # u1 = tanh(u1,20)
            agent[i].step(u1)

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
            distance_err[i].append(err)

        # orientation error recording
        orientation_err.append(np.linalg.norm(po_bar))
 
    # display process by 1Hz
    if step%frequency==0:
        print('Simulation Time: %.0f/%.0fs '%(time,total_seconds))

print('-'*30)
print('Motion Calculated.')
print('-'*30)









 # 3 save data to json ----------------------------------------------------------   

data_label = 'demo1'

results = {
    "trajectory": np.array(trajectory_data).tolist(),
    "distance_error": np.array(distance_err).tolist(),
    "orientation_error": np.array(orientation_err).tolist(),
    "edge_list": np.array(edge_list).tolist()
}
 
# 存入 JSON
with open('./log/'+data_label+".json", "w") as f:
    json.dump(results, f, indent=4)

print('Data saved, View <./log/'+data_label+'> for details.')
print('-'*30)

