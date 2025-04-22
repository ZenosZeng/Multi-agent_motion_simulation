from matplotlib import pyplot as plt
import numpy as np
import matplotlib.patches as patches
from math import cos,sin
from matplotlib.animation import FuncAnimation


def body_points_unicycle(state,width=0.5,length=0.6):
    '''
    Create a list of points representing the body
    centroid is on the middle of the rear wheel
    '''
    x=state[0]
    y=state[1]
    theta=state[2]

    x1 = x + length*cos(theta) - width*sin(theta)
    y1 = y + width*cos(theta) + length*sin(theta) # front left

    x2 = x - width*sin(theta)
    y2 = y + width*cos(theta) # rear left

    x3 = x + width*sin(theta)
    y3 = y - width*cos(theta) # rear right

    x4 = x + length*cos(theta) + width*sin(theta)
    y4 = y - width*cos(theta) + length*sin(theta) # front right

    x5 = x + (length+length)*cos(theta) 
    y5 = y + (length+length)*sin(theta) # head

    body_points = [(x1,y1),(x2,y2),(x3,y3),(x4,y4),(x5,y5)]
 
    # return [x1,x2,x3,x4,x5,x1],[y1,y2,y3,y4,y5,y1]
    return body_points



def wheel_points_unicycle(body_point,theta,width=0.1,length=0.2):
    '''
    Create a list of points representing the wheel of unicycle
    '''
    # rear left wheel
    x,y = body_point[1]

    x1 = x + length*cos(theta) - width*sin(theta)
    y1 = y + width*cos(theta) + length*sin(theta) # front left

    x2 = x - width*sin(theta) - length*cos(theta)
    y2 = y + width*cos(theta) - length*sin(theta) # rear left

    x3 = x + width*sin(theta) - length*cos(theta)
    y3 = y - width*cos(theta) - length*sin(theta) # rear right

    x4 = x + length*cos(theta) + width*sin(theta)
    y4 = y - width*cos(theta) + length*sin(theta) # front right

    wheel_points_rear_left = [(x1,y1),(x2,y2),(x3,y3),(x4,y4)]

    # rear right wheel
    x,y = body_point[2]

    x1 = x + length*cos(theta) - width*sin(theta)
    y1 = y + width*cos(theta) + length*sin(theta) # front left

    x2 = x - width*sin(theta) - length*cos(theta)
    y2 = y + width*cos(theta) - length*sin(theta) # rear left

    x3 = x + width*sin(theta) - length*cos(theta)
    y3 = y - width*cos(theta) - length*sin(theta) # rear right

    x4 = x + length*cos(theta) + width*sin(theta)
    y4 = y - width*cos(theta) + length*sin(theta) # front right

    wheel_points_rear_right = [(x1,y1),(x2,y2),(x3,y3),(x4,y4)]
 
    return wheel_points_rear_left,wheel_points_rear_right



def Unicycle_animation(data,
                       color_list,edge_list,
                        save_path,
                        offset,
                        trajectory_width=1,point_size=0.8,edge_width=0.5,
                        target_trajectory = None,
                        Show=True):
    '''
    Animation function for masspoint motion
    Input as follows:
    data--a list of agent data with 3 dimensions (agent-dimension-value along time)
    !! the data frequency must be 50Hz !!
    timestep-- 1/frequency(Hz)
    color_list--a list of color for each agent
    label_list--a list of label for each agent
    '''
    num_agent = len(data)
    dimension = len(data[0])
    total_step = len(data[0][0])

    print('Animation Frequency: 50Hz')        
    print('Trajectory data length: '+str(len(data[0][0])))

    fig, ax = plt.subplots()
    ax.set_aspect(1)
    ax.set_xlabel('x')
    ax.set_ylabel('y')

    line = [ 0 for i in range(num_agent) ]
    body = [ 0 for i in range(num_agent) ]
    edge = [ 0 for i in range(len(edge_list)) ]
    wheel= [ [0,0] for i in range(num_agent) ]

    t=0
    for i in range(num_agent):
        # display trajectory
        line[i]=ax.plot(data[i][0][:t],data[i][1][:t],\
                        color=color_list[i],
                        linewidth=trajectory_width)
        # display body
        body_point = body_points_unicycle([data[i][0][t],data[i][1][t],data[i][2][t]])
        body[i] = patches.Polygon(body_point, closed=True,lw=point_size, edgecolor=color_list[i], facecolor='none')
        ax.add_artist(body[i])
        # wheel
        wheel_point_1,wheel_point_2 = wheel_points_unicycle(body_point,data[i][2][t])
        wheel[i][0] = patches.Polygon(wheel_point_1, closed=True,lw=point_size, \
                                edgecolor=color_list[i], facecolor=color_list[i])
        wheel[i][1] = patches.Polygon(wheel_point_2, closed=True,lw=point_size, \
                                edgecolor=color_list[i], facecolor=color_list[i])
        ax.add_artist(wheel[i][0])
        ax.add_artist(wheel[i][1])
       
            
    # display edge between agent
    for i in range(len(edge_list)): 
        a,b = edge_list[i]
        x = [data[a][0][t]+offset*cos(data[a][2][t]),
                data[b][0][t]+offset*cos(data[b][2][t])]
        y = [data[a][1][t]+offset*sin(data[a][2][t]),
                data[b][1][t]+offset*sin(data[b][2][t])]
        edge[i]=ax.plot(x,y,color='grey',linestyle='dashed',
                linewidth=edge_width)

    # display target trajectory
    if target_trajectory is not None:
        tt = ax.plot(target_trajectory[0], target_trajectory[1],
                color='purple',label='Target Trajectory',
                linewidth=trajectory_width*1.2,linestyle='--')

    def update(t):
        # t += timestep  
            
        ax.set_title('t = {:.1f} s'.format(t/50))
        x_ave,y_ave = 0,0

        for i in range(num_agent):
            # update trajectory
            line[i][0].set_data(data[i][0][:t],data[i][1][:t])
            x_ave += data[i][0][t]
            y_ave += data[i][1][t]
            
            # update body
            body_point = body_points_unicycle([data[i][0][t],data[i][1][t],data[i][2][t]])
            body[i].set_xy(body_point)
            
            wheel_point_1,wheel_point_2 = wheel_points_unicycle(body_point,data[i][2][t])
            wheel[i][0].set_xy(wheel_point_1)
            wheel[i][1].set_xy(wheel_point_2)
        
        # update plot range
        x_ave /= num_agent
        y_ave /= num_agent
        ax.set_xlim(x_ave-15,x_ave+15)
        ax.set_ylim(y_ave-15,y_ave+15)
        # plt.grid(True)

        # update edge
        for i in range(len(edge_list)):
            a,b = edge_list[i]  
            x = [data[a][0][t]+offset*cos(data[a][2][t]),
                data[b][0][t]+offset*cos(data[b][2][t])]
            y = [data[a][1][t]+offset*sin(data[a][2][t]),
                    data[b][1][t]+offset*sin(data[b][2][t])]
            edge[i][0].set_data(x,y)
        
        if target_trajectory is not None:
            return line,body,edge,tt,wheel
        else:
            return line,body,edge,wheel

    
    ani=FuncAnimation(fig,update,interval=20,frames=int(total_step))
    if Show:
        plt.show()
    else:
        ani.save(save_path,fps=50,dpi=100)
    plt.close()

    print('animation done.')
    print('-'*30)