from matplotlib import pyplot as plt
import numpy as np
import matplotlib.patches as patches
from math import cos,sin
from matplotlib.animation import FuncAnimation

def Point_animation(data,color_list,
                     edge_list,
                     save_path,
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
        
        # display si body
        body[i] = patches.Circle((data[i][0][t],data[i][1][t]),point_size,
                            edgecolor=color_list[i],facecolor='snow')
        ax.add_artist(body[i])
            
    # display edge between agent
    for i in range(len(edge_list)): 
        a,b = edge_list[i]
        x = [data[a][0][t],data[b][0][t]]
        y = [data[a][1][t],data[b][1][t]]
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
            body[i].set_center((data[i][0][t],data[i][1][t]))

        # update plot range
        x_ave /= num_agent
        y_ave /= num_agent
        ax.set_xlim(x_ave-15,x_ave+15)
        ax.set_ylim(y_ave-15,y_ave+15)
        # plt.grid(True)

        # update edge
        for i in range(len(edge_list)):
            a,b = edge_list[i]  
            x = [data[a][0][t],data[b][0][t]]
            y = [data[a][1][t],data[b][1][t]]
            edge[i][0].set_data(x,y)
        
        if target_trajectory is not None:
            return line,body,edge,tt
        else:
            return line,body,edge
    
    ani=FuncAnimation(fig,update,interval=20,frames=int(total_step))
    if Show:
        plt.show()
    else:
        ani.save(save_path,fps=50,dpi=100)
    plt.close()

    print('animation done.')
    print('-'*30)