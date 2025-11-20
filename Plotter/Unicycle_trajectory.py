from matplotlib import pyplot as plt
import numpy as np
import matplotlib.patches as patches
from math import cos,sin

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



def Unicycle_trajectory(data,
                        color_list,label_list,edge_list,drawtime_list,
                        offset,
                        save_path,
                        target_trajectory=None,
                        trajectory_width=1,point_size=0.8,edge_width=0.5,
                        xy_range = [],
                        Show=True
                        ):

    num_agent = len(data)
    dimension = len(data[0])
    total_step = len(data[0][0])

    fig, ax = plt.subplots() # figsize=(1920/200,1080/200)

    # 1 plot trajectory 
    for i in range(num_agent):
        # trajectory
        if label_list[i] != 'x':
            ax.plot(data[i][0],data[i][1],
                    color=color_list[i],label=label_list[i],
                    linewidth=trajectory_width,
                    alpha=0.8)
        else:
            ax.plot(data[i][0],data[i][1],
                    color=color_list[i],
                    linewidth=trajectory_width,
                    alpha=0.8)
    
    # display target trajectory
    if target_trajectory is not None:
        ax.plot(target_trajectory[0], target_trajectory[1],
                color='purple',label='参考轨迹',
                linewidth=trajectory_width*1.2,linestyle='--',
                alpha=0.9)

    # 2 plot agent body in drawtime_list 
    drawtime_list = [ int(x*50) for x in drawtime_list]
    drawtime_list.append(0)
    drawtime_list.append(-1)

    for t in drawtime_list:
        # display edge between agent
        for a,b in edge_list:
            if t==0:
                continue 
            x = [data[a][0][t]+offset*cos(data[a][2][t]),
                    data[b][0][t]+offset*cos(data[b][2][t])]
            y = [data[a][1][t]+offset*sin(data[a][2][t]),
                    data[b][1][t]+offset*sin(data[b][2][t])]
            ax.plot(x,y,color='gray',linestyle='dashed',
                    linewidth=edge_width, alpha=0.6)
            
        # display agent body
        for i in range(num_agent):
            body_point = body_points_unicycle([data[i][0][t],
                                                data[i][1][t],
                                                data[i][2][t]])
            body = patches.Polygon(body_point, closed=True,
                                    lw = point_size, 
                                    edgecolor=color_list[i],
                                    facecolor='none')
            wheel_point_1,wheel_point_2 = wheel_points_unicycle(body_point,
                                                                data[i][2][t])
            wheel_1 = patches.Polygon(wheel_point_1, closed=True,lw = point_size, 
                                    edgecolor=color_list[i], facecolor=color_list[i])
            wheel_2 = patches.Polygon(wheel_point_2, closed=True,lw = point_size, 
                                    edgecolor=color_list[i], facecolor=color_list[i])
            ax.add_artist(body)
            ax.add_artist(wheel_1)
            ax.add_artist(wheel_2)
        
    # axe
    ax = plt.gca()
    ax.set_aspect(1)

    # plt.title('Trajectory')
    plt.xlabel(r'$x$')
    plt.ylabel(r'$y$')
    plt.tick_params(axis='both', direction='in',labelsize=12)

    plt.legend(fontsize=10, loc='best', frameon=True, edgecolor='black')
    
    if xy_range != []:
        plt.xlim(xy_range[0],xy_range[1])
        plt.ylim(xy_range[2],xy_range[3])

    if Show:
        plt.show()
    else:
        plt.savefig(save_path,dpi=300,bbox_inches='tight',pad_inches=0.1)
    plt.close()    

    print('MassPoint Trajectory Ploted.')
    print('-'*30)