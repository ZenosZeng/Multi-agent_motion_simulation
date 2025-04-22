from matplotlib import pyplot as plt
import numpy as np
import matplotlib.patches as patches
from math import cos, sin, pi
from matplotlib.animation import FuncAnimation

def body_points_mecanum(state, width=0.5, length=0.6):
    '''
    Create a list of points representing the body
    centroid is at the center of the robot
    '''
    x, y, theta = state

    x1 = x + length * cos(theta) - width * sin(theta)
    y1 = y + width * cos(theta) + length * sin(theta)  # front left

    x2 = x - length * cos(theta) - width * sin(theta)
    y2 = y + width * cos(theta) - length * sin(theta)  # rear left

    x3 = x - length * cos(theta) + width * sin(theta)
    y3 = y - width * cos(theta) - length * sin(theta)  # rear right

    x4 = x + length * cos(theta) + width * sin(theta)
    y4 = y - width * cos(theta) + length * sin(theta)  # front right

    return [(x1, y1), (x2, y2), (x3, y3), (x4, y4)]

def wheel_points_mecanum(body_point, theta, width=0.1, length=0.2):
    '''
    Create a list of points representing the four mecanum wheels
    '''
    wheels = []
    for i in range(4):
        x, y = body_point[i]
        x1 = x + length * cos(theta) - width * sin(theta)
        y1 = y + width * cos(theta) + length * sin(theta)
        x2 = x - width * sin(theta) - length * cos(theta)
        y2 = y + width * cos(theta) - length * sin(theta)
        x3 = x + width * sin(theta) - length * cos(theta)
        y3 = y - width * cos(theta) - length * sin(theta)
        x4 = x + length * cos(theta) + width * sin(theta)
        y4 = y - width * cos(theta) + length * sin(theta)
        wheels.append([(x1, y1), (x2, y2), (x3, y3), (x4, y4)])
    return wheels

def Mecanum_animation(data, 
                      color_list, edge_list, 
                      save_path, 
                      trajectory_width=1, point_size=0.8, edge_width=0.5, target_trajectory=None, 
                      Show=True):
    
    num_agent = len(data)
    total_step = len(data[0][0])

    fig, ax = plt.subplots()
    ax.set_aspect(1)
    ax.set_xlabel('x')
    ax.set_ylabel('y')

    line = [0 for _ in range(num_agent)]
    body = [0 for _ in range(num_agent)]
    edge = [0 for _ in range(len(edge_list))]
    wheels = [[0, 0, 0, 0] for _ in range(num_agent)]

    t = 0
    for i in range(num_agent):
        line[i] = ax.plot(data[i][0][:t], data[i][1][:t], color=color_list[i], linewidth=trajectory_width)
        body_point = body_points_mecanum([data[i][0][t], data[i][1][t], data[i][2][t]])
        body[i] = patches.Polygon(body_point, closed=True, lw=point_size, edgecolor=color_list[i], facecolor='none')
        ax.add_artist(body[i])
        
        wheel_points = wheel_points_mecanum(body_point, data[i][2][t])
        for j in range(4):
            wheels[i][j] = patches.Polygon(wheel_points[j], closed=True, lw=point_size, edgecolor=color_list[i], facecolor=color_list[i])
            ax.add_artist(wheels[i][j])
    
    for i in range(len(edge_list)):
        a, b = edge_list[i]
        x = [data[a][0][t], data[b][0][t]]
        y = [data[a][1][t], data[b][1][t]]
        edge[i] = ax.plot(x, y, color='grey', linestyle='dashed', linewidth=edge_width)
    
    if target_trajectory is not None:
        tt = ax.plot(target_trajectory[0], target_trajectory[1], color='purple', label='Target Trajectory', linewidth=trajectory_width * 1.2, linestyle='--')
    
    def update(t):
        ax.set_title('t = {:.1f} s'.format(t / 50))
        x_ave, y_ave = 0, 0
        
        for i in range(num_agent):
            line[i][0].set_data(data[i][0][:t], data[i][1][:t])
            x_ave += data[i][0][t]
            y_ave += data[i][1][t]
            
            body_point = body_points_mecanum([data[i][0][t], data[i][1][t], data[i][2][t]])
            body[i].set_xy(body_point)
            
            wheel_points = wheel_points_mecanum(body_point, data[i][2][t])
            for j in range(4):
                wheels[i][j].set_xy(wheel_points[j])
        
        x_ave /= num_agent
        y_ave /= num_agent
        ax.set_xlim(x_ave - 15, x_ave + 15)
        ax.set_ylim(y_ave - 15, y_ave + 15)
        
        for i in range(len(edge_list)):
            a, b = edge_list[i]
            x = [data[a][0][t], data[b][0][t]]
            y = [data[a][1][t], data[b][1][t]]
            edge[i][0].set_data(x, y)
        
        if target_trajectory is not None:
            return line, body, edge, tt, wheels
        else:
            return line, body, edge, wheels
    
    ani = FuncAnimation(fig, update, interval=20, frames=int(total_step))
    if Show:
        plt.show()
    else:
        ani.save(save_path, fps=50, dpi=100)
    plt.close()

    print('animation done.')
    print('-' * 30)
