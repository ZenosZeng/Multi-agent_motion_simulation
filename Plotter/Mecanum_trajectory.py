from matplotlib import pyplot as plt
import numpy as np
import matplotlib.patches as patches
from math import cos, sin

def body_points_mecanum(state, width=0.5, length=0.6):
    '''
    Create a list of points representing the Mecanum wheeled robot's body.
    The centroid is now at the center of the vehicle.
    '''
    x, y, theta = state

    # width = width / 2
    # length = length / 2

    # Compute the four corners relative to the center
    x1 = x + length * cos(theta) - width * sin(theta)  # Front Left
    y1 = y + length * sin(theta) + width * cos(theta)

    x2 = x - length * cos(theta) - width * sin(theta)  # Rear Left
    y2 = y - length * sin(theta) + width * cos(theta)

    x3 = x - length * cos(theta) + width * sin(theta)  # Rear Right
    y3 = y - length * sin(theta) - width * cos(theta)

    x4 = x + length * cos(theta) + width * sin(theta)  # Front Right
    y4 = y + length * sin(theta) - width * cos(theta)

    body_points = [(x1, y1), (x2, y2), (x3, y3), (x4, y4)]
    return body_points


def wheel_points_mecanum(body_points, theta, width=0.1, length=0.2):
    '''
    Compute the four wheel positions for a Mecanum wheeled robot.
    '''
    wheels = []
    for x, y in body_points:
        x1 = x + length * cos(theta) - width * sin(theta)
        y1 = y + length * sin(theta) + width * cos(theta)

        x2 = x - length * cos(theta) - width * sin(theta)
        y2 = y - length * sin(theta) + width * cos(theta)

        x3 = x - length * cos(theta) + width * sin(theta)
        y3 = y - length * sin(theta) - width * cos(theta)

        x4 = x + length * cos(theta) + width * sin(theta)
        y4 = y + length * sin(theta) - width * cos(theta)

        wheels.append([(x1, y1), (x2, y2), (x3, y3), (x4, y4)])

    return wheels


def Mecanum_trajectory(data,
                       color_list, label_list, edge_list, drawtime_list,
                       save_path, 
                       target_trajectory=None,
                       trajectory_width=1, point_size=0.8, edge_width=0.5,
                       xy_range=[], Show=True):

    num_agent = len(data)
    total_step = len(data[0][0])

    fig, ax = plt.subplots()

    # Plot trajectory
    for i in range(num_agent):
        if label_list[i] != 'x':
            ax.plot(data[i][0], data[i][1],
                    color=color_list[i], label=label_list[i],
                    linewidth=trajectory_width, alpha=0.8)
        else:
            ax.plot(data[i][0], data[i][1],
                    color=color_list[i],
                    linewidth=trajectory_width, alpha=0.8)

    # Display target trajectory
    if target_trajectory is not None:
        ax.plot(target_trajectory[0], target_trajectory[1],
                color='purple', label='参考轨迹',
                linewidth=trajectory_width * 1.2, linestyle='--', alpha=0.9)

    # Adjust drawtime_list
    drawtime_list = [int(x * 50) for x in drawtime_list]
    drawtime_list.append(0)
    drawtime_list.append(-1)

    for t in drawtime_list:
        # Display edge between agents
        for a, b in edge_list:
            if t == 0:
                continue
            x = [data[a][0][t],
                 data[b][0][t]]
            y = [data[a][1][t],
                 data[b][1][t]]
            ax.plot(x, y, color='gray', linestyle='dashed',
                    linewidth=edge_width, alpha=0.6)

        # Display agent body
        for i in range(num_agent):
            body_points = body_points_mecanum([data[i][0][t],
                                               data[i][1][t],
                                               data[i][2][t]])
            body = patches.Polygon(body_points, closed=True,
                                   lw=point_size,
                                   edgecolor=color_list[i],
                                   facecolor='none')

            # Draw four wheels
            wheels = wheel_points_mecanum(body_points, data[i][2][t])
            wheel_patches = [patches.Polygon(w, closed=True, lw=point_size,
                                             edgecolor=color_list[i], facecolor=color_list[i])
                             for w in wheels]

            ax.add_artist(body)
            for wheel in wheel_patches:
                ax.add_artist(wheel)

    # Set axis
    ax.set_aspect(1)
    plt.xlabel(r'$x$')
    plt.ylabel(r'$y$')
    plt.tick_params(axis='both', direction='in', labelsize=12)
    plt.legend(fontsize=10, loc='best', frameon=True, edgecolor='black')

    if xy_range:
        plt.xlim(xy_range[0], xy_range[1])
        plt.ylim(xy_range[2], xy_range[3])

    if Show:
        plt.show()
    else:
        plt.savefig(save_path, dpi=300, bbox_inches='tight', pad_inches=0.1)
    plt.close()

    print('Mecanum Trajectory Plotted.')
    print('-' * 30)
