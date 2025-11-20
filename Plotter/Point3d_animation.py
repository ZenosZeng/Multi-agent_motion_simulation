from matplotlib.animation import FuncAnimation
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def Point3d_animation(  data, 
                        color_list,edge_list,
                        save_path,
                        trajectory_width=0.6, point_size=10,edge_width=0.5,
                        target_trajectory=None,
                        Show=True):
    '''
    3D Animation for masspoint motion
    data--a list of agent data with 3 dimensions (agent-dimension-value along time)
    '''
    num_agent = len(data)
    dimension = len(data[0])
    total_step = len(data[0][0])

    print('3D Animation Frequency: 50Hz')
    print('Trajectory data length: ' + str(len(data[0][0])))

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_aspect('equal')
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')

    line = [None for _ in range(num_agent)]
    body = [None for _ in range(num_agent)]
    edge = [None for _ in range(len(edge_list))]

    t = 0
    for i in range(num_agent):
        # Display trajectory
        line[i] = ax.plot(data[i][0][:t], data[i][1][:t], data[i][2][:t],
                          color=color_list[i],
                          linewidth=trajectory_width)
        # Display body
        body[i], = ax.plot([data[i][0][t]], [data[i][1][t]], [data[i][2][t]],
                            marker='o', markersize=point_size,
                            color=color_list[i])

    # Display edge between agents
    for i in range(len(edge_list)):
        a, b = edge_list[i]
        x = [data[a][0][t], data[b][0][t]]
        y = [data[a][1][t], data[b][1][t]]
        z = [data[a][2][t], data[b][2][t]]
        edge[i] = ax.plot(x, y, z, color='grey', linestyle='dashed',
                          linewidth=edge_width)

    # Display target trajectory
    if target_trajectory is not None:
        ax.plot(target_trajectory[0], target_trajectory[1], target_trajectory[2],
                color='purple', label='Target Trajectory',
                linewidth=trajectory_width*1.2, linestyle='--')

    def update(t):
        ax.set_title(f't = {t / 50:.1f} s')
        x_ave,y_ave,z_ave = 0,0,0

        for i in range(num_agent):
            # Update trajectory
            line[i][0].set_data(data[i][0][:t], data[i][1][:t])
            line[i][0].set_3d_properties(data[i][2][:t])

            x_ave += data[i][0][t]
            y_ave += data[i][1][t]
            z_ave += data[i][2][t]

            # Update body
            body[i].set_data([data[i][0][t]], [data[i][1][t]])
            body[i].set_3d_properties([data[i][2][t]])

        # update plot range
        x_ave /= num_agent
        y_ave /= num_agent
        z_ave /= num_agent
        ax.set_xlim(x_ave-10,x_ave+10)
        ax.set_ylim(y_ave-10,y_ave+10)
        ax.set_zlim(z_ave-10,z_ave+10)

        # Update edges
        for i in range(len(edge_list)):
            a, b = edge_list[i]
            x = [data[a][0][t], data[b][0][t]]
            y = [data[a][1][t], data[b][1][t]]
            z = [data[a][2][t], data[b][2][t]]
            edge[i][0].set_data(x, y)
            edge[i][0].set_3d_properties(z)

        return line + body + edge

    ani = FuncAnimation(fig, update, frames=int(total_step), interval=20)

    if Show:
        plt.show()
    else:
        ani.save(save_path,fps=50,dpi=100)
    plt.close()

    print('3D Animation Done.')
    print('-' * 30)
