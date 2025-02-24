# Package for Motion Animation
import numpy as np
from math import sqrt,cos,sin,atan2,atan,exp,tan,pi

from matplotlib import pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.patches as patches

from Ploting import plot_err











def Plot_Trajectory_3D(agent_type,
                       data,
                       color_list, label_list, edge_list, drawtime_list,
                       target_trajectory=None,
                       trajectory_width=0.6, point_size=10,
                       pltrange_x=[],pltrange_y=[],pltrange_z=[],
                       if_label=False,
                       SavePath='TRA_3D.png'):

    num_agent = len(data)
    dimension = len(data[0])
    total_step = len(data[0][0])

    # 1. 创建 3D 图
    fig = plt.figure() # figsize=(1920/200,1080/200)
    ax = fig.add_subplot(111, projection='3d')  # 设置 3D 投影

    # 2. 绘制轨迹
    for i in range(num_agent):
        if label_list[i] != 'x':
            ax.plot(data[i][0], data[i][1], data[i][2],
                    color=color_list[i], linewidth=trajectory_width,
                    label=label_list[i] )
        else:
            ax.plot(data[i][0], data[i][1], data[i][2],
                    color=color_list[i], linewidth=trajectory_width)

    # 3. 显示目标轨迹
    if target_trajectory is not None:
        ax.plot(target_trajectory[0], target_trajectory[1], target_trajectory[2],
                color='red', label='Reference Trajectory',
                linewidth=trajectory_width, linestyle='--')

    # 4. 绘制指定时间点的 agent 和连边
    drawtime_list = [int(x * 50) for x in drawtime_list]
    drawtime_list.append(0)
    drawtime_list.append(-1)

    for t in drawtime_list:
        # 显示 agent 间的连边
        for a, b in edge_list:
            if t == 0:
                continue
            x = [data[a][0][t], data[b][0][t]]
            y = [data[a][1][t], data[b][1][t]]
            z = [data[a][2][t], data[b][2][t]]
            ax.plot(x, y, z, color='grey', linestyle='dashed',
                    linewidth=trajectory_width*0.8)

        # 显示 agent 的身体
        if agent_type == 'SI':
            for i in range(num_agent):
                body = ax.scatter(data[i][0][t], data[i][1][t], data[i][2][t],
                                  color='snow',
                                  s=point_size, 
                                  edgecolors=color_list[i])
        else:
            raise Exception('Unknown agent type.')

    # 5. 设置轴标签和范围
    ax.set_aspect('equal')
    ax.set_xlabel('x',fontsize=12)
    ax.set_ylabel('y',fontsize=12)
    ax.set_zlabel('z',fontsize=12)

    ax.tick_params(axis='both', direction='in',)

    # ax.set_zticks([-5,0,5,10,15])
    # ax.set_yticks([0,10,20,30])

    if if_label:
        ax.legend(edgecolor='black',fancybox=False,framealpha=1,loc='upper left')

    if pltrange_x!=[]:
        ax.set_xlim(pltrange_x[0], pltrange_x[1])
    if pltrange_y!=[]:
        ax.set_ylim(pltrange_y[0], pltrange_y[1])
    if pltrange_z!=[]:
        ax.set_zlim(pltrange_z[0], pltrange_z[1])

    ax.view_init(elev=22, azim=-58)

    plt.savefig(SavePath, dpi=200,bbox_inches='tight', pad_inches=0.4) # 
    plt.close()

    print('MassPoint 3D Trajectory Plotted.')
    print('-' * 30)









def Animation_motion_3D(agent_type,
                        data, color_list, label_list,
                        edge_list,
                        trajectory_width=0.6, point_size=5,
                        target_trajectory=None,
                        if_label=False,
                        Show=True,
                        SavePath='ANI_3D.gif'):
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
                          color=color_list[i], label=label_list[i],
                          linewidth=trajectory_width)
        # Display body
        if agent_type == 'SI':
            body[i], = ax.plot([data[i][0][t]], [data[i][1][t]], [data[i][2][t]],
                               marker='o', markersize=point_size,
                               color=color_list[i])
        else:
            raise Exception('Unknown agent type.')

    # Display edge between agents
    for i in range(len(edge_list)):
        a, b = edge_list[i]
        x = [data[a][0][t], data[b][0][t]]
        y = [data[a][1][t], data[b][1][t]]
        z = [data[a][2][t], data[b][2][t]]
        edge[i] = ax.plot(x, y, z, color='grey', linestyle='dashed',
                          linewidth=trajectory_width*0.8)

    # Display target trajectory
    if target_trajectory is not None:
        ax.plot(target_trajectory[0], target_trajectory[1], target_trajectory[2],
                color='red', label='Target Trajectory',
                linewidth=trajectory_width, linestyle='--')

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
        ani.save(SavePath,fps=50,dpi=200)
    plt.close()

    print('3D Animation Done.')
    print('-' * 30)









# ----------------------------------------------------------------










plt.rcParams['font.family'] = 'serif'
plt.rcParams['font.serif'] = ['Times New Roman']
plt.rcParams['axes.unicode_minus']=False

# plt.rcParams['axes.titlesize'] = 18     # 坐标轴标题字体大小
# plt.rcParams['axes.labelsize'] = 17     # 坐标轴标签字体大小
# plt.rcParams['xtick.labelsize'] = 22    # x轴刻度字体大小
# plt.rcParams['ytick.labelsize'] = 22    # y轴刻度字体大小
# plt.rcParams['legend.fontsize'] = 25    # 图例大小

plt.rcParams['text.usetex'] = True 

# main
if __name__ == "__main__":  
    # fig parameter setup
    sim_time = 10
    data_label = 'paper2sim5'
    if_trajectory_tracking = True
    if_animation = False

    # define folder path
    from pathlib import Path
    folder_path = Path("./fig/"+data_label)
    folder_path.mkdir(parents=True, exist_ok=True)

    # read motion data
    trajectory_data = np.load('./log/'+data_label+'/trajectory.npy')
    distance_err = np.loadtxt('./log/'+data_label+'/distance_error.txt')
    orientation_err = np.loadtxt('./log/'+data_label+'/orientation_error.txt')
    edge_list = np.loadtxt('./log/'+data_label+'/edge_list.txt',dtype=int)

    if if_trajectory_tracking:
        tracking_err = np.loadtxt('./log/'+data_label+'/tracking_error.txt')
        target_trajectory = np.loadtxt('./log/'+data_label+'/target_trajectory.txt')

    # output fig name
    preffix = './fig/'+data_label+'/'
    suffix = ''

    # define ploting parameter
    color_list = ['blue','green','gold',
                  'darkgrey','darkgrey','darkgrey','darkgrey']

    label_list = ['Leader','Co-Leader 1','Co-Leader 2',
                  'Follower','x','x']

    drawtime_list = [3,6.5]

    # ploting

    # 1 trajectory
    Plot_Trajectory_3D('SI',
                        trajectory_data,
                        color_list,label_list,edge_list,drawtime_list,
                        trajectory_width=0.4,point_size=8,
                        target_trajectory=\
                            target_trajectory if if_trajectory_tracking else None,
                        pltrange_z = [-3,34],
                        if_label=True,
                        SavePath=preffix+'tra'+suffix+'.png' )

    # 2 distance error
    label_list = [ 'x' for a in range(len(distance_err)-1) ]
    label_list.append(r'$\|p_{ij}\|-d_{ij}$')
    print(label_list)
    plot_err(distance_err,
             xy_label=['Time (s)','Distance error (m)'],
             line_width=0.8,
             plt_label=label_list,
             pltrange_x=[0,sim_time],
             pltrange_y=[],
             SavePath=preffix+'err_d'+suffix+'.png')

    # 3 normalized error 
    sigma_list = np.array([])
    distance_err = np.array(distance_err)
    for i in range(len(distance_err[0])):
        err = distance_err[:,i]
        sigma_list = np.append(sigma_list, np.linalg.norm(err))
    sigma_list /= sigma_list[0]

    label = [r'$\frac{\| \sigma(t) \|}{\| \sigma(0) \|}$' ]
    print(label)
    plot_err([sigma_list],
             xy_label=['Time (s)','Normalized error'],
             line_width=0.8,
             plt_label=label,
             pltrange_x=[0,sim_time],
             pltrange_y=[-0.1,1.1],
             color_list=['blue'],
             SavePath=preffix+'err_n'+suffix+'.png')

    # 4 orientation error
    label = [ r'$ \| p_{o}-p_{o}^*(t)  \|$']
    print(label)
    plot_err([orientation_err],
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
        plot_err([tracking_err],
                xy_label=['Time (s)','Tracking error (m)'],
                line_width=0.8,
                plt_label=label,
                pltrange_x=[0,sim_time],
                pltrange_y=[],
                color_list=['blue'],
                SavePath=preffix+'err_t'+suffix+'.png')

    # 6 Animation
    if if_animation:
        Animation_motion_3D('SI',
                            trajectory_data,color_list,label_list,edge_list,
                            trajectory_width=0.6,
                            point_size=5,
                            target_trajectory=\
                                target_trajectory if if_trajectory_tracking else None,
                            Show=False,
                            SavePath=preffix+'ani'+suffix+'.gif')

