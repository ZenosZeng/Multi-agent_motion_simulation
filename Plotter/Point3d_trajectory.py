from mpl_toolkits.mplot3d import Axes3D
from matplotlib import pyplot as plt

def Point3d_trajectory( data,
                        color_list, label_list, edge_list, drawtime_list,
                        save_path,
                        target_trajectory=None,
                        trajectory_width=1,point_size=20,edge_width=1,
                        x_range=[],y_range=[],z_range=[],):

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
                color='purple', label='参考轨迹',
                linewidth=trajectory_width*1.2, linestyle='--')

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
            ax.plot(x, y, z, color='gray', linestyle='dashed',
                    linewidth=edge_width, alpha=0.6)

        # 显示 agent 的身体
        for i in range(num_agent):
            body = ax.scatter(data[i][0][t], data[i][1][t], data[i][2][t],
                                color='snow',
                                s=point_size, 
                                edgecolors=color_list[i])

    # 5. 设置轴标签和范围
    ax.set_aspect('equal')
    ax.set_xlabel(r'$x$',fontsize=12)
    ax.set_ylabel(r'$y$',fontsize=12)
    ax.set_zlabel(r'$z$',fontsize=12)

    ax.tick_params(axis='both', direction='in',)

    # ax.set_zticks([-5,0,5,10,15])
    # ax.set_yticks([0,10,20,30])
    ax.legend(edgecolor='black',fancybox=False,framealpha=1,loc='upper left')

    if x_range!=[]:
        ax.set_xlim(x_range[0], x_range[1])
    if y_range!=[]:
        ax.set_ylim(y_range[0], y_range[1])
    if z_range!=[]:
        ax.set_zlim(z_range[0], z_range[1])

    ax.view_init(elev=22, azim=-58)

    plt.tight_layout()
    plt.savefig(save_path,dpi=300,bbox_inches='tight', pad_inches=0.3) 
    plt.close()

    print('MassPoint 3D Trajectory Plotted.')
    print('-' * 30)