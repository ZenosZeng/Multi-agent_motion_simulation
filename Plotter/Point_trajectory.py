from matplotlib import pyplot as plt
import numpy as np
import matplotlib.patches as patches

plt.rcParams['font.family'] = 'serif'
plt.rcParams['font.serif'] = ['Times New Roman']
plt.rcParams['axes.unicode_minus']=False
plt.rcParams['text.usetex'] = True 

def Point_trajectory(data,
                    color_list,label_list,edge_list,drawtime_list,
                    save_path,
                    target_trajectory=None,
                    trajectory_width=1,point_size=0.8,edge_width=0.5,
                    xy_range = [],
                    Show=True):

    num_agent = len(data)
    dimension = len(data[0])
    total_step = len(data[0][0])

    fig, ax = plt.subplots() # figsize=(1920/200,1080/200)
    dpi=300

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
                color='purple',label='Reference Trajectory',
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
            else:
                x = [data[a][0][t],data[b][0][t]]
                y = [data[a][1][t],data[b][1][t]]
                ax.plot(x,y,color='gray',linestyle='dashed',
                        linewidth=edge_width, alpha=0.6)
           
            
        # display SI body
        for i in range(num_agent):
            body = patches.Circle((data[i][0][t],data[i][1][t]),
                                point_size,
                                edgecolor=color_list[i],
                                facecolor='snow')
            ax.add_artist(body)
        
    # axe
    ax = plt.gca()
    ax.set_aspect(1)

    # plt.title('Trajectory')
    plt.xlabel('x')
    plt.ylabel('y')
    plt.tick_params(axis='both', direction='in',labelsize=12)

    plt.legend(fontsize=10, loc='best', frameon=True, edgecolor='black')
    
    if xy_range != []:
        plt.xlim(xy_range[0],xy_range[1])
        plt.ylim(xy_range[2],xy_range[3])

    if Show:
        plt.show()
    else:
        plt.savefig(save_path,dpi=dpi,bbox_inches='tight',pad_inches=0.1)
    plt.close()    

    print('MassPoint Trajectory Ploted.')
    print('-'*30)