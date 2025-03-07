# Package for Motion Animation

from matplotlib import pyplot as plt
import numpy as np
import matplotlib.patches as patches
from math import sqrt,cos,sin,atan2,atan,exp,tan,pi
from matplotlib.animation import FuncAnimation
import json
import os

# plt.rcParams.update({
#     "text.usetex": True,
#     "font.family": "Helvetica"
# })

plt.rcParams['font.family'] = 'serif'
plt.rcParams['font.serif'] = ['Times New Roman']
plt.rcParams['axes.unicode_minus']=False
plt.rcParams['text.usetex'] = True 

# plt.rcParams['axes.titlesize'] = 18     # 坐标轴标题字体大小
# plt.rcParams['axes.labelsize'] = 17     # 坐标轴标签字体大小
# plt.rcParams['xtick.labelsize'] = 22    # x轴刻度字体大小
# plt.rcParams['ytick.labelsize'] = 22    # y轴刻度字体大小
# plt.rcParams['legend.fontsize'] = 25    # 图例大小


# Unicycle Plotting and Animation ----------------------------------------------------------------

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


def Unicycle_Animation(data,color_list,label_list, # abandoned!!!
                       edge_list,
                       if_label=False,
                       trajectory_width = 0.8,
                       point_size=0.5,
                       Show=True,SavePath='ANI.gif'):
    '''
    Animation function for unicycle motion
    !! the data frequency must be 50Hz !!
    Input as follows:
    data--a list of agent data with 3 dimensions (agent-dimension-value along time)
    timestep-- 1/frequency(Hz)
    color_list--a list of color for each agent
    label_list--a list of label for each agent
    '''
    num_agent = len(data)
    dimension = len(data[0])
    total_step = len(data[0][0])

    # slice_ratio = int(1/timestep/50)
    # # slice the data
    # for i in range(num_agent):
    #     for j in range(dimension):
    #         data[i][j] = data[i][j][::slice_ratio]

    # print('Original Data Frequency: '+str(int(1/timestep)))
    # print('Animation Slice Ratio: '+str(int(slice_ratio)))
    # print('Animation Frequency: 50Hz')        
    # print('Trajectory data length: '+str(len(data[0][0])))

    fig, ax = plt.subplots(figsize=(8,6))
    ax.set_aspect(1)
    ax.set_xlabel('x/m')
    ax.set_ylabel('y/m')
    ax.legend()

    line=[ 0 for i in range(num_agent) ]
    body=[ 0 for i in range(num_agent) ]
    wheel=[ [0,0] for i in range(num_agent) ]
    edge = [ 0 for i in range(len(edge_list)) ]

    t=0
    for i in range(num_agent):
        # trajectory
        line[i]=ax.plot(data[i][0][:t],data[i][1][:t],\
                        color=color_list[i],label=label_list[i],linewidth=trajectory_width)
        # body
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
    
    for i in range(len(edge_list)): # connection line between agent
        a,b = edge_list[i]  
        x = [data[a][0][t],data[b][0][t]]
        y = [data[a][1][t],data[b][1][t]]
        edge[i]=ax.plot(x,y,color='grey',linestyle='dashed',
                linewidth=point_size)

    def update(t):
        # t += timestep   
        ax.legend()
        ax.set_title('Trajectory {:.1f}s'.format(t/50))
        x_ave,y_ave = 0,0

        for i in range(num_agent):
            line[i][0].set_data(data[i][0][:t],data[i][1][:t])
            x_ave += data[i][0][t]
            y_ave += data[i][1][t]

            body_point = body_points_unicycle([data[i][0][t],data[i][1][t],data[i][2][t]])
            body[i].set_xy(body_point)
            
            wheel_point_1,wheel_point_2 = wheel_points_unicycle(body_point,data[i][2][t])
            wheel[i][0].set_xy(wheel_point_1)
            wheel[i][1].set_xy(wheel_point_2)
        
        x_ave /= num_agent
        y_ave /= num_agent
        ax.set_xlim(x_ave-15,x_ave+15)
        ax.set_ylim(y_ave-15,y_ave+15)
        # plt.grid(True)

        for i in range(len(edge_list)):
            a,b = edge_list[i]  
            x = [data[a][0][t],data[b][0][t]]
            y = [data[a][1][t],data[b][1][t]]
            edge[i][0].set_data(x,y)

        return line,body,edge

    from matplotlib.animation import FuncAnimation
    ani=FuncAnimation(fig,update,interval=20,frames=int(total_step))
    if Show:
        plt.show()
    else:
        ani.save(SavePath)
    plt.close()

    print('animation done.')
    print('-'*30)



# def Unicycle_Trajectory(data,color_list,label_list, # abandoned!!!
#                         edge_list,drawtime_list,
#                         if_label=False,
#                         trajectory_width=0.8,
#                         point_size=0.5,
#                         Show=True,
#                         SavePath='TRA.png'):
#     '''
#     Trajectory Plotting function for unicycle motion
#     Input as follows:
#     data--a list of agent data with 3 dimensions (agent-dimension-value along time)
#     timestep-- 1/frequency(Hz)
#     color_list--a list of color for each agent
#     label_list--a list of label for each agent
#     '''
#     num_agent = len(data)
#     dimension = len(data[0])
#     total_step = len(data[0][0])

#     fig, ax = plt.subplots(figsize=(8,6))

#     # trajectory for each car
#     for i in range(num_agent):
#         # trajectory
#         ax.plot(data[i][0],data[i][1],\
#                 color=color_list[i],label=label_list[i],linewidth=trajectory_width)
    
#     # plot agent body in drawtime_list 
#     drawtime_list = [ int(x*50) for x in drawtime_list]
#     drawtime_list.append(0)
#     drawtime_list.append(-1)

#     for t in drawtime_list:
#         for i in range(num_agent):
#             body_point = body_points_unicycle([data[i][0][t],data[i][1][t],data[i][2][t]])
#             body = patches.Polygon(body_point, closed=True,lw = point_size, \
#                                 edgecolor=color_list[i], facecolor='none')
#             wheel_point_1,wheel_point_2 = wheel_points_unicycle(body_point,data[i][2][t])
#             wheel_1 = patches.Polygon(wheel_point_1, closed=True,lw = point_size, \
#                                     edgecolor=color_list[i], facecolor=color_list[i])
#             wheel_2 = patches.Polygon(wheel_point_2, closed=True,lw = point_size, \
#                                     edgecolor=color_list[i], facecolor=color_list[i])
#             ax.add_artist(body)
#             ax.add_artist(wheel_1)
#             ax.add_artist(wheel_2)
#         for a,b in edge_list: # connection line between agent
#             x = [data[a][0][t],data[b][0][t]]
#             y = [data[a][1][t],data[b][1][t]]
#             ax.plot(x,y,color='grey',linestyle='dashed',
#                     linewidth=trajectory_width)
        
#     # axe
#     ax = plt.gca()
#     ax.set_aspect(1)

#     # plt.title('Trajectory')
#     plt.xlabel('x[m]')
#     plt.ylabel('y[m]')
#     # plt.legend()
#     # plt.xlim(-1,2)
#     # plt.ylim(-1,1)

#     if Show:
#         plt.show()
#     else:
#         plt.savefig(SavePath,dpi=200)
#     plt.close()    

#     print('Trajectory Ploted.')
#     print('-'*30)













def Plot_Trajectory(agent_type,
                    data,
                    color_list,label_list,edge_list,drawtime_list,
                    offset=None,
                    target_trajectory=None,
                    trajectory_width=0.4,point_size=0.6,
                    pltrange_xy = [],
                    if_label = False,
                    Show=True,
                    SavePath='TRA.png'):

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
                    linewidth=trajectory_width)
        else:
            ax.plot(data[i][0],data[i][1],
                    color=color_list[i],linewidth=trajectory_width)
    
    # display target trajectory
    if target_trajectory is not None:
        ax.plot(target_trajectory[0], target_trajectory[1],
                color='red',label='Reference Trajectory',
                linewidth=trajectory_width,linestyle='--')

    # 2 plot agent body in drawtime_list 
    drawtime_list = [ int(x*50) for x in drawtime_list]
    drawtime_list.append(0)
    drawtime_list.append(-1)

    for t in drawtime_list:
        # display edge between agent
        for a,b in edge_list:
            if t==0:
                continue 
            if offset==None:
                x = [data[a][0][t],data[b][0][t]]
                y = [data[a][1][t],data[b][1][t]]
                ax.plot(x,y,color='grey',linestyle='dashed',
                        linewidth=point_size*0.6)
            else:
                x = [data[a][0][t]+offset*cos(data[a][2][t]),
                     data[b][0][t]+offset*cos(data[b][2][t])]
                y = [data[a][1][t]+offset*sin(data[a][2][t]),
                     data[b][1][t]+offset*sin(data[b][2][t])]
                ax.plot(x,y,color='grey',linestyle='dashed',
                        linewidth=point_size*0.6)
            
        # display agent body
        if agent_type=='SI':
            for i in range(num_agent):
                body = patches.Circle((data[i][0][t],data[i][1][t]),
                                    point_size,
                                    edgecolor=color_list[i],
                                    facecolor='snow')
                ax.add_artist(body)
        elif agent_type=='Unicycle':
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

        else:
            raise Exception('Unknown agent type.')
        
    # axe
    ax = plt.gca()
    ax.set_aspect(1)

    # plt.title('Trajectory')
    plt.xlabel('x')
    plt.ylabel('y')
    plt.tick_params(axis='both', direction='in',labelsize=12)

    if if_label:
        plt.legend(loc='upper right',edgecolor='black',
                   fontsize=10,fancybox=False,framealpha=1)
    
    if pltrange_xy != []:
        plt.xlim(pltrange_xy[0],pltrange_xy[1])
        plt.ylim(pltrange_xy[2],pltrange_xy[3])

    if Show:
        plt.show()
    else:
        plt.savefig(SavePath,dpi=200,bbox_inches='tight',pad_inches=0.1)
    plt.close()    

    print('MassPoint Trajectory Ploted.')
    print('-'*30)










def Animation_motion(agent_type,
                     data,color_list,label_list,
                     edge_list,
                     offset=None,
                     trajectory_width=0.6,
                     point_size=0.6,
                     target_trajectory = None,
                     if_label=False,
                     Show=True,
                     SavePath='ANI.gif'):
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
    if if_label:
        ax.legend(loc='upper right')

    line = [ 0 for i in range(num_agent) ]
    body = [ 0 for i in range(num_agent) ]
    edge = [ 0 for i in range(len(edge_list)) ]
    wheel= [ [0,0] for i in range(num_agent) ]

    t=0
    for i in range(num_agent):
        # display trajectory
        line[i]=ax.plot(data[i][0][:t],data[i][1][:t],\
                        color=color_list[i],label=label_list[i],
                        linewidth=trajectory_width)
        # display body
        if agent_type=='SI':
            body[i] = patches.Circle((data[i][0][t],data[i][1][t]),point_size,
                                edgecolor=color_list[i],facecolor='snow')
            ax.add_artist(body[i])
        elif agent_type=='Unicycle':
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
        else:
            raise Exception('Unknown agent type.')
            
    # display edge between agent
    for i in range(len(edge_list)): 
        a,b = edge_list[i]
        if offset==None:
            x = [data[a][0][t],data[b][0][t]]
            y = [data[a][1][t],data[b][1][t]]
            edge[i]=ax.plot(x,y,color='grey',linestyle='dashed',
                    linewidth=point_size*0.6)
        else:
            x = [data[a][0][t]+offset*cos(data[a][2][t]),
                    data[b][0][t]+offset*cos(data[b][2][t])]
            y = [data[a][1][t]+offset*sin(data[a][2][t]),
                    data[b][1][t]+offset*sin(data[b][2][t])]
            edge[i]=ax.plot(x,y,color='grey',linestyle='dashed',
                    linewidth=point_size*0.6)

    # display target trajectory
    if target_trajectory is not None:
        tt = ax.plot(target_trajectory[0], target_trajectory[1],
                color='red',label='Target Trajectory',
                linewidth=trajectory_width,linestyle='--')

    def update(t):
        # t += timestep  
        if if_label: 
            ax.legend(loc='upper right')
            
        ax.set_title('t = {:.1f} s'.format(t/50))
        x_ave,y_ave = 0,0

        for i in range(num_agent):
            # update trajectory
            line[i][0].set_data(data[i][0][:t],data[i][1][:t])
            x_ave += data[i][0][t]
            y_ave += data[i][1][t]
            
            # update body
            if agent_type=='SI':
                body[i].set_center((data[i][0][t],data[i][1][t]))
            elif agent_type =='Unicycle':
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
            if offset==None:
                x = [data[a][0][t],data[b][0][t]]
                y = [data[a][1][t],data[b][1][t]]
            else:
                x = [data[a][0][t]+offset*cos(data[a][2][t]),
                    data[b][0][t]+offset*cos(data[b][2][t])]
                y = [data[a][1][t]+offset*sin(data[a][2][t]),
                     data[b][1][t]+offset*sin(data[b][2][t])]
            edge[i][0].set_data(x,y)
        
        if agent_type=='SI':
            if target_trajectory is not None:
                return line,body,edge,tt
            else:
                return line,body,edge
        elif agent_type=='Unicycle':
            if target_trajectory is not None:
                return line,body,edge,tt,wheel
            else:
                return line,body,edge,wheel

    
    ani=FuncAnimation(fig,update,interval=20,frames=int(total_step))
    if Show:
        plt.show()
    else:
        ani.save(SavePath,fps=50,dpi=150)
    plt.close()

    print('animation done.')
    print('-'*30)









def plot_err( err_list,
              xy_label,
              line_width,
              plt_label,
              pltrange_x=[],
              pltrange_y=[],
              color_list=[],
              SavePath=None):
    
    dpi = 200 
    tlist = np.linspace(0, len(err_list[0])*(1/50), len(err_list[0]))
    sim_time = int((len(err_list[0])-1)/50)

    plt.figure(figsize=(500/dpi,500/dpi))

    for i in range(len(err_list)):
        # check if [color] or [label]
        if color_list != []:
            if plt_label[i] == 'x':
                plt.plot(tlist, err_list[i], color=color_list[i], 
                         linewidth=line_width)
            else:
                plt.plot(tlist, err_list[i], color=color_list[i], 
                         label=plt_label[i], linewidth=line_width)
        else: # no color requirement
            if plt_label[i] == 'x':
                plt.plot(tlist, err_list[i], linewidth=line_width) 
            else:
                plt.plot(tlist, err_list[i], label=plt_label[i], 
                         linewidth=line_width)

    plt.ylabel(xy_label[1])
    plt.xlabel(xy_label[0])

    if xy_label[1] == 'Normalized error':
        plt.yticks([0,1])
    # elif xy_label[1] == 'Distance error':
    else:
        max_y = max(max(row) for row in err_list)
        m = int(max_y // 2)
        plt.yticks(range(0,m*2+2,2))
    
    if sim_time < 16:
        plt.xticks(range(0,sim_time+2,2))
    else:
        plt.xticks(range(0,sim_time+4,4))

    # plt.grid(True, linestyle='--')
    plt.tick_params(axis='both', direction='in')
    plt.legend(loc='upper right',edgecolor='black',
               fancybox=False,framealpha=1)

    if pltrange_x != []:
        plt.xlim(pltrange_x[0], pltrange_x[1])
    if pltrange_y != []:
        plt.ylim(pltrange_y[0], pltrange_y[1])   

    plt.savefig(SavePath,dpi=dpi,bbox_inches='tight',pad_inches=0.1)
    # plt.show()
    plt.close()   

    print(xy_label[1]+' Ploted.')
    print('-'*30)
    









# ----------------------------------------------------------------

# ----------------------------------------------------------------

# ----------------------------------------------------------------










# main
# if __name__ == "__main__":  
#     # [ fig parameter setup ] ------------------------------
#     sim_time = 10
#     data_label = 'demo1'
#     if_trajectory_tracking = False
#     if_animation = True
#     # --------------------------------------------------------

#     # create folder for figure
#     folder_path = f"./fig/{data_label}"
#     os.makedirs(folder_path, exist_ok=True) 

#     # read motion data
#     with open(f'./log/{data_label}.json', "r") as f:
#         data = json.load(f)  

#     # output fig name
#     preffix = './fig/'+data_label+'/'
#     suffix = ''

#     # define ploting parameter
#     color_list = ['blue','green',
#                   'darkgrey','darkgrey','darkgrey','darkgrey']

#     label_list = ['Leader','Co-Leader','Followers','x','x','x']

#     drawtime_list = [3,6.14]

#     # ploting

#     # 1 trajectory
#     Plot_Trajectory('SI',
#                     data['trajectory'],
#                     color_list,label_list,data['edge_list'],drawtime_list,
#                     # trajectory_width=0.6,point_size=0.6,
#                     target_trajectory=\
#                         data['target_trajectory'] if if_trajectory_tracking else None,
#                     pltrange_xy = [-19,64,-14,29],
#                     if_label=True,
#                     Show=False,
#                     SavePath=preffix+'tra'+suffix+'.png' )

#     # 2 distance error
#     label_list = [ 'x' for a in range(len(data['distance_error'])-1) ]
#     label_list.append(r'$\|p_{ij}\|-d_{ij}$')
#     print(label_list)

#     plot_err(data['distance_error'],
#              xy_label=['Time (s)','Distance error'],
#              line_width=0.8,
#              plt_label=label_list,
#              pltrange_x=[0,sim_time],
#              pltrange_y=[],
#              color_list=[],
#              SavePath=preffix+'err_d'+suffix+'.png')

#     # 3 normalized error 
#     sigma_list = np.array([])
#     distance_err = np.array(data['distance_error'])
#     for i in range(len(distance_err[0])):
#         err = distance_err[:,i]
#         sigma_list = np.append(sigma_list, np.linalg.norm(err))
#     sigma_list /= sigma_list[0]

#     label = [r'$\frac{\|\sigma(t)\|}{\|\sigma(0)\|}$' ]
#     print(label)
#     plot_err([sigma_list],
#              xy_label=['Time (s)','Normalized error'],
#              line_width=0.8,
#              plt_label=label,
#              pltrange_x=[0,sim_time],
#              pltrange_y=[-0.1,1.1],
#              color_list=['blue'],
#              SavePath=preffix+'err_n'+suffix+'.png')

#     # 4 orientation error
#     label = [r'$\left\| p_o-p^*_o(t) \right\|$']
#     print(label)
#     plot_err([data['orientation_error']],
#              xy_label=['Time (s)','Orientation error'],
#              line_width=0.8,
#              plt_label=label,
#              pltrange_x=[0,sim_time],
#              pltrange_y=[],
#              color_list=['blue'],
#              SavePath=preffix+'err_o'+suffix+'.png')
    
#     # 5 tracking error
#     if if_trajectory_tracking:
#         label = [r'$\| p_1-p^*(t) \|$']
#         print(label)
#         plot_err(data['tracking_error'],
#                 xy_label=['Time (s)','Tracking error'],
#                 line_width=0.8,
#                 plt_label=label,
#                 pltrange_x=[0,sim_time],
#                 pltrange_y=[],
#                 color_list=['blue'],
#                 SavePath=preffix+'err_t'+suffix+'.png')

#     # 6 Animation
#     if if_animation:
#         Animation_motion('SI',
#                         data['trajectory'],color_list,label_list,data['edge_list'],
#                         trajectory_width=0.6,
#                         point_size=0.6,
#                         target_trajectory=\
#                             data['target_trajectory'] if if_trajectory_tracking else None,
#                         Show=False,
#                         SavePath=preffix+'ani'+suffix+'.gif')

#     print('All figs done.')
