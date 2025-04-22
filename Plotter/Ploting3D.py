# Package for Motion Animation
import numpy as np
from math import sqrt,cos,sin,atan2,atan,exp,tan,pi

from matplotlib import pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.patches as patches

from Ploting import plot_err






























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
# if __name__ == "__main__":  
#     # fig parameter setup
#     sim_time = 10
#     data_label = 'paper2sim5'
#     if_trajectory_tracking = True
#     if_animation = False

#     # define folder path
#     from pathlib import Path
#     folder_path = Path("./fig/"+data_label)
#     folder_path.mkdir(parents=True, exist_ok=True)

#     # read motion data
#     trajectory_data = np.load('./log/'+data_label+'/trajectory.npy')
#     distance_err = np.loadtxt('./log/'+data_label+'/distance_error.txt')
#     orientation_err = np.loadtxt('./log/'+data_label+'/orientation_error.txt')
#     edge_list = np.loadtxt('./log/'+data_label+'/edge_list.txt',dtype=int)

#     if if_trajectory_tracking:
#         tracking_err = np.loadtxt('./log/'+data_label+'/tracking_error.txt')
#         target_trajectory = np.loadtxt('./log/'+data_label+'/target_trajectory.txt')

#     # output fig name
#     preffix = './fig/'+data_label+'/'
#     suffix = ''

#     # define ploting parameter
#     color_list = ['blue','green','gold',
#                   'darkgrey','darkgrey','darkgrey','darkgrey']

#     label_list = ['Leader','Co-Leader 1','Co-Leader 2',
#                   'Follower','x','x']

#     drawtime_list = [3,6.5]

#     # ploting

#     # 1 trajectory
#     Plot_Trajectory_3D('SI',
#                         trajectory_data,
#                         color_list,label_list,edge_list,drawtime_list,
#                         trajectory_width=0.4,point_size=8,
#                         target_trajectory=\
#                             target_trajectory if if_trajectory_tracking else None,
#                         pltrange_z = [-3,34],
#                         if_label=True,
#                         SavePath=preffix+'tra'+suffix+'.png' )

#     # 2 distance error
#     label_list = [ 'x' for a in range(len(distance_err)-1) ]
#     label_list.append(r'$\|p_{ij}\|-d_{ij}$')
#     print(label_list)
#     plot_err(distance_err,
#              xy_label=['Time (s)','Distance error (m)'],
#              line_width=0.8,
#              plt_label=label_list,
#              pltrange_x=[0,sim_time],
#              pltrange_y=[],
#              SavePath=preffix+'err_d'+suffix+'.png')

#     # 3 normalized error 
#     sigma_list = np.array([])
#     distance_err = np.array(distance_err)
#     for i in range(len(distance_err[0])):
#         err = distance_err[:,i]
#         sigma_list = np.append(sigma_list, np.linalg.norm(err))
#     sigma_list /= sigma_list[0]

#     label = [r'$\frac{\| \sigma(t) \|}{\| \sigma(0) \|}$' ]
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
#     label = [ r'$ \| p_{o}-p_{o}^*(t)  \|$']
#     print(label)
#     plot_err([orientation_err],
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
#         plot_err([tracking_err],
#                 xy_label=['Time (s)','Tracking error (m)'],
#                 line_width=0.8,
#                 plt_label=label,
#                 pltrange_x=[0,sim_time],
#                 pltrange_y=[],
#                 color_list=['blue'],
#                 SavePath=preffix+'err_t'+suffix+'.png')

#     # 6 Animation
#     if if_animation:
#         Animation_motion_3D('SI',
#                             trajectory_data,color_list,label_list,edge_list,
#                             trajectory_width=0.6,
#                             point_size=5,
#                             target_trajectory=\
#                                 target_trajectory if if_trajectory_tracking else None,
#                             Show=False,
#                             SavePath=preffix+'ani'+suffix+'.gif')

