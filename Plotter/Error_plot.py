# default data frequency: 50Hz
from matplotlib import pyplot as plt
import numpy as np

plt.rcParams['font.family'] = 'sans-serif'
plt.rcParams['font.sans-serif'] = ['simsun']   # 使用宋体
plt.rcParams['axes.unicode_minus'] = False     # 解决负号显示为方块的问题
plt.rcParams['text.usetex'] = False

def Error_plot( err_list, # 2-d list
                xy_label,
                plt_label, # list of label , 'x' for no label
                save_path,
                line_width=1.5,
                x_range=[],y_range=[],
                color_list=[] 
                ):
    
    dpi = 300 
    tlist = np.linspace(0, len(err_list[0])*(1/50), len(err_list[0]))
    sim_time = int((len(err_list[0])-1)/50)

    plt.figure(figsize=(4,3)) 

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

    plt.ylabel(xy_label[1],fontsize=14, fontweight='bold')
    plt.xlabel(xy_label[0],fontsize=14, fontweight='bold')

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
    plt.tick_params(axis='both', which='major', labelsize=12)
    # plt.legend(loc='upper right',edgecolor='black',
    #            fancybox=False,framealpha=1)
    plt.legend(fontsize=14, loc='best', frameon=True, edgecolor='black')

    if x_range != []:
        plt.xlim(x_range[0],x_range[1])
    if y_range != []:
        plt.ylim(y_range[0],y_range[1])

    plt.savefig(save_path,dpi=dpi, bbox_inches='tight', pad_inches=0.1)
    # plt.tight_layout()
    plt.close()

    print(xy_label[1]+' Ploted.')
    print('-'*30)
