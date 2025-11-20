import numpy as np
import matplotlib.pyplot as plt

# 生成数据
x = np.linspace(0, 10, 100)
y1 = np.sin(x)
y2 = np.cos(x)

# 设置全局参数
plt.rcParams['font.family'] = 'Times New Roman'  # 统一字体
plt.rcParams['axes.linewidth'] = 1.2  # 坐标轴加粗
plt.rcParams['xtick.direction'] = 'in'  # 刻度朝内
plt.rcParams['ytick.direction'] = 'in'
plt.rcParams['xtick.major.size'] = 5  # 主刻度长度
plt.rcParams['ytick.major.size'] = 5
plt.rcParams['legend.frameon'] = False  # 去掉图例边框

# 创建图表
fig, ax = plt.subplots(figsize=(6, 4), dpi=300)  # 适合论文的尺寸

# 绘制曲线
ax.plot(x, y1, linestyle='-', color='black', linewidth=1.5, label=r'$\sin(x)$')  # 细实线
ax.plot(x, y2, linestyle='--', color='black', linewidth=1.5, label=r'$\cos(x)$')  # 虚线

# 坐标轴设置
ax.set_xlabel(r'$x$', fontsize=14, fontweight='bold')  # x 轴标签
ax.set_ylabel(r'$y$', fontsize=14, fontweight='bold')  # y 轴标签
ax.tick_params(axis='both', which='major', labelsize=12)  # 坐标轴刻度大小
ax.spines['top'].set_visible(False)  # 隐藏上边框
ax.spines['right'].set_visible(False)  # 隐藏右边框

# 图例
ax.legend(fontsize=12, loc='best', edgecolor='black')

# 保存高分辨率图像
plt.tight_layout()
# plt.savefig('sci_style_plot.pdf', format='pdf', dpi=300, bbox_inches='tight')
plt.show()
