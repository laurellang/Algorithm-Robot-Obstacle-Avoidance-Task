"""
Visualization utilities for robot path planning algorithms.
"""
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import ListedColormap


def visualize_map(grid, start, end, title="Robot Navigation Map"):
    """
    可视化地图矩阵：
    - 0: 可行区域 (白色)
    - 1: 障碍物区域 (蓝色)
    - start: 起点 S (绿色)
    - end: 终点 E (红色)
    
    Args:
        grid: numpy array representing the grid
        start: tuple (row, col) for start position
        end: tuple (row, col) for end position
        title: title for the plot
    """
    cmap = ListedColormap(['white', 'blue'])
    
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.imshow(grid, cmap=cmap, origin='upper', extent=[0, grid.shape[1], grid.shape[0], 0])
    
    # 绘制网格线
    ax.set_xticks(np.arange(0, grid.shape[1], 1))
    ax.set_yticks(np.arange(0, grid.shape[0], 1))
    ax.grid(which='both', color='black', linestyle='-', linewidth=1)
    
    # 标注起点 S 和终点 E
    ax.text(start[1] + 0.5, start[0] + 0.5, 'S', color='green', 
            fontsize=20, ha='center', va='center', fontweight='bold')
    
    ax.text(end[1] + 0.5, end[0] + 0.5, 'E', color='red', 
            fontsize=20, ha='center', va='center', fontweight='bold')
    
    ax.set_title(title, fontsize=14, fontweight='bold')
    ax.set_xlabel('Column')
    ax.set_ylabel('Row')
    
    return fig, ax


def visualize_search_result(grid, start, end, path, explored, algorithm_name, output_file=None):
    """
    可视化搜索算法的结果，包括探索的节点和最终路径
    
    Args:
        grid: numpy array representing the grid
        start: tuple (row, col) for start position
        end: tuple (row, col) for end position
        path: list of tuples representing the path
        explored: list of tuples representing explored nodes
        algorithm_name: string name of the algorithm
        output_file: optional filename to save the result
    """
    cmap = ListedColormap(['white', 'blue'])
    
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.imshow(grid, cmap=cmap, origin='upper', extent=[0, grid.shape[1], grid.shape[0], 0])
    
    ax.set_xticks(np.arange(0, grid.shape[1], 1))
    ax.set_yticks(np.arange(0, grid.shape[0], 1))
    ax.grid(which='both', color='black', linestyle='-', linewidth=1)
    
    # 画探索过的节点 (浅黄色)
    for node in explored:
        if node != start and node != end:
            ax.add_patch(plt.Rectangle((node[1], node[0]), 1, 1, color='yellow', alpha=0.4))
    
    # 画路径
    if path:
        path_y = [p[0] + 0.5 for p in path]
        path_x = [p[1] + 0.5 for p in path]
        ax.plot(path_x, path_y, marker='o', color='magenta', linewidth=3, markersize=6, label='Path')
    
    # 标注起点和终点
    ax.text(start[1] + 0.5, start[0] + 0.5, 'S', color='green', 
            fontsize=20, ha='center', va='center', fontweight='bold')
    ax.text(end[1] + 0.5, end[0] + 0.5, 'E', color='red', 
            fontsize=20, ha='center', va='center', fontweight='bold')
    
    title = f"{algorithm_name} - Path Length: {len(path)-1 if path else 'N/A'} steps"
    ax.set_title(title, fontsize=14, fontweight='bold')
    ax.set_xlabel('Column')
    ax.set_ylabel('Row')
    ax.legend(loc='upper right')
    
    if output_file:
        plt.savefig(output_file, dpi=150, bbox_inches='tight')
        print(f"Result saved to: {output_file}")
    
    return fig, ax
