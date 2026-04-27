import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from ..config.map_hard import hard_grid, start_point_hard, end_point_hard
from .apf_basic import get_obstacles, calculate_potential

def get_neighbors(pos, grid):
    """获取周围4个方向的可通行邻居"""
    directions = [(-1,0), (1,0), (0,-1), (0,1)] # 上下左右四个方向
    neighbors = []
    for d in directions:
        nr, nc = pos[0] + d[0], pos[1] + d[1]
        if 0 <= nr < grid.shape[0] and 0 <= nc < grid.shape[1]:
            if grid[nr, nc] == 0:
                neighbors.append((nr, nc))
    return neighbors

def is_obstacle_adjacent(pos, grid):
    """判断周围是否有障碍物"""
    directions = [(-1,0), (1,0), (0,-1), (0,1)]
    for d in directions:
        nr, nc = pos[0] + d[0], pos[1] + d[1]
        if 0 <= nr < grid.shape[0] and 0 <= nc < grid.shape[1]:
            if grid[nr, nc] == 1:
                return True
    return False

def apf_improved_search(grid, start, end, max_iter=2000):
    """
    改进型 APF: 在陷入死锁时引入动态记忆惩罚（Tabu 填坑法则）
    """
    obs = get_obstacles(grid)
    path = [start]
    explored = [start]
    
    current = start
    visited_counts = {}
    
    for _ in range(max_iter):
        if current == end:
            print(">>> 成功到达终点！")
            return path, explored, True
            
        visited_counts[current] = visited_counts.get(current, 0) + 1
        
        best_next = current
        min_u = float('inf')
        
        neighbors = get_neighbors(current, grid)
        # 用带惩罚的势能去探查前路
        for neighbor in neighbors:
            # 基础势能
            u = calculate_potential(neighbor, end, obs)
            # 动态惩罚
            u += visited_counts.get(neighbor, 0) * 100.0
            
            if u < min_u:
                min_u = u
                best_next = neighbor
                
        current = best_next
        path.append(current)
        explored.append(current)
        
    print(">>> 超过最大迭代次数，寻路失败。")
    return path, explored, False

def visualize_improved_apf(grid, start, end, path, explored, success):
    cmap = ListedColormap(['white', 'blue'])
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.imshow(grid, cmap=cmap, origin='upper', extent=[0, grid.shape[1], grid.shape[0], 0])
    ax.set_xticks(np.arange(0, grid.shape[1], 1))
    ax.set_yticks(np.arange(0, grid.shape[0], 1))
    ax.grid(which='both', color='black', linestyle='-', linewidth=0.5)
    
    if path:
        path_y = [p[0] + 0.5 for p in path]
        path_x = [p[1] + 0.5 for p in path]
        color = 'cyan' if success else 'orange'
        ax.plot(path_x, path_y, marker='o', color=color, linewidth=2, markersize=5, label='Improved APF Path')
        
    ax.text(start[1] + 0.5, start[0] + 0.5, 'S', color='green', fontsize=18, ha='center', va='center', fontweight='bold')
    ax.text(end[1] + 0.5, end[0] + 0.5, 'E', color='red', fontsize=18, ha='center', va='center', fontweight='bold')
    
    plt.title("Improved APF (Wall-Following Recovery)")
    plt.legend(loc='lower left')
    plt.savefig('../../results/images/apf_improved_result.png')
    plt.show()

if __name__ == "__main__":
    print("运行改进型人工势场法(APF + 沿墙摸索脱困)...")
    path, explored, success = apf_improved_search(hard_grid, start_point_hard, end_point_hard)
    visualize_improved_apf(hard_grid, start_point_hard, end_point_hard, path, explored, success)
