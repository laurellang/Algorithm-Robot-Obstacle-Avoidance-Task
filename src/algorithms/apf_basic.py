import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from ..config.map_hard import hard_grid, start_point_hard, end_point_hard

def get_obstacles(grid):
    obs = []
    for r in range(grid.shape[0]):
        for c in range(grid.shape[1]):
            if grid[r, c] == 1:
                obs.append((r, c))
    return obs

def calculate_potential(pos, end, obs, k_att=1.0, k_rep=20.0, rho_0=3.0):
    """
    计算给定位置的总势能 U = U_att + U_rep
    """
    # 引力势能 U_att = 0.5 * k_att * distance(pos, end)^2
    d_goal = np.sqrt((pos[0] - end[0])**2 + (pos[1] - end[1])**2)
    u_att = 0.5 * k_att * (d_goal ** 2)
    
    u_rep = 0.0
    # 斥力势能：距离小于影响半径 rho_0 才产生斥力
    for ob in obs:
        d_obs = np.sqrt((pos[0] - ob[0])**2 + (pos[1] - ob[1])**2)
        if d_obs > 0 and d_obs <= rho_0:
            u_rep += 0.5 * k_rep * ((1.0 / d_obs - 1.0 / rho_0) ** 2)
    
    return u_att + u_rep

def apf_basic_search(grid, start, end, max_iter=200):
    """
    传统人工势场法（离散网格版本）
    机器人在每一步观察周围 8 个邻居，向势能最低的走。
    """
    obs = get_obstacles(grid)
    path = [start]
    explored = [start]
    
    current = start
    directions = [(-1,0), (1,0), (0,-1), (0,1), (-1,-1), (-1,1), (1,-1), (1,1)] # 八方向
    
    for _ in range(max_iter):
        if current == end:
            print(">>> 到达终点！")
            return path, explored, True
            
        best_next = current
        min_u = calculate_potential(current, end, obs)
        
        # 遍历附近可走的格子寻找势能更小的邻居
        found_better = False
        for d in directions:
            nr, nc = current[0] + d[0], current[1] + d[1]
            # 边界及障碍检查
            if 0 <= nr < grid.shape[0] and 0 <= nc < grid.shape[1] and grid[nr, nc] == 0:
                neighbor = (nr, nc)
                u = calculate_potential(neighbor, end, obs)
                explored.append(neighbor)
                
                # 如果严格小于当前势能，则接受
                if u < min_u:
                    min_u = u
                    best_next = neighbor
                    found_better = True
        
        if not found_better:
            # 没有找到势能更低的点 -> 陷入了局部极小值！
            print(f">>> 陷入局部极小值 (Deadlock) 在坐标: {current}！无法继续前进。")
            return path, explored, False
            
        current = best_next
        path.append(current)
        
    print(">>> 超过最大迭代次数。")
    return path, explored, False

def visualize_apf(grid, start, end, path, explored, success):
    obs = get_obstacles(grid)
    # 计算整个地图的势能场分布
    potential_grid = np.zeros_like(grid, dtype=float)
    valid_potentials = []
    
    for r in range(grid.shape[0]):
        for c in range(grid.shape[1]):
            if grid[r, c] == 0:
                u = calculate_potential((r, c), end, obs)
                potential_grid[r, c] = u
                valid_potentials.append(u)
                
    # 为了可视化效果，设定一个合理的势能上限，避免障碍物周围极大的斥力使颜色比例完全失衡
    vmax = np.percentile(valid_potentials, 95) 
    
    # 针对障碍物赋予极高势能以便在图中凸显出极值
    for r in range(grid.shape[0]):
        for c in range(grid.shape[1]):
            if grid[r, c] == 1:
                potential_grid[r, c] = vmax * 1.1
                
    fig, ax = plt.subplots(figsize=(8, 8))
    
    # 绘制势能场热力图 (Heatmap: viridis越黄势能越高，越紫势能越低)
    cax = ax.imshow(potential_grid, cmap='viridis', origin='upper', 
                    extent=[0, grid.shape[1], grid.shape[0], 0], vmax=vmax)
    fig.colorbar(cax, shrink=0.8, label='Total Potential Energy Field')
    
    ax.set_xticks(np.arange(0, grid.shape[1], 1))
    ax.set_yticks(np.arange(0, grid.shape[0], 1))
    ax.grid(which='both', color='white', linestyle='-', linewidth=0.5, alpha=0.3)
    
    # 叠加半透明黑色障碍物标记，使地形依然清晰
    for r in range(grid.shape[0]):
        for c in range(grid.shape[1]):
            if grid[r, c] == 1:
                ax.add_patch(plt.Rectangle((c, r), 1, 1, facecolor='black', alpha=0.6))
    
    # 绘制机器人实际走的路径
    if path:
        path_y = [p[0] + 0.5 for p in path]
        path_x = [p[1] + 0.5 for p in path]
        color = 'cyan' if success else 'orange' # 成功则青色，失败橙色
        ax.plot(path_x, path_y, marker='o', color=color, linewidth=2, markersize=5, label='APF Path (Gradient Descent)')
        
        # 失败终点用红叉号标出卡死的“局部极小值盆地”位置
        if not success:
            ax.plot(path_x[-1], path_y[-1], marker='X', color='red', markersize=15, label='Deadlock / Local Min Basin')
            
    ax.text(start[1] + 0.5, start[0] + 0.5, 'S', color='lightgreen', fontsize=18, ha='center', va='center', fontweight='bold')
    ax.text(end[1] + 0.5, end[0] + 0.5, 'E', color='red', fontsize=18, ha='center', va='center', fontweight='bold')
    
    plt.title("APF Navigation & Potential Field Heatmap")
    plt.legend(loc='lower left')
    plt.savefig('../../results/images/apf_basic_result.png')
    plt.show()

if __name__ == "__main__":
    print("运行传统人工势场法(APF)...")
    path, explored, success = apf_basic_search(hard_grid, start_point_hard, end_point_hard)
    visualize_apf(hard_grid, start_point_hard, end_point_hard, path, explored, success)
