import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import ListedColormap
import heapq
from ..config.map_basic import mock_grid, start_point, end_point

def heuristic(a, b):
    # 曼哈顿距离 (Manhattan distance)
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def a_star_search(grid, start, end):
    rows, cols = grid.shape
    # 四个移动方向：上下左右 (每步代价为1)
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)] # 八方向（包含对角线）
    
    # 优先队列 open_list, 保存记录形式：(f_score, g_score, (row, col))
    open_list = []
    heapq.heappush(open_list, (0 + heuristic(start, end), 0, start))
    
    came_from = {}
    g_score = {start: 0}
    visited = set()
    
    explored_nodes = [] # 用于记录探索顺序
    
    while open_list:
        _, current_g, current = heapq.heappop(open_list)
        
        # 记录已探索点
        if current not in explored_nodes:
            explored_nodes.append(current)
            
        # 到达终点
        if current == end:
            path = []
            curr = current
            while curr in came_from:
                path.append(curr)
                curr = came_from[curr]
            path.append(start)
            path.reverse()
            return path, explored_nodes
            
        if current in visited:
            continue
        visited.add(current)
        
        for d in directions:
            neighbor = (current[0] + d[0], current[1] + d[1])
            
            # 边界检查
            if 0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols:
                # 避障检查
                if grid[neighbor[0], neighbor[1]] == 1:
                    continue
                    
                tentative_g = current_g + 1
                
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f = tentative_g + heuristic(neighbor, end)
                    heapq.heappush(open_list, (f, tentative_g, neighbor))
                    
    return None, explored_nodes

def visualize_astar(grid, start, end, path, explored):
    """
    可视化地图、探索的节点以及最终的最短路径
    """
    cmap = ListedColormap(['white', 'blue'])
    
    fig, ax = plt.subplots(figsize=(6, 6))
    ax.imshow(grid, cmap=cmap, origin='upper', extent=[0, grid.shape[1], grid.shape[0], 0])
    
    ax.set_xticks(np.arange(0, grid.shape[1], 1))
    ax.set_yticks(np.arange(0, grid.shape[0], 1))
    ax.grid(which='both', color='black', linestyle='-', linewidth=1)
    
    # 画探索过的节点 (浅灰色或浅黄色)
    for node in explored:
        if node != start and node != end:
            ax.add_patch(plt.Rectangle((node[1], node[0]), 1, 1, color='yellow', alpha=0.4))
            
    # 如果有路径，画出路径
    if path:
        path_y = [p[0] + 0.5 for p in path]
        path_x = [p[1] + 0.5 for p in path]
        ax.plot(path_x, path_y, marker='o', color='magenta', linewidth=3, markersize=6, label='A* Path')
    
    # 标注起点 S 和终点 E
    ax.text(start[1] + 0.5, start[0] + 0.5, 'S', color='green', 
            fontsize=20, ha='center', va='center', fontweight='bold')
    ax.text(end[1] + 0.5, end[0] + 0.5, 'E', color='red', 
            fontsize=20, ha='center', va='center', fontweight='bold')
    
    plt.title("A* Algorithm Pathfinding")
    plt.legend(loc='upper right')
    
    plt.savefig('../../results/images/astar_result.png')
    print("A* 搜索结果图片已保存至: results/images/astar_result.png")
    plt.show()

if __name__ == "__main__":
    print(f"起点: {start_point}, 终点: {end_point}")
    path, explored = a_star_search(mock_grid, start_point, end_point)
    
    if path:
        print(f"寻路成功！找到最短路径，步数：{len(path)-1}")
        print("路径坐标:", path)
    else:
        print("未找到到达终点的路径！")
        
    visualize_astar(mock_grid, start_point, end_point, path, explored)