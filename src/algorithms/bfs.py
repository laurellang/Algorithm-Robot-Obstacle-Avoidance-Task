import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import ListedColormap
from collections import deque
from ..config.map_basic import mock_grid, start_point, end_point

def bfs_search(grid, start, end):
    rows, cols = grid.shape
    # 四个移动方向：上下左右
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)] # 八方向（包含对角线）
    
    # 使用双端队列 deque 实现先进先出 (FIFO) 的结构
    queue = deque([start])
    
    # 用集合记录已经访问过的节点，防止重复访问
    visited = set([start])
    
    # 用于记录前驱节点，方便找到目标后回溯路径
    came_from = {start: None}
    
    # 用于记录探索顺序，供后续可视化展示
    explored_nodes = []
    
    while queue:
        current = queue.popleft()
        
        # 记录已探索点 (如果尚未记录)
        if current not in explored_nodes:
            explored_nodes.append(current)
            
        # 如果当前节点就是终点，回溯出完整路径
        if current == end:
            path = []
            curr = current
            while curr is not None:
                path.append(curr)
                curr = came_from[curr]
            path.reverse()
            return path, explored_nodes
            
        # 遍历四个方向的邻居
        for d in directions:
            neighbor = (current[0] + d[0], current[1] + d[1])
            
            # 边界检查
            if 0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols:
                # 避障检查 (grid值为1处为障碍物) 以及是否已经访问过的检查
                if grid[neighbor[0], neighbor[1]] == 0 and neighbor not in visited:
                    visited.add(neighbor)
                    came_from[neighbor] = current
                    queue.append(neighbor)
                    
    # 若遍历完整个连通区域仍找不到终点，则返回 None
    return None, explored_nodes

def visualize_bfs(grid, start, end, path, explored):
    """
    可视化地图、探索的节点以及最终的 BFS 最短路径
    """
    cmap = ListedColormap(['white', 'blue'])
    
    fig, ax = plt.subplots(figsize=(6, 6))
    ax.imshow(grid, cmap=cmap, origin='upper', extent=[0, grid.shape[1], grid.shape[0], 0])
    
    ax.set_xticks(np.arange(0, grid.shape[1], 1))
    ax.set_yticks(np.arange(0, grid.shape[0], 1))
    ax.grid(which='both', color='black', linestyle='-', linewidth=1)
    
    # 画探索过的节点 (浅黄色)
    for node in explored:
        if node != start and node != end:
            ax.add_patch(plt.Rectangle((node[1], node[0]), 1, 1, color='yellow', alpha=0.4))
            
    # 如果有路径，画出路径
    if path:
        path_y = [p[0] + 0.5 for p in path]
        path_x = [p[1] + 0.5 for p in path]
        ax.plot(path_x, path_y, marker='o', color='cyan', linewidth=3, markersize=6, label='BFS Path')
    
    # 标注起点 S 和终点 E
    ax.text(start[1] + 0.5, start[0] + 0.5, 'S', color='green', 
            fontsize=20, ha='center', va='center', fontweight='bold')
    ax.text(end[1] + 0.5, end[0] + 0.5, 'E', color='red', 
            fontsize=20, ha='center', va='center', fontweight='bold')
    
    plt.title("BFS Algorithm Pathfinding")
    plt.legend(loc='upper right')
    
    plt.savefig('../../results/images/bfs_result.png')
    print("BFS 搜索结果图片已保存至: results/images/bfs_result.png")
    plt.show()

if __name__ == "__main__":
    print(f"BFS - 起点: {start_point}, 终点: {end_point}")
    path, explored = bfs_search(mock_grid, start_point, end_point)
    
    if path:
        print(f"BFS 寻路成功！找到最短路径，步数：{len(path)-1}")
        print("路径坐标:", path)
        print(f"BFS 探索过的节点总数: {len(explored)}")
    else:
        print("BFS 未找到到达终点的路径！")
        
    visualize_bfs(mock_grid, start_point, end_point, path, explored)