import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import ListedColormap
import heapq
from ..config.map_basic import mock_grid, start_point, end_point


def dijkstra_search(grid, start, end):
    """
    Dijkstra 算法：无启发函数的最短路径算法
    
    与 A* 的核心区别：
    - A* 使用 f(n) = g(n) + h(n)（结合启发距离）
    - Dijkstra 仅使用 f(n) = g(n)（距离起点的实际距离，无启发）
    
    因此 Dijkstra 会探索更多节点，但不需要设计启发函数。
    """
    rows, cols = grid.shape
    # 八方向移动：上下左右 + 四个对角线
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]
    
    # 距离字典：记录起点到各节点的最短距离（初始为无穷）
    distance = {start: 0}
    
    # 优先队列：(当前距离, 节点坐标)
    # 注意：Dijkstra 中优先队列只包含距离值，没有启发函数的加权
    pq = [(0, start)]
    
    # 已访问集合（闭表）
    visited = set()
    
    # 记录父节点，用于回溯路径
    came_from = {}
    
    # 用于记录探索顺序，供可视化使用
    explored_nodes = []
    
    while pq:
        current_dist, current = heapq.heappop(pq)
        
        # 记录探索过的点
        if current not in explored_nodes:
            explored_nodes.append(current)
        
        # 如果已访问过，跳过（避免重复处理）
        if current in visited:
            continue
        
        # 标记为已访问
        visited.add(current)
        
        # 如果到达终点，立刻回溯路径并返回
        if current == end:
            path = []
            curr = current
            while curr in came_from:
                path.append(curr)
                curr = came_from[curr]
            path.append(start)
            path.reverse()
            return path, explored_nodes
        
        # 对当前节点的所有邻居进行松弛（relaxation）
        for d in directions:
            neighbor = (current[0] + d[0], current[1] + d[1])
            
            # 边界检查
            if not (0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols):
                continue
            
            # 避障检查
            if grid[neighbor[0], neighbor[1]] == 1:
                continue
            
            # 如果邻居已访问，则跳过（已确定为最短距离）
            if neighbor in visited:
                continue
            
            # 计算经过当前节点到达邻居的距离
            # 对角线移动的距离为 sqrt(2) ≈ 1.414，直线移动为 1
            move_cost = 1.414 if (d[0] != 0 and d[1] != 0) else 1.0
            new_dist = current_dist + move_cost
            
            # 如果这是首次发现该邻居，或找到了更短的路径，则更新
            if neighbor not in distance or new_dist < distance[neighbor]:
                distance[neighbor] = new_dist
                came_from[neighbor] = current
                heapq.heappush(pq, (new_dist, neighbor))
    
    # 无法到达终点
    return None, explored_nodes


def visualize_dijkstra(grid, start, end, path, explored):
    """
    可视化 Dijkstra 搜索过程：地图、探索节点、最短路径
    """
    cmap = ListedColormap(['white', 'blue'])
    
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.imshow(grid, cmap=cmap, origin='upper', extent=[0, grid.shape[1], grid.shape[0], 0])
    
    ax.set_xticks(np.arange(0, grid.shape[1], 1))
    ax.set_yticks(np.arange(0, grid.shape[0], 1))
    ax.grid(which='both', color='black', linestyle='-', linewidth=1)
    
    # 绘制探索过的节点（浅黄色背景）
    for node in explored:
        if node != start and node != end:
            ax.add_patch(plt.Rectangle((node[1], node[0]), 1, 1, color='yellow', alpha=0.4))
    
    # 绘制最短路径
    if path:
        path_y = [p[0] + 0.5 for p in path]
        path_x = [p[1] + 0.5 for p in path]
        ax.plot(path_x, path_y, marker='o', color='red', linewidth=3, markersize=6, label='Dijkstra Path')
    
    # 标注起点和终点
    ax.text(start[1] + 0.5, start[0] + 0.5, 'S', color='green', 
            fontsize=20, ha='center', va='center', fontweight='bold')
    ax.text(end[1] + 0.5, end[0] + 0.5, 'E', color='red', 
            fontsize=20, ha='center', va='center', fontweight='bold')
    
    plt.title("Dijkstra Algorithm Pathfinding")
    plt.legend(loc='upper right')
    plt.tight_layout()
    
    plt.savefig('../../results/images/dijkstra_result.png', dpi=100)
    print("Dijkstra 搜索结果图片已保存至: results/images/dijkstra_result.png")
    plt.show()


if __name__ == "__main__":
    print(f"起点: {start_point}, 终点: {end_point}")
    print("\n【Dijkstra 算法运行】")
    print("="*50)
    
    path, explored = dijkstra_search(mock_grid, start_point, end_point)
    
    if path:
        print(f"✓ 寻路成功！")
        print(f"  最短路径长度：{len(path)-1} 步")
        print(f"  探索节点总数：{len(explored)} 个")
        print(f"  路径坐标：{path}")
    else:
        print("✗ 未找到到达终点的路径！")
    
    print("="*50)
    
    visualize_dijkstra(mock_grid, start_point, end_point, path, explored)
