"""
Robot Path Planning Algorithms - Main Runner

This script demonstrates and compares different pathfinding algorithms:
- Search algorithms (A*, BFS, Dijkstra)
- APF algorithms (Basic, Improved, Hybrid with A*)
"""

import sys
import os
import argparse
import numpy as np
import matplotlib.pyplot as plt
from src.config.map_basic import mock_grid, start_point, end_point
from src.config.map_hard import hard_grid, start_point_hard, end_point_hard
from src.algorithms import a_star_search, bfs_search, dijkstra_search
from src.algorithms import apf_basic_search, apf_improved_search, apf_astar_search
from src.utils.visualization import visualize_search_result, visualize_map


def run_search_algorithms():
    """运行所有搜索算法并比较结果"""
    print("\n" + "="*60)
    print("  SEARCH ALGORITHMS (A*, BFS, Dijkstra)")
    print("="*60)
    
    # 准备结果保存目录
    result_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'results', 'images')
    os.makedirs(result_dir, exist_ok=True)
    
    algorithms = {
        'A*': (a_star_search, 'A* Algorithm', 'astar_result.png'),
        'BFS': (bfs_search, 'BFS Algorithm', 'bfs_result.png'),
        'Dijkstra': (dijkstra_search, 'Dijkstra Algorithm', 'dijkstra_result.png'),
    }
    
    results = {}
    
    for name, (algo_func, display_name, image_name) in algorithms.items():
        print(f"\n[{name}] Running {display_name}...")
        path, explored = algo_func(mock_grid, start_point, end_point)
        
        if path:
            print(f"  ✓ Success! Path length: {len(path)-1} steps")
            print(f"  Explored nodes: {len(explored)}")
            
            # 生成可视化图片
            image_path = os.path.join(result_dir, image_name)
            visualize_search_result(
                mock_grid, start_point, end_point, path, explored,
                display_name, image_path
            )
            print(f"  Image saved: {image_name}")
            plt.close('all')
            
            results[name] = {
                'path': path,
                'explored': explored,
                'success': True,
                'steps': len(path) - 1,
            }
        else:
            print(f"  ✗ Failed to find path")
            results[name] = {'success': False}
    
    return results


def run_apf_algorithms():
    """运行所有APF算法"""
    print("\n" + "="*60)
    print("  APF ALGORITHMS (Hard Grid with U-Shaped Obstacle)")
    print("="*60)
    
    # 准备结果保存目录
    result_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'results', 'images')
    os.makedirs(result_dir, exist_ok=True)
    
    apf_algorithms = {
        'APF-Basic': (apf_basic_search, 'apf_basic_result.png', False),
        'APF-Improved': (apf_improved_search, 'apf_improved_result.png', False),
        'APF+A* Hybrid': (apf_astar_search, 'apf_astar_result.png', True),
    }
    
    results = {}
    
    for name, (algo_func, image_name, is_hybrid) in apf_algorithms.items():
        print(f"\n[{name}] Running {name}...")
        
        if is_hybrid:
            path, explored, success, mode_log = algo_func(hard_grid, start_point_hard, end_point_hard)
            results[name] = {
                'path': path,
                'explored': explored,
                'success': success,
                'steps': len(path) - 1,
                'mode_log': mode_log,
            }
        else:
            path, explored, success = algo_func(hard_grid, start_point_hard, end_point_hard)
            results[name] = {
                'path': path,
                'explored': explored,
                'success': success,
                'steps': len(path) - 1,
            }
        
        if success:
            print(f"  ✓ Success! Path length: {len(path)-1} steps")
            print(f"  Explored nodes: {len(explored)}")
        else:
            print(f"  ✗ Failed or stuck in local minimum")
        
        # 生成可视化图片（通过调用原始算法的可视化函数）
        image_path = os.path.join(result_dir, image_name)
        _generate_apf_visualization(name, hard_grid, start_point_hard, end_point_hard, 
                                   path, explored, success, image_path, mode_log if is_hybrid else None)
        print(f"  Image saved: {image_name}")
    
    return results


def _generate_apf_visualization(algo_name, grid, start, end, path, explored, success, image_path, mode_log=None):
    """生成APF算法的可视化图片"""
    from src.algorithms.apf_basic import get_obstacles, calculate_potential
    import numpy as np
    from matplotlib.colors import ListedColormap
    
    obs = get_obstacles(grid)
    
    if 'Basic' in algo_name:
        # 基础APF可视化：显示势能场热力图
        potential_grid = np.zeros_like(grid, dtype=float)
        valid_potentials = []
        
        for r in range(grid.shape[0]):
            for c in range(grid.shape[1]):
                if grid[r, c] == 0:
                    u = calculate_potential((r, c), end, obs)
                    potential_grid[r, c] = u
                    valid_potentials.append(u)
        
        vmax = np.percentile(valid_potentials, 95) if valid_potentials else 1
        
        for r in range(grid.shape[0]):
            for c in range(grid.shape[1]):
                if grid[r, c] == 1:
                    potential_grid[r, c] = vmax * 1.1
        
        fig, ax = plt.subplots(figsize=(8, 8))
        cax = ax.imshow(potential_grid, cmap='viridis', origin='upper', 
                        extent=[0, grid.shape[1], grid.shape[0], 0], vmax=vmax)
        fig.colorbar(cax, shrink=0.8, label='Total Potential Energy Field')
        
        ax.set_xticks(np.arange(0, grid.shape[1], 1))
        ax.set_yticks(np.arange(0, grid.shape[0], 1))
        ax.grid(which='both', color='white', linestyle='-', linewidth=0.5, alpha=0.3)
        
        for r in range(grid.shape[0]):
            for c in range(grid.shape[1]):
                if grid[r, c] == 1:
                    ax.add_patch(plt.Rectangle((c, r), 1, 1, facecolor='black', alpha=0.6))
        
        if path:
            path_y = [p[0] + 0.5 for p in path]
            path_x = [p[1] + 0.5 for p in path]
            color = 'cyan' if success else 'orange'
            ax.plot(path_x, path_y, marker='o', color=color, linewidth=2, markersize=5, label='APF Path')
            
            if not success:
                ax.plot(path_x[-1], path_y[-1], marker='X', color='red', markersize=15)
        
        ax.text(start[1] + 0.5, start[0] + 0.5, 'S', color='lightgreen', fontsize=18, ha='center', va='center', fontweight='bold')
        ax.text(end[1] + 0.5, end[0] + 0.5, 'E', color='red', fontsize=18, ha='center', va='center', fontweight='bold')
        
        ax.set_title(f"Basic APF - {'Success' if success else 'Deadlock'}")
        ax.legend(loc='lower left')
        
    elif 'Improved' in algo_name or 'Hybrid' in algo_name:
        # 简化APF可视化（无热力图）
        cmap = ListedColormap(['white', 'blue'])
        fig, ax = plt.subplots(figsize=(8, 8))
        ax.imshow(grid, cmap=cmap, origin='upper', extent=[0, grid.shape[1], grid.shape[0], 0])
        
        ax.set_xticks(np.arange(0, grid.shape[1], 1))
        ax.set_yticks(np.arange(0, grid.shape[0], 1))
        ax.grid(which='both', color='black', linestyle='-', linewidth=0.5)
        
        if path:
            path_y = [p[0] + 0.5 for p in path]
            path_x = [p[1] + 0.5 for p in path]
            
            if 'Hybrid' in algo_name and mode_log:
                # 分段绘制混合算法的路径
                COLOR = {'APF': '#00e5ff', 'A*': '#ff9800'}
                for i in range(len(path) - 1):
                    p0, p1 = path[i], path[i+1]
                    x0, y0 = p0[1]+0.5, p0[0]+0.5
                    x1, y1 = p1[1]+0.5, p1[0]+0.5
                    mode = mode_log[i+1] if i+1 < len(mode_log) else mode_log[-1]
                    ax.annotate("", xy=(x1, y1), xytext=(x0, y0),
                               arrowprops=dict(arrowstyle="-|>", color=COLOR[mode], lw=2.2))
                
                for mode, color in COLOR.items():
                    pts = [path[i] for i in range(len(path))
                           if i < len(mode_log) and mode_log[i] == mode]
                    if pts:
                        ax.scatter([p[1]+0.5 for p in pts],
                                  [p[0]+0.5 for p in pts],
                                  c=color, s=35, zorder=5)
            else:
                color = 'cyan' if success else 'orange'
                ax.plot(path_x, path_y, marker='o', color=color, linewidth=2, markersize=5)
        
        ax.text(start[1] + 0.5, start[0] + 0.5, 'S', color='green', fontsize=18, ha='center', va='center', fontweight='bold')
        ax.text(end[1] + 0.5, end[0] + 0.5, 'E', color='red', fontsize=18, ha='center', va='center', fontweight='bold')
        
        title = f"{algo_name} - {'Success' if success else 'Failed'}"
        ax.set_title(title)
    
    plt.tight_layout()
    plt.savefig(image_path, dpi=150, bbox_inches='tight')
    plt.close('all')


def main():
    parser = argparse.ArgumentParser(description='Robot Path Planning Algorithms')
    parser.add_argument('--search', action='store_true', help='Run search algorithms')
    parser.add_argument('--apf', action='store_true', help='Run APF algorithms')
    parser.add_argument('--all', action='store_true', help='Run all algorithms')
    parser.add_argument('--visualize', action='store_true', help='Generate visualizations')
    
    args = parser.parse_args()
    
    # Default: run all
    if not args.search and not args.apf and not args.all:
        args.all = True
    
    search_results = None
    apf_results = None
    
    if args.search or args.all:
        search_results = run_search_algorithms()
    
    if args.apf or args.all:
        apf_results = run_apf_algorithms()
    
    # 总结
    print("\n" + "="*60)
    print("  SUMMARY")
    print("="*60)
    
    if search_results:
        print("\n[Search Algorithms on Basic Grid]")
        for name, result in search_results.items():
            if result['success']:
                print(f"  {name:12s}: {result['steps']:3d} steps, "
                      f"{result['explored'].__len__():3d} explored nodes")
            else:
                print(f"  {name:12s}: FAILED")
    
    if apf_results:
        print("\n[APF Algorithms on Hard Grid]")
        for name, result in apf_results.items():
            status = "✓ SUCCESS" if result['success'] else "✗ FAILED"
            print(f"  {name:20s}: {status}, {result['steps']:3d} steps")
    
    print("\n" + "="*60)


if __name__ == '__main__':
    main()
