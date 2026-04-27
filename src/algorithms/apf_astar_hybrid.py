import numpy as np
import matplotlib.pyplot as plt
import heapq
from ..config.map_hard import hard_grid, start_point_hard, end_point_hard


# ─────────────────────────────────────────────
#  共用工具
# ─────────────────────────────────────────────

def get_obstacles(grid):
    obs = []
    for r in range(grid.shape[0]):
        for c in range(grid.shape[1]):
            if grid[r, c] == 1:
                obs.append((r, c))
    return obs


def calculate_potential(pos, end, obs, k_att=1.0, k_rep=20.0, rho_0=3.0):
    """
    总势能 U = U_att + U_rep（与 apf_basic.py 完全一致）
    """
    d_goal = np.sqrt((pos[0] - end[0])**2 + (pos[1] - end[1])**2)
    u_att = 0.5 * k_att * (d_goal ** 2)

    u_rep = 0.0
    for ob in obs:
        d_obs = np.sqrt((pos[0] - ob[0])**2 + (pos[1] - ob[1])**2)
        if 0 < d_obs <= rho_0:
            u_rep += 0.5 * k_rep * ((1.0 / d_obs - 1.0 / rho_0) ** 2)

    return u_att + u_rep


# ─────────────────────────────────────────────
#  A* 全局寻路
# ─────────────────────────────────────────────

def astar(grid, start, end):
    """
    标准 A*，返回从 start 到 end 的节点列表（含首尾）。
    找不到路径则返回 None。
    """
    rows, cols = grid.shape
    directions = [(-1,0),(1,0),(0,-1),(0,1),
                  (-1,-1),(-1,1),(1,-1),(1,1)]

    def heuristic(a, b):
        return max(abs(a[0]-b[0]), abs(a[1]-b[1]))  # 切比雪夫距离（八方向）

    open_heap = []
    heapq.heappush(open_heap, (heuristic(start, end), 0, start))

    came_from = {}
    g_score   = {start: 0}
    visited   = set()

    while open_heap:
        f, g, current = heapq.heappop(open_heap)

        if current in visited:
            continue
        visited.add(current)

        if current == end:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()
            return path

        for d in directions:
            nr, nc = current[0]+d[0], current[1]+d[1]
            if not (0 <= nr < rows and 0 <= nc < cols):
                continue
            if grid[nr, nc] == 1:
                continue
            neighbor    = (nr, nc)
            move_cost   = 1.414 if (d[0] != 0 and d[1] != 0) else 1.0
            tentative_g = g + move_cost

            if tentative_g < g_score.get(neighbor, float('inf')):
                g_score[neighbor]   = tentative_g
                came_from[neighbor] = current
                heapq.heappush(open_heap,
                               (tentative_g + heuristic(neighbor, end),
                                tentative_g, neighbor))
    return None


# ─────────────────────────────────────────────
#  APF + A* 混合算法（修复版）
# ─────────────────────────────────────────────

def apf_astar_search(grid, start, end,
                     max_iter=500,
                     k_att=1.0, k_rep=20.0, rho_0=3.0):
    """
    修复版混合策略，两阶段硬切换：

    【Phase 1 — 纯 APF】
        正常执行 APF 梯度下降，向势能最低的邻居移动。

    【Phase 2 — A* 完全接管】
        一旦检测到 deadlock，立即调用 A* 规划从当前位置到终点的完整路径。
        之后 **逐步跟随 A* 路径，完全不再做 APF 势能判断**。

        关键修复：之前版本在 A* 接管后仍用 APF 势能做梯度下降，
        导致 A* waypoint 附近的障碍斥力场使当前点势能反而最低，
        立刻再次触发 deadlock。现在 A* 接管后直接 pop 队列前进，
        不再调用 calculate_potential，彻底消除二次死锁。

    返回: (path, explored, success, mode_log)
        mode_log: 每步模式 'APF' 或 'A*'
    """
    obs      = get_obstacles(grid)
    path     = [start]
    explored = [start]
    mode_log = ['APF']

    directions = [(-1,0),(1,0),(0,-1),(0,1),
                  (-1,-1),(-1,1),(1,-1),(1,1)]

    current     = start
    astar_queue = []    # A* 接管后的剩余路径队列
    phase       = 'APF'

    for iteration in range(max_iter):
        if current == end:
            print(f">>> 到达终点！共 {iteration} 步。")
            return path, explored, True, mode_log

        # ══════════════════════════════════════════
        #  Phase 2：A* 完全接管 — 直接跟路径走，不做任何势能判断
        # ══════════════════════════════════════════
        if phase == 'A*':
            if not astar_queue:
                print(">>> A* 队列意外耗尽但未到终点，失败。")
                return path, explored, False, mode_log

            current = astar_queue.pop(0)
            path.append(current)
            mode_log.append('A*')
            explored.append(current)
            continue

        # ══════════════════════════════════════════
        #  Phase 1：APF 梯度下降
        # ══════════════════════════════════════════
        min_u        = calculate_potential(current, end, obs, k_att, k_rep, rho_0)
        best_next    = current
        found_better = False

        for d in directions:
            nr, nc = current[0]+d[0], current[1]+d[1]
            if not (0 <= nr < grid.shape[0] and 0 <= nc < grid.shape[1]):
                continue
            if grid[nr, nc] == 1:
                continue
            neighbor = (nr, nc)
            u        = calculate_potential(neighbor, end, obs, k_att, k_rep, rho_0)
            explored.append(neighbor)
            if u < min_u:
                min_u        = u
                best_next    = neighbor
                found_better = True

        # ── Deadlock 检测 → 硬切换到 A* ────────
        if not found_better:
            print(f">>> [步{iteration}] APF deadlock @ {current}，"
                  f"切换到 A* 完全接管模式...")

            astar_path = astar(grid, current, end)
            if astar_path is None:
                print(">>> A* 也找不到路径，地图无解。")
                return path, explored, False, mode_log

            astar_queue = astar_path[1:]   # 跳过 current（已在 path 里）
            phase       = 'A*'
            print(f"    A* 规划成功，接管剩余 {len(astar_queue)} 步，"
                  f"终点确认: {astar_path[-1]}")
            continue    # 下一轮直接走 A* 分支

        # APF 正常前进
        current = best_next
        path.append(current)
        mode_log.append('APF')

    print(">>> 超过最大迭代次数。")
    return path, explored, False, mode_log


# ─────────────────────────────────────────────
#  可视化
# ─────────────────────────────────────────────

def visualize_apf_astar(grid, start, end, path, explored, success, mode_log):
    obs = get_obstacles(grid)

    # 势能场热力图
    potential_grid   = np.zeros_like(grid, dtype=float)
    valid_potentials = []
    for r in range(grid.shape[0]):
        for c in range(grid.shape[1]):
            if grid[r, c] == 0:
                u = calculate_potential((r, c), end, obs)
                potential_grid[r, c] = u
                valid_potentials.append(u)

    vmax = np.percentile(valid_potentials, 95)
    for r in range(grid.shape[0]):
        for c in range(grid.shape[1]):
            if grid[r, c] == 1:
                potential_grid[r, c] = vmax * 1.1

    fig, ax = plt.subplots(figsize=(9, 9))
    cax = ax.imshow(potential_grid, cmap='viridis', origin='upper',
                    extent=[0, grid.shape[1], grid.shape[0], 0], vmax=vmax)
    fig.colorbar(cax, shrink=0.8, label='Total Potential Energy Field')

    ax.set_xticks(np.arange(0, grid.shape[1], 1))
    ax.set_yticks(np.arange(0, grid.shape[0], 1))
    ax.grid(which='both', color='white', linestyle='-', linewidth=0.5, alpha=0.3)

    # 障碍物
    for r in range(grid.shape[0]):
        for c in range(grid.shape[1]):
            if grid[r, c] == 1:
                ax.add_patch(plt.Rectangle((c, r), 1, 1,
                                           facecolor='black', alpha=0.6))

    # ── 路径分段绘制 ─────────────────────────────
    COLOR = {'APF': '#00e5ff', 'A*': '#ff9800'}

    if path and len(path) > 1:
        for i in range(len(path) - 1):
            p0, p1 = path[i], path[i+1]
            x0, y0 = p0[1]+0.5, p0[0]+0.5
            x1, y1 = p1[1]+0.5, p1[0]+0.5
            mode  = mode_log[i+1] if i+1 < len(mode_log) else mode_log[-1]
            ax.annotate("", xy=(x1, y1), xytext=(x0, y0),
                        arrowprops=dict(arrowstyle="-|>",
                                        color=COLOR[mode], lw=2.2))

        for mode, color in COLOR.items():
            pts = [path[i] for i in range(len(path))
                   if i < len(mode_log) and mode_log[i] == mode]
            if pts:
                label = (f'APF Gradient Descent ({len(pts)} steps)'
                         if mode == 'APF'
                         else f'A* Full Control ({len(pts)} steps)')
                ax.scatter([p[1]+0.5 for p in pts],
                           [p[0]+0.5 for p in pts],
                           c=color, s=35, zorder=5, label=label)

    # 切换点（钻石标记）
    switch_idx = [i for i, m in enumerate(mode_log) if m == 'A*']
    if switch_idx:
        sp = path[switch_idx[0] - 1]
        ax.plot(sp[1]+0.5, sp[0]+0.5,
                marker='D', color='yellow', markersize=11, zorder=6,
                label=f'Deadlock → A* Switch @ {sp}')

    if not success and path:
        lp = path[-1]
        ax.plot(lp[1]+0.5, lp[0]+0.5, marker='X',
                color='red', markersize=15, zorder=6,
                label='Deadlock (unresolved)')

    ax.text(start[1]+0.5, start[0]+0.5, 'S',
            color='lightgreen', fontsize=18, ha='center', va='center',
            fontweight='bold', zorder=7)
    ax.text(end[1]+0.5, end[0]+0.5, 'E',
            color='red', fontsize=18, ha='center', va='center',
            fontweight='bold', zorder=7)

    status = "SUCCESS ✓" if success else "FAILED ✗"
    plt.title(f"APF + A* Hybrid Navigation  [{status}]  |  "
              f"Total steps: {len(path)-1}")
    plt.legend(loc='lower left', fontsize=9)
    plt.tight_layout()
    plt.savefig('../../results/images/apf_astar_result.png', dpi=150)
    print(">>> 图像已保存为 results/images/apf_astar_result.png")
    plt.show()



if __name__ == "__main__":
    print("=" * 55)
    print("  APF + A* Hybrid Path Planning  (Fixed)")
    print("=" * 55)

    path, explored, success, mode_log = apf_astar_search(
        hard_grid, start_point_hard, end_point_hard
    )

    apf_steps   = mode_log.count('APF')
    astar_steps = mode_log.count('A*')
    print(f"\n路径总长度  : {len(path)-1} 步")
    print(f"  APF 阶段  : {apf_steps} 步")
    print(f"  A* 接管   : {astar_steps} 步")
    print(f"是否成功    : {'✓ YES' if success else '✗ NO'}")

    visualize_apf_astar(
        hard_grid, start_point_hard, end_point_hard,
        path, explored, success, mode_log
    )