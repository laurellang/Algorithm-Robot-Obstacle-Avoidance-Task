import numpy as np

# 进阶难度地图 (15x15)
# 设计了一个经典的 "U型陷阱" 障碍物，这正是传统人工势场法(APF)的克星
hard_grid = np.zeros((15, 15), dtype=int)

# 构造 U 型障碍物区域
hard_grid[4:11, 9] = 1   # 右侧墙壁
hard_grid[4, 5:10] = 1   # 上侧墙壁
hard_grid[10, 5:10] = 1  # 下侧墙壁

# 还在边缘增加一些随机干扰障碍
hard_grid[2, 2:5] = 1
hard_grid[12, 10:14] = 1

start_point_hard = (7, 3)   # 起点在 U 型陷阱开口正前方
end_point_hard = (7, 13)    # 终点在 U 型陷阱后面
