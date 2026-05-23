"""
cbs_solver.py - 云仓灵枢 CBS (Conflict-Based Search) 算法 Python 验证版

用于验证CBS算法在32x32栅格地图上为3台AGV规划无碰撞路径的正确性。
验证通过后将移植到K230的C代码中。

用法:
    python cbs_solver.py              # 运行演示
    python cbs_solver.py --test       # 运行单元测试
"""

import heapq
import time
from dataclasses import dataclass, field
from typing import Optional

# ============================================================
# 数据结构
# ============================================================

GRID_W = 32
GRID_H = 32
MAX_TIMESTEP = 64

CELL_EMPTY = 0
CELL_OBSTACLE = 1
CELL_CHARGER = 2
CELL_SHELF = 3


@dataclass
class SpacetimePoint:
    x: int
    y: int
    t: int


@dataclass
class SpacetimePath:
    agv_id: int
    points: list = field(default_factory=list)

    @property
    def len(self):
        return len(self.points)

    def get_position_at(self, t: int) -> tuple:
        """获取AGV在时间步t的位置，超出路径长度停留在终点"""
        if not self.points:
            return (0, 0)
        if t >= len(self.points):
            return (self.points[-1].x, self.points[-1].y)
        return (self.points[t].x, self.points[t].y)


@dataclass
class Constraint:
    agv_id: int
    x: int
    y: int
    timestep: int


@dataclass
class Conflict:
    agv_a: int
    agv_b: int
    x: int
    y: int
    timestep: int


@dataclass(order=True)
class CTNode:
    cost: int
    constraints: list = field(default_factory=list, compare=False)
    paths: dict = field(default_factory=dict, compare=False)
    parent: object = field(default=None, compare=False)


# ============================================================
# A* 低层路径规划 (带时空约束)
# ============================================================

def a_star_single(grid: list, start: tuple, goal: tuple,
                  agv_id: int, constraints: list) -> Optional[SpacetimePath]:
    """
    单AGV的A*路径规划，考虑时空约束。

    Args:
        grid: 32x32栅格地图 (0=空, 1=障碍)
        start: (x, y) 起点
        goal: (x, y) 终点
        agv_id: AGV编号
        constraints: 约束列表 [Constraint, ...]

    Returns:
        SpacetimePath 或 None (无解)
    """
    # 构建约束查找表: (x, y, t) -> True
    constraint_set = set()
    for c in constraints:
        if c.agv_id == agv_id:
            constraint_set.add((c.x, c.y, c.timestep))

    # A*节点: (f, g, x, y, t, parent_idx)
    # 使用heapq优先队列
    open_set = []
    h0 = abs(goal[0] - start[0]) + abs(goal[1] - start[1])
    heapq.heappush(open_set, (h0, 0, start[0], start[1], 0, -1))

    # closed表: (x, y, t) -> True
    closed = set()

    # 用于回溯: 记录每个节点的父节点
    came_from = {}  # (x, y, t) -> (px, py, pt)

    directions = [(0, 0), (0, 1), (0, -1), (1, 0), (-1, 0)]  # 等待+四方向

    while open_set:
        f, g, x, y, t, _ = heapq.heappop(open_set)

        state = (x, y, t)
        if state in closed:
            continue
        closed.add(state)

        # 到达目标：在目标点等待到时间窗口结束也算成功
        if (x, y) == goal:
            # 构建路径
            path_points = []
            cur = state
            while cur != (start[0], start[1], 0):
                path_points.append(SpacetimePoint(cur[0], cur[1], cur[2]))
                cur = came_from[cur]
            path_points.append(SpacetimePoint(start[0], start[1], 0))
            path_points.reverse()

            # 到达目标后，补充"等待"到最大时间步（让其他AGV有时间通过）
            last = path_points[-1]
            for wait_t in range(last.t + 1, min(last.t + 5, MAX_TIMESTEP)):
                path_points.append(SpacetimePoint(last.x, last.y, wait_t))

            return SpacetimePath(agv_id=agv_id, points=path_points)

        # 超时保护
        if t >= MAX_TIMESTEP - 1:
            continue

        # 扩展邻居
        for dx, dy in directions:
            nx, ny = x + dx, y + dy
            nt = t + 1

            # 边界检查
            if nx < 0 or nx >= GRID_W or ny < 0 or ny >= GRID_H:
                continue

            # 障碍物检查
            if grid[ny][nx] == CELL_OBSTACLE:
                continue

            # 约束检查
            if (nx, ny, nt) in constraint_set:
                continue

            if (nx, ny, nt) in closed:
                continue

            ng = g + 1
            nh = abs(goal[0] - nx) + abs(goal[1] - ny)
            nf = ng + nh

            came_from[(nx, ny, nt)] = state
            heapq.heappush(open_set, (nf, ng, nx, ny, nt, 0))

    return None  # 无解


# ============================================================
# 冲突检测
# ============================================================

def detect_conflict(paths: dict) -> Optional[Conflict]:
    """
    检测路径集合中的第一个冲突。

    Args:
        paths: {agv_id: SpacetimePath}

    Returns:
        Conflict 或 None (无冲突)
    """
    agv_ids = sorted(paths.keys())
    for i in range(len(agv_ids)):
        for j in range(i + 1, len(agv_ids)):
            id_a, id_b = agv_ids[i], agv_ids[j]
            path_a, path_b = paths[id_a], paths[id_b]

            max_t = max(path_a.len, path_b.len)
            for t in range(max_t):
                ax, ay = path_a.get_position_at(t)
                bx, by = path_b.get_position_at(t)

                # 顶点冲突：同一时间同一位置
                if ax == bx and ay == by:
                    return Conflict(
                        agv_a=id_a, agv_b=id_b,
                        x=ax, y=ay, timestep=t
                    )
    return None


# ============================================================
# CBS 高层算法
# ============================================================

def cbs_solve(grid: list, starts: dict, goals: dict,
              timeout_ms: int = 100) -> tuple:
    """
    CBS主函数：为多台AGV规划无碰撞路径。

    Args:
        grid: 32x32栅格地图
        starts: {agv_id: (x, y)} 各AGV起始位置
        goals: {agv_id: (x, y)} 各AGV目标位置
        timeout_ms: 超时限制(毫秒)

    Returns:
        (paths, solve_time_ms)
        paths: {agv_id: SpacetimePath} 或 None (无解)
        solve_time_ms: 求解耗时
    """
    start_time = time.time()

    # 初始化根节点：无约束，各AGV独立规划
    root = CTNode(cost=0)
    for agv_id in starts:
        path = a_star_single(grid, starts[agv_id], goals[agv_id],
                             agv_id, [])
        if path is None:
            return None, 0  # 单体就无解
        root.paths[agv_id] = path
        root.cost += path.len

    # 优先队列 (按cost排序)
    open_list = [root]

    iterations = 0

    while open_list:
        # 超时检查
        elapsed_ms = (time.time() - start_time) * 1000
        if elapsed_ms > timeout_ms:
            print(f"  [CBS] 超时! {elapsed_ms:.1f}ms > {timeout_ms}ms, "
                  f"迭代次数: {iterations}")
            return None, elapsed_ms

        best = heapq.heappop(open_list)
        iterations += 1

        # 检测冲突
        conflict = detect_conflict(best.paths)

        if conflict is None:
            # 无冲突，找到解!
            elapsed_ms = (time.time() - start_time) * 1000
            print(f"  [CBS] 求解成功! 耗时: {elapsed_ms:.2f}ms, "
                  f"迭代: {iterations}, 总代价: {best.cost}")
            return best.paths, elapsed_ms

        # 分裂：为冲突双方各创建一个子节点
        for side in range(2):
            constrained_agv = conflict.agv_a if side == 0 else conflict.agv_b
            other_agv = conflict.agv_b if side == 0 else conflict.agv_a

            child = CTNode(
                cost=0,
                constraints=list(best.constraints),
                paths=dict(best.paths),
                parent=best
            )

            # 添加约束
            child.constraints.append(Constraint(
                agv_id=constrained_agv,
                x=conflict.x, y=conflict.y,
                timestep=conflict.timestep
            ))

            # 低层：重新规划被约束AGV的路径
            new_path = a_star_single(
                grid,
                starts[constrained_agv],
                goals[constrained_agv],
                constrained_agv,
                child.constraints
            )

            if new_path is not None:
                child.paths[constrained_agv] = new_path
                # 重新计算总代价
                child.cost = sum(p.len for p in child.paths.values())
                heapq.heappush(open_list, child)

    elapsed_ms = (time.time() - start_time) * 1000
    print(f"  [CBS] 无解! 耗时: {elapsed_ms:.2f}ms, 迭代: {iterations}")
    return None, elapsed_ms


# ============================================================
# 降级策略：优先级规划 (CBS超时时使用)
# ============================================================

def priority_solve(grid: list, starts: dict, goals: dict) -> tuple:
    """
    优先级规划降级策略：按AGV编号顺序逐个规划，
    后规划的AGV把先规划的路径作为动态障碍物。

    优点：速度极快，保证有解（如果单体有解）
    缺点：不是最优解，高优先级AGV路径更短
    """
    start_time = time.time()

    paths = {}
    # 已规划路径作为动态障碍的约束
    all_constraints = []

    for agv_id in sorted(starts.keys()):
        path = a_star_single(grid, starts[agv_id], goals[agv_id],
                             agv_id, all_constraints)
        if path is None:
            return None, 0

        paths[agv_id] = path

        # 把这条路径的每个点作为其他AGV的约束
        for pt in path.points:
            for other_id in starts:
                if other_id != agv_id:
                    all_constraints.append(Constraint(
                        agv_id=other_id,
                        x=pt.x, y=pt.y,
                        timestep=pt.t
                    ))

    elapsed_ms = (time.time() - start_time) * 1000
    print(f"  [Priority] 降级规划完成! 耗时: {elapsed_ms:.2f}ms")
    return paths, elapsed_ms


def cbs_solve_with_fallback(grid: list, starts: dict, goals: dict,
                            timeout_ms: int = 100) -> tuple:
    """
    CBS + 优先级降级的组合策略。
    先尝试CBS，超时则降级为优先级规划。
    """
    paths, elapsed = cbs_solve(grid, starts, goals, timeout_ms)

    if paths is not None:
        return paths, elapsed, "CBS"

    print("  [Fallback] CBS超时，降级为优先级规划...")
    paths, elapsed = priority_solve(grid, starts, goals)
    return paths, elapsed, "Priority"


# ============================================================
# 可视化
# ============================================================

def print_grid(grid: list, paths: dict = None, step: int = -1):
    """打印栅格地图，可选显示AGV位置"""
    # 复制地图
    display = [row[:] for row in grid]

    # 标记AGV位置
    if paths:
        colors = {0: 'A', 1: 'B', 2: 'C', 3: 'D', 4: 'E'}
        for agv_id, path in paths.items():
            if step >= 0:
                x, y = path.get_position_at(step)
            else:
                x, y = path.points[0].x, path.points[0].y
            if 0 <= x < GRID_W and 0 <= y < GRID_H:
                display[y][x] = 10 + agv_id  # 用10+id标记AGV

    symbols = {0: ' . ', 1: '███', 2: ' ⚡', 3: ' 📦'}
    agv_symbols = {10: ' A ', 11: ' B ', 12: ' C ', 13: ' D ', 14: ' E '}

    print('   ' + ''.join(f'{i:3d}' for i in range(GRID_W)))
    for y in range(GRID_H):
        print(f'{y:2d} ', end='')
        for x in range(GRID_W):
            val = display[y][x]
            if val in agv_symbols:
                print(agv_symbols[val], end='')
            elif val in symbols:
                print(symbols[val], end='')
            else:
                print(f'{val:3d}', end='')
        print()


def print_path_summary(paths: dict):
    """打印路径摘要"""
    for agv_id in sorted(paths.keys()):
        path = paths[agv_id]
        start = (path.points[0].x, path.points[0].y)
        end = (path.points[-1].x, path.points[-1].y)
        print(f"  AGV#{agv_id}: ({start[0]},{start[1]}) -> "
              f"({end[0]},{end[1]}), 长度={path.len}步")


def animate_paths(grid: list, paths: dict, delay: float = 0.3):
    """动画显示AGV移动过程"""
    import os

    max_t = max(p.len for p in paths.values())

    for t in range(max_t):
        os.system('cls' if os.name == 'nt' else 'clear')
        print(f"=== 云仓灵枢 CBS路径规划 - 时间步 {t}/{max_t-1} ===\n")
        print_grid(grid, paths, step=t)
        print()
        for agv_id in sorted(paths.keys()):
            x, y = paths[agv_id].get_position_at(t)
            print(f"  AGV#{agv_id}: ({x},{y})")
        time.sleep(delay)


# ============================================================
# 测试用例
# ============================================================

def create_demo_grid() -> list:
    """创建一个演示用的32x32仓库地图"""
    grid = [[CELL_EMPTY] * GRID_W for _ in range(GRID_H)]

    # 添加一些货架 (障碍物)
    # 货架区域1: 左上
    for y in range(4, 8):
        for x in range(4, 8):
            grid[y][x] = CELL_OBSTACLE

    # 货架区域2: 右上
    for y in range(4, 8):
        for x in range(24, 28):
            grid[y][x] = CELL_OBSTACLE

    # 货架区域3: 中央
    for y in range(14, 18):
        for x in range(12, 20):
            grid[y][x] = CELL_OBSTACLE

    # 货架区域4: 左下
    for y in range(24, 28):
        for x in range(4, 8):
            grid[y][x] = CELL_OBSTACLE

    # 货架区域5: 右下
    for y in range(24, 28):
        for x in range(24, 28):
            grid[y][x] = CELL_OBSTACLE

    # 充电站
    grid[0][0] = CELL_CHARGER
    grid[0][31] = CELL_CHARGER
    grid[31][0] = CELL_CHARGER

    return grid


def test_basic():
    """基础测试：3台AGV简单路径"""
    print("=" * 60)
    print("测试1: 基础3-AGV路径规划")
    print("=" * 60)

    grid = create_demo_grid()

    starts = {1: (0, 0), 2: (31, 0), 3: (0, 31)}
    goals = {1: (31, 31), 2: (0, 31), 3: (31, 0)}

    print("\n起始位置:")
    for aid, pos in starts.items():
        print(f"  AGV#{aid}: {pos}")
    print("目标位置:")
    for aid, pos in goals.items():
        print(f"  AGV#{aid}: {pos}")

    print("\n地图:")
    print_grid(grid, {aid: SpacetimePath(agv_id=aid,
               points=[SpacetimePoint(s[0], s[1], 0)])
               for aid, s in starts.items()})

    print("\n开始CBS求解...")
    paths, elapsed = cbs_solve(grid, starts, goals, timeout_ms=2000)

    if paths:
        print("\n路径规划结果:")
        print_path_summary(paths)

        # 验证无碰撞
        conflict = detect_conflict(paths)
        if conflict:
            print(f"\n[错误] 检测到碰撞! AGV#{conflict.agv_a} vs "
                  f"AGV#{conflict.agv_b} 在 ({conflict.x},{conflict.y}) "
                  f"时间步 {conflict.timestep}")
        else:
            print("\n[通过] 无碰撞验证通过!")
    else:
        print("\n[失败] 无法找到无碰撞路径")

    return paths is not None


def test_crossing():
    """交叉测试：2台AGV需要交叉通过"""
    print("\n" + "=" * 60)
    print("测试2: 交叉路径 (需要CBS协调)")
    print("=" * 60)

    grid = [[CELL_EMPTY] * GRID_W for _ in range(GRID_H)]

    # 两台AGV面对面需要交叉
    starts = {1: (0, 15), 2: (31, 15)}
    goals = {1: (31, 15), 2: (0, 15)}

    print("\n场景: AGV#1从左到右, AGV#2从右到左, 必须交叉")

    paths, elapsed = cbs_solve(grid, starts, goals, timeout_ms=2000)

    if paths:
        print_path_summary(paths)
        conflict = detect_conflict(paths)
        if conflict:
            print(f"[错误] 检测到碰撞!")
            return False
        else:
            print("[通过] 交叉路径无碰撞!")
            return True
    return False


def test_corridor():
    """走廊测试：2台AGV在窄走廊中需要让行"""
    print("\n" + "=" * 60)
    print("测试3: 窄走廊场景 (2台AGV让行)")
    print("=" * 60)

    grid = [[CELL_EMPTY] * GRID_W for _ in range(GRID_H)]

    # 添加墙壁，形成窄走廊 (y=10和y=20是墙)
    for x in range(GRID_W):
        grid[10][x] = CELL_OBSTACLE
        grid[20][x] = CELL_OBSTACLE

    # 2台AGV在走廊中面对面
    starts = {1: (5, 15), 2: (25, 15)}
    goals = {1: (25, 15), 2: (5, 15)}

    print("\n场景: 2台AGV在窄走廊中面对面需要让行")

    paths, elapsed = cbs_solve(grid, starts, goals, timeout_ms=2000)

    if paths:
        print_path_summary(paths)
        conflict = detect_conflict(paths)
        if conflict:
            print(f"[错误] 检测到碰撞!")
            return False
        else:
            print("[通过] 窄走廊无碰撞!")
            return True
    return False


def test_performance():
    """性能测试：测量CBS求解时间"""
    print("\n" + "=" * 60)
    print("测试4: 性能基准 (3台AGV, 32x32地图)")
    print("=" * 60)

    grid = create_demo_grid()

    import random
    random.seed(42)

    times = []
    successes = 0

    for i in range(10):
        # 随机生成起始和目标位置 (避开障碍物)
        starts = {}
        goals = {}
        used = set()

        for agv_id in range(1, 4):
            while True:
                sx, sy = random.randint(0, 31), random.randint(0, 31)
                if grid[sy][sx] == CELL_EMPTY and (sx, sy) not in used:
                    starts[agv_id] = (sx, sy)
                    used.add((sx, sy))
                    break

            while True:
                gx, gy = random.randint(0, 31), random.randint(0, 31)
                if grid[gy][gx] == CELL_EMPTY and (gx, gy) not in used:
                    goals[agv_id] = (gx, gy)
                    used.add((gx, gy))
                    break

        paths, elapsed = cbs_solve(grid, starts, goals, timeout_ms=500)
        times.append(elapsed)
        if paths:
            successes += 1

    print(f"\n结果: {successes}/10 成功")
    print(f"平均耗时: {sum(times)/len(times):.2f}ms")
    print(f"最大耗时: {max(times):.2f}ms")
    print(f"最小耗时: {min(times):.2f}ms")

    return successes >= 8  # 至少80%成功率


def run_all_tests():
    """运行所有测试"""
    print("╔══════════════════════════════════════════════════════╗")
    print("║     云仓灵枢 CBS算法 Python验证版 - 测试套件       ║")
    print("╚══════════════════════════════════════════════════════╝\n")

    results = []

    results.append(("基础3-AGV", test_basic()))
    results.append(("交叉路径", test_crossing()))
    results.append(("窄走廊", test_corridor()))
    results.append(("性能基准", test_performance()))

    print("\n" + "=" * 60)
    print("测试汇总:")
    print("=" * 60)
    for name, passed in results:
        status = "PASS" if passed else "FAIL"
        print(f"  [{status}] {name}")

    all_passed = all(r[1] for r in results)
    print(f"\n{'全部通过!' if all_passed else '存在失败项!'}")
    return all_passed


# ============================================================
# 入口
# ============================================================

if __name__ == '__main__':
    import sys

    if '--test' in sys.argv:
        run_all_tests()
    else:
        # 默认运行演示
        print("╔══════════════════════════════════════════════════════╗")
        print("║     云仓灵枢 CBS算法 - 3-AGV无碰撞路径规划演示     ║")
        print("╚══════════════════════════════════════════════════════╝\n")

        grid = create_demo_grid()

        starts = {1: (0, 0), 2: (31, 0), 3: (16, 31)}
        goals = {1: (31, 31), 2: (16, 0), 3: (0, 31)}

        print("地图 (█=障碍, ⚡=充电站):")
        print_grid(grid)
        print()

        print("AGV任务:")
        for aid in starts:
            print(f"  AGV#{aid}: ({starts[aid][0]},{starts[aid][1]}) "
                  f"-> ({goals[aid][0]},{goals[aid][1]})")

        print("\n开始CBS路径规划...")
        paths, elapsed = cbs_solve(grid, starts, goals, timeout_ms=2000)

        if paths:
            print("\n规划结果:")
            print_path_summary(paths)

            # 验证
            conflict = detect_conflict(paths)
            print(f"\n碰撞验证: {'通过 (零碰撞)' if conflict is None else '失败!'}")

            # 显示动画 (如果有--animate参数)
            if '--animate' in sys.argv:
                print("\n按Ctrl+C停止动画...")
                try:
                    animate_paths(grid, paths, delay=0.5)
                except KeyboardInterrupt:
                    print("\n动画停止")
        else:
            print("规划失败!")
