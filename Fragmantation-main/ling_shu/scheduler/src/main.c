/**
 * scheduler - 云仓灵枢 CBS调度引擎进程
 *
 * 运行在K230 (RT-Thread Smart) 上，负责：
 * 1. 从共享内存读取AGV实时状态
 * 2. 执行CBS算法规划无碰撞路径
 * 3. 管理任务队列（分配、执行、完成）
 * 4. 动态重规划（冲突/新任务/AGV故障触发）
 * 5. 通过消息队列将指令发送给comm_gw
 */

#include <rtthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <mqueue.h>

#include "agv_protocol.h"

/* ============================================================
 * 全局变量
 * ============================================================ */

static shm_agv_t *g_shm = NULL;
static mqd_t g_mq_cmd = -1;            /* scheduler -> comm_gw 指令队列 */
static mqd_t g_mq_alert = -1;          /* scheduler -> web_monitor 告警队列 */
static volatile int g_running = 1;

/* CBS节点池 (静态分配) */
static ct_node_t g_ct_nodes[MAX_CT_NODES];
static uint16_t g_open_list[MAX_CT_NODES];
static uint8_t g_open_count = 0;
static uint16_t g_next_free = 0;

/* A*静态数据 */
static uint8_t g_closed[GRID_H][GRID_W][MAX_TIMESTEP];

/* ============================================================
 * 工具函数
 * ============================================================ */

static int abs_int(int x) { return x < 0 ? -x : x; }

void get_position_at_time(const spacetime_path_t *path, uint8_t t,
                          int8_t *out_x, int8_t *out_y)
{
    if (path->len == 0) {
        *out_x = 0; *out_y = 0;
        return;
    }
    if (t >= path->len) {
        *out_x = path->points[path->len - 1].x;
        *out_y = path->points[path->len - 1].y;
        return;
    }
    *out_x = path->points[t].x;
    *out_y = path->points[t].y;
}

/* ============================================================
 * 共享内存连接
 * ============================================================ */

static int shm_connect(void)
{
    int fd = shm_open(SHM_NAME, O_RDWR, 0);
    if (fd < 0) {
        printf("[scheduler] 无法连接共享内存: %s\n", strerror(errno));
        return -1;
    }

    g_shm = (shm_agv_t *)mmap(NULL, sizeof(shm_agv_t),
                               PROT_READ | PROT_WRITE,
                               MAP_SHARED, fd, 0);
    if (g_shm == MAP_FAILED) {
        printf("[scheduler] mmap失败\n");
        close(fd);
        return -1;
    }

    close(fd);
    printf("[scheduler] 共享内存连接成功\n");
    return 0;
}

/* ============================================================
 * A* 低层路径规划 (带时空约束)
 * ============================================================ */

/* A*节点 */
typedef struct {
    int8_t  x, y;
    uint8_t g_cost;
    uint16_t parent_idx;
    uint8_t t;
    uint8_t valid;
} astar_node_t;

static astar_node_t g_astar_nodes[GRID_W * GRID_H * 2];

static int a_star_single(const uint8_t grid[GRID_H][GRID_W],
                         int16_t start_x, int16_t start_y,
                         int16_t goal_x, int16_t goal_y,
                         uint8_t agv_id,
                         const constraint_t *constraints,
                         uint8_t num_constraints,
                         spacetime_path_t *out_path)
{
    /* 构建约束查找表 */
    uint8_t constrained[GRID_H][GRID_W][MAX_TIMESTEP];
    memset(constrained, 0, sizeof(constrained));
    for (uint8_t i = 0; i < num_constraints; i++) {
        if (constraints[i].agv_id == agv_id &&
            constraints[i].x >= 0 && constraints[i].x < GRID_W &&
            constraints[i].y >= 0 && constraints[i].y < GRID_H &&
            constraints[i].timestep < MAX_TIMESTEP) {
            constrained[constraints[i].y][constraints[i].x]
                       [constraints[i].timestep] = 1;
        }
    }

    /* 清空closed表 */
    memset(g_closed, 0, sizeof(g_closed));

    /* 开放列表 (简单数组+线性搜索，适合小规模) */
    uint16_t open_count = 0;
    uint16_t node_count = 0;

    /* 起点 */
    g_astar_nodes[0] = (astar_node_t){
        .x = (int8_t)start_x,
        .y = (int8_t)start_y,
        .g_cost = 0,
        .parent_idx = 0xFFFF,
        .t = 0,
        .valid = 1
    };
    open_count = 1;
    node_count = 1;

    /* 四方向+等待 */
    const int8_t dx[] = {0, 0, 0, 1, -1};
    const int8_t dy[] = {0, 1, -1, 0, 0};

    uint16_t goal_node_idx = 0xFFFF;
    uint32_t iterations = 0;

    while (open_count > 0 && iterations < 10000) {
        iterations++;

        /* 找f值最小的节点 */
        uint16_t best_idx = 0;
        uint8_t best_f = 255;
        for (uint16_t i = 0; i < open_count; i++) {
            uint16_t ni = (i == 0) ? 0 : i;  /* 简化 */
            astar_node_t *n = &g_astar_nodes[ni];
            if (!n->valid) continue;
            int h = abs_int(goal_x - n->x) + abs_int(goal_y - n->y);
            uint8_t f = n->g_cost + (uint8_t)(h > 255 ? 255 : h);
            if (f < best_f) {
                best_f = f;
                best_idx = ni;
            }
        }

        astar_node_t *current = &g_astar_nodes[best_idx];
        current->valid = 0;  /* 移出开放列表 */

        /* 到达目标 */
        if (current->x == goal_x && current->y == goal_y) {
            goal_node_idx = best_idx;

            /* 回溯路径 */
            uint16_t path_len = 0;
            uint16_t cur = goal_node_idx;
            while (cur != 0xFFFF && path_len < MAX_PATH_LEN) {
                path_len++;
                cur = g_astar_nodes[cur].parent_idx;
            }

            out_path->len = (uint8_t)path_len;
            cur = goal_node_idx;
            for (int16_t i = path_len - 1; i >= 0; i--) {
                out_path->points[i].x = g_astar_nodes[cur].x;
                out_path->points[i].y = g_astar_nodes[cur].y;
                out_path->points[i].t = g_astar_nodes[cur].t;
                cur = g_astar_nodes[cur].parent_idx;
            }

            /* 补充等待步 */
            if (path_len > 0 && path_len < MAX_PATH_LEN) {
                int8_t last_x = out_path->points[path_len - 1].x;
                int8_t last_y = out_path->points[path_len - 1].y;
                uint8_t last_t = out_path->points[path_len - 1].t;
                for (uint8_t wt = 1; wt < 5 && path_len < MAX_PATH_LEN; wt++) {
                    out_path->points[path_len].x = last_x;
                    out_path->points[path_len].y = last_y;
                    out_path->points[path_len].t = last_t + wt;
                    path_len++;
                }
                out_path->len = path_len;
            }

            return 0;  /* 成功 */
        }

        /* 超时保护 */
        if (current->t >= MAX_TIMESTEP - 1) continue;

        /* 扩展邻居 */
        for (int d = 0; d < 5; d++) {
            int8_t nx = current->x + dx[d];
            int8_t ny = current->y + dy[d];
            uint8_t nt = current->t + 1;

            /* 边界检查 */
            if (nx < 0 || nx >= GRID_W || ny < 0 || ny >= GRID_H) continue;

            /* 障碍物检查 */
            if (grid[ny][nx] == CELL_OBSTACLE) continue;

            /* 约束检查 */
            if (constrained[ny][nx][nt]) continue;

            /* closed检查 */
            if (g_closed[ny][nx][nt]) continue;

            g_closed[ny][nx][nt] = 1;

            if (node_count >= GRID_W * GRID_H * 2) continue;

            g_astar_nodes[node_count] = (astar_node_t){
                .x = nx, .y = ny,
                .g_cost = current->g_cost + 1,
                .parent_idx = best_idx,
                .t = nt,
                .valid = 1
            };
            node_count++;
            open_count++;
        }
    }

    return -1;  /* 无解 */
}

/* ============================================================
 * CBS 高层算法
 * ============================================================ */

static int detect_conflict(const spacetime_path_t *paths, uint8_t num_agents,
                           conflict_t *out)
{
    for (uint8_t i = 0; i < num_agents; i++) {
        for (uint8_t j = i + 1; j < num_agents; j++) {
            uint8_t max_t = paths[i].len > paths[j].len ?
                           paths[i].len : paths[j].len;
            for (uint8_t t = 0; t < max_t; t++) {
                int8_t ax, ay, bx, by;
                get_position_at_time(&paths[i], t, &ax, &ay);
                get_position_at_time(&paths[j], t, &bx, &by);
                if (ax == bx && ay == by) {
                    out->agv_a = i;
                    out->agv_b = j;
                    out->x = ax;
                    out->y = ay;
                    out->timestep = t;
                    return 1;
                }
            }
        }
    }
    return 0;
}

/* CT节点池操作 */
static void ct_pool_init(void)
{
    memset(g_ct_nodes, 0, sizeof(g_ct_nodes));
    g_open_count = 0;
    g_next_free = 0;
}

static ct_node_t *ct_pool_alloc(void)
{
    if (g_next_free >= MAX_CT_NODES) return NULL;
    return &g_ct_nodes[g_next_free++];
}

static void ct_pool_push_open(uint16_t idx)
{
    if (g_open_count >= MAX_CT_NODES) return;
    g_open_list[g_open_count++] = idx;
}

static uint16_t ct_pool_pop_open(void)
{
    uint16_t best_idx = 0;
    int32_t best_cost = INT32_MAX;

    for (uint8_t i = 0; i < g_open_count; i++) {
        ct_node_t *node = &g_ct_nodes[g_open_list[i]];
        if (node->valid && node->cost < best_cost) {
            best_cost = node->cost;
            best_idx = i;
        }
    }

    uint16_t result = g_open_list[best_idx];
    /* 移除 */
    g_open_list[best_idx] = g_open_list[--g_open_count];
    return result;
}

/**
 * CBS主函数
 * 返回: 0=成功, -1=无解, -2=超时
 */
int cbs_solve(const uint8_t grid[GRID_H][GRID_W],
              const int16_t starts[][2],
              const int16_t goals[][2],
              uint8_t num_agents,
              spacetime_path_t *out_paths,
              uint32_t timeout_ms)
{
    uint32_t start_tick = get_tick_ms();

    ct_pool_init();

    /* 根节点：无约束，各AGV独立规划 */
    ct_node_t *root = ct_pool_alloc();
    if (!root) return -2;

    root->num_agents = num_agents;
    root->num_constraints = 0;
    root->cost = 0;
    root->valid = 1;

    for (uint8_t i = 0; i < num_agents; i++) {
        root->paths[i].agv_id = i;
        if (a_star_single(grid, starts[i][0], starts[i][1],
                          goals[i][0], goals[i][1], i,
                          NULL, 0, &root->paths[i]) != 0) {
            return -1;  /* 单体无解 */
        }
        root->cost += root->paths[i].len;
    }

    uint16_t root_idx = (uint16_t)(root - g_ct_nodes);
    ct_pool_push_open(root_idx);

    uint32_t iterations = 0;

    while (g_open_count > 0) {
        /* 超时检查 */
        if (get_tick_ms() - start_tick > timeout_ms) {
            printf("[scheduler] CBS超时! 迭代=%u\n", iterations);
            return -2;
        }

        iterations++;

        uint16_t best_idx = ct_pool_pop_open();
        ct_node_t *best = &g_ct_nodes[best_idx];
        if (!best->valid) continue;

        /* 检测冲突 */
        conflict_t conflict;
        if (!detect_conflict(best->paths, num_agents, &conflict)) {
            /* 无冲突，找到解! */
            memcpy(out_paths, best->paths,
                   sizeof(spacetime_path_t) * num_agents);
            uint32_t elapsed = get_tick_ms() - start_tick;
            printf("[scheduler] CBS求解成功! 耗时=%ums 迭代=%u 代价=%d\n",
                   elapsed, iterations, best->cost);
            return 0;
        }

        /* 分裂：为冲突双方各创建一个子节点 */
        for (int side = 0; side < 2; side++) {
            ct_node_t *child = ct_pool_alloc();
            if (!child) continue;

            memcpy(child, best, sizeof(ct_node_t));
            child->valid = 1;

            uint8_t constrained_agv = (side == 0) ?
                                      conflict.agv_a : conflict.agv_b;

            /* 添加约束 */
            if (child->num_constraints < MAX_CONSTRAINTS) {
                child->constraints[child->num_constraints++] =
                    (constraint_t){
                        .agv_id = constrained_agv,
                        .x = conflict.x,
                        .y = conflict.y,
                        .timestep = conflict.timestep
                    };
            }

            /* 低层：重新规划被约束AGV的路径 */
            if (a_star_single(grid,
                              starts[constrained_agv][0],
                              starts[constrained_agv][1],
                              goals[constrained_agv][0],
                              goals[constrained_agv][1],
                              constrained_agv,
                              child->constraints,
                              child->num_constraints,
                              &child->paths[constrained_agv]) == 0) {
                /* 重新计算总代价 */
                child->cost = 0;
                for (uint8_t i = 0; i < num_agents; i++)
                    child->cost += child->paths[i].len;

                uint16_t child_idx = (uint16_t)(child - g_ct_nodes);
                ct_pool_push_open(child_idx);
            } else {
                child->valid = 0;
            }
        }
    }

    printf("[scheduler] CBS无解! 迭代=%u\n", iterations);
    return -1;
}

/* ============================================================
 * 降级策略：优先级规划
 * ============================================================ */

static int priority_solve(const uint8_t grid[GRID_H][GRID_W],
                          const int16_t starts[][2],
                          const int16_t goals[][2],
                          uint8_t num_agents,
                          spacetime_path_t *out_paths)
{
    printf("[scheduler] 降级为优先级规划...\n");

    /* 简单策略：按顺序规划，不考虑碰撞 */
    for (uint8_t i = 0; i < num_agents; i++) {
        out_paths[i].agv_id = i;
        if (a_star_single(grid, starts[i][0], starts[i][1],
                          goals[i][0], goals[i][1], i,
                          NULL, 0, &out_paths[i]) != 0) {
            return -1;
        }
    }

    printf("[scheduler] 优先级规划完成\n");
    return 0;
}

/* ============================================================
 * 任务管理
 * ============================================================ */

static uint8_t g_next_task_id = 1;

static int add_task(int16_t goal_x, int16_t goal_y, uint8_t priority)
{
    pthread_mutex_lock(&g_shm->lock);

    /* 找空闲slot */
    int slot = -1;
    for (int i = 0; i < MAX_TASKS; i++) {
        if (g_shm->state.task_queue[i].state == TASK_DONE ||
            g_shm->state.task_queue[i].task_id == 0) {
            slot = i;
            break;
        }
    }

    if (slot < 0) {
        pthread_mutex_unlock(&g_shm->lock);
        printf("[scheduler] 任务队列已满!\n");
        return -1;
    }

    /* 找空闲AGV */
    uint8_t assigned_agv = 0;
    for (int i = 0; i < MAX_AGVS; i++) {
        agv_state_t *agv = &g_shm->state.agvs[i];
        if (agv->online && agv->task_id == 0 &&
            agv->status == AGV_STATUS_IDLE) {
            assigned_agv = agv->agv_id;
            break;
        }
    }

    if (assigned_agv == 0) {
        pthread_mutex_unlock(&g_shm->lock);
        printf("[scheduler] 无空闲AGV可分配!\n");
        return -1;
    }

    task_t *task = &g_shm->state.task_queue[slot];
    task->task_id = g_next_task_id++;
    task->agv_id = assigned_agv;
    task->start_x = g_shm->state.agvs[assigned_agv - 1].pos_x;
    task->start_y = g_shm->state.agvs[assigned_agv - 1].pos_y;
    task->goal_x = goal_x;
    task->goal_y = goal_y;
    task->priority = priority;
    task->state = TASK_PENDING;

    g_shm->state.num_tasks++;
    g_shm->version++;

    pthread_mutex_unlock(&g_shm->lock);

    printf("[scheduler] 新任务#%d: AGV#%d (%d,%d)->(%d,%d)\n",
           task->task_id, assigned_agv,
           task->start_x, task->start_y, goal_x, goal_y);

    return task->task_id;
}

/* ============================================================
 * 调度主循环线程
 * ============================================================ */

static void *scheduler_thread(void *arg)
{
    (void)arg;
    printf("[scheduler] 调度主循环启动\n");

    while (g_running) {
        usleep(100000);  /* 100ms */

        pthread_mutex_lock(&g_shm->lock);

        /* 检查是否有待处理任务 */
        for (int i = 0; i < MAX_TASKS; i++) {
            task_t *task = &g_shm->state.task_queue[i];
            if (task->state != TASK_PENDING) continue;

            task->state = TASK_PLANNING;
            uint8_t agv_id = task->agv_id;
            if (agv_id == 0 || agv_id > MAX_AGVS) {
                task->state = TASK_FAILED;
                continue;
            }

            /* 准备CBS输入 */
            int16_t starts[1][2] = {{task->start_x, task->start_y}};
            int16_t goals[1][2] = {{task->goal_x, task->goal_y}};
            spacetime_path_t paths[1];

            pthread_mutex_unlock(&g_shm->lock);

            /* 单AGV直接A* (无需CBS冲突检测) */
            int rc = a_star_single(g_shm->state.grid_map,
                                   starts[0][0], starts[0][1],
                                   goals[0][0], goals[0][1],
                                   agv_id - 1,
                                   NULL, 0, &paths[0]);

            pthread_mutex_lock(&g_shm->lock);

            if (rc == 0) {
                task->state = TASK_EXECUTING;
                g_shm->state.agvs[agv_id - 1].task_id = task->task_id;

                /* 通过消息队列发送路径给comm_gw */
                agv_command_t cmd;
                memset(&cmd, 0, sizeof(cmd));
                cmd.agv_id = agv_id;
                cmd.cmd_type = CMD_FOLLOW_PATH;
                cmd.target_x = task->goal_x;
                cmd.target_y = task->goal_y;
                cmd.speed_pct = 80;
                cmd.path_len = paths[0].len > 32 ? 32 : paths[0].len;
                for (uint8_t p = 0; p < cmd.path_len; p++) {
                    cmd.path[p][0] = paths[0].points[p].x;
                    cmd.path[p][1] = paths[0].points[p].y;
                }

                pthread_mutex_unlock(&g_shm->lock);
                mq_send(g_mq_cmd, (const char *)&cmd, sizeof(cmd), 0);
                pthread_mutex_lock(&g_shm->lock);

                printf("[scheduler] 任务#%d 路径已下发 AGV#%d (%d步)\n",
                       task->task_id, agv_id, cmd.path_len);
            } else {
                task->state = TASK_FAILED;
                printf("[scheduler] 任务#%d 规划失败!\n", task->task_id);
            }
        }

        /* 检查任务完成状态 */
        for (int i = 0; i < MAX_TASKS; i++) {
            task_t *task = &g_shm->state.task_queue[i];
            if (task->state != TASK_EXECUTING) continue;

            uint8_t agv_id = task->agv_id;
            agv_state_t *agv = &g_shm->state.agvs[agv_id - 1];

            /* 检查AGV是否到达目标 (距离<2格) */
            int dist = abs_int(agv->pos_x - task->goal_x) +
                       abs_int(agv->pos_y - task->goal_y);
            if (dist < 2 && agv->speed == 0) {
                task->state = TASK_DONE;
                agv->task_id = 0;
                g_shm->state.num_tasks--;
                printf("[scheduler] 任务#%d 完成! AGV#%d 到达(%d,%d)\n",
                       task->task_id, agv_id, agv->pos_x, agv->pos_y);
            }
        }

        g_shm->version++;
        pthread_mutex_unlock(&g_shm->lock);
    }

    printf("[scheduler] 调度主循环退出\n");
    return NULL;
}

/* ============================================================
 * 多AGV CBS调度 (当有多台AGV同时有任务时)
 * ============================================================ */

/**
 * 检查是否有多个pending/executing任务需要CBS协调
 * 如果是，调用CBS规划无碰撞路径
 */
static void check_multi_agv_conflicts(void)
{
    /* 收集所有活跃任务的AGV */
    uint8_t active_agvs[MAX_AGVS];
    int16_t starts[MAX_AGVS][2];
    int16_t goals_arr[MAX_AGVS][2];
    uint8_t count = 0;

    pthread_mutex_lock(&g_shm->lock);

    for (int i = 0; i < MAX_TASKS && count < MAX_AGVS; i++) {
        task_t *task = &g_shm->state.task_queue[i];
        if (task->state == TASK_EXECUTING ||
            task->state == TASK_PENDING) {
            uint8_t aid = task->agv_id;
            if (aid > 0 && aid <= MAX_AGVS) {
                active_agvs[count] = aid - 1;
                starts[count][0] = g_shm->state.agvs[aid - 1].pos_x;
                starts[count][1] = g_shm->state.agvs[aid - 1].pos_y;
                goals_arr[count][0] = task->goal_x;
                goals_arr[count][1] = task->goal_y;
                count++;
            }
        }
    }

    pthread_mutex_unlock(&g_shm->lock);

    /* 只有2+AGV才需要CBS */
    if (count < 2) return;

    /* 运行CBS */
    spacetime_path_t paths[MAX_AGVS];
    int rc = cbs_solve(g_shm->state.grid_map,
                       (const int16_t (*)[2])starts,
                       (const int16_t (*)[2])goals_arr,
                       count, paths, 100);

    if (rc == -2) {
        /* 超时，降级为优先级规划 */
        priority_solve(g_shm->state.grid_map,
                       (const int16_t (*)[2])starts,
                       (const int16_t (*)[2])goals_arr,
                       count, paths);
    }

    /* 下发路径 */
    for (uint8_t i = 0; i < count; i++) {
        agv_command_t cmd;
        memset(&cmd, 0, sizeof(cmd));
        cmd.agv_id = active_agvs[i] + 1;
        cmd.cmd_type = CMD_FOLLOW_PATH;
        cmd.target_x = goals_arr[i][0];
        cmd.target_y = goals_arr[i][1];
        cmd.speed_pct = 80;
        cmd.path_len = paths[i].len > 32 ? 32 : paths[i].len;
        for (uint8_t p = 0; p < cmd.path_len; p++) {
            cmd.path[p][0] = paths[i].points[p].x;
            cmd.path[p][1] = paths[i].points[p].y;
        }
        mq_send(g_mq_cmd, (const char *)&cmd, sizeof(cmd), 0);
    }
}

/* ============================================================
 * 主函数
 * ============================================================ */

int main(int argc, char *argv[])
{
    printf("╔══════════════════════════════════════════════╗\n");
    printf("║    云仓灵枢 - CBS调度引擎 (scheduler)        ║\n");
    printf("╚══════════════════════════════════════════════╝\n");

    /* 连接共享内存 */
    if (shm_connect() != 0) {
        printf("[scheduler] 无法连接共享内存, 请先启动 comm_gw\n");
        return -1;
    }

    /* 打开消息队列 */
    struct mq_attr mq_attr = {
        .mq_flags = 0,
        .mq_maxmsg = 16,
        .mq_msgsize = sizeof(agv_command_t),
        .mq_curmsgs = 0
    };

    g_mq_cmd = mq_open("/mq_cmd_gw", O_CREAT | O_WRONLY, 0666, &mq_attr);
    if (g_mq_cmd == (mqd_t)-1) {
        printf("[scheduler] 打开指令队列失败\n");
        return -1;
    }

    g_mq_alert = mq_open("/mq_alert", O_CREAT | O_WRONLY, 0666, &mq_attr);
    if (g_mq_alert == (mqd_t)-1) {
        printf("[scheduler] 打开告警队列失败 (非致命)\n");
    }

    printf("[scheduler] 初始化完成\n");

    /* 启动调度线程 */
    pthread_t tid_scheduler;
    pthread_create(&tid_scheduler, NULL, scheduler_thread, NULL);

    /* 主线程：定期检查多AGV冲突 */
    while (g_running) {
        sleep(2);
        check_multi_agv_conflicts();
    }

    pthread_join(tid_scheduler, NULL);

    mq_close(g_mq_cmd);
    mq_close(g_mq_alert);
    munmap(g_shm, sizeof(shm_agv_t));

    printf("[scheduler] 已退出\n");
    return 0;
}
