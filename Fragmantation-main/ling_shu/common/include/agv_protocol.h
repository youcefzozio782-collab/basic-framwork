/**
 * agv_protocol.h - 云仓灵枢 AGV调度系统 公共协议定义
 *
 * 本文件定义了K230边缘服务器与AGV小车之间的通信协议，
 * 以及K230内部三进程间共享的数据结构。
 * 所有组件（comm_gw, scheduler, web_monitor, agv_firmware）都依赖此文件。
 */

#ifndef AGV_PROTOCOL_H
#define AGV_PROTOCOL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 1. 系统常量
 * ============================================================ */

#define AGV_MAGIC_0         0xAA        /* 帧同步字节1 */
#define AGV_MAGIC_1         0x55        /* 帧同步字节2 */

#define MAX_AGVS            5           /* 最大AGV数量 */
#define ACTIVE_AGVS         3           /* 当前活跃AGV数量 */

#define GRID_W              32          /* 栅格地图宽度 */
#define GRID_H              32          /* 栅格地图高度 */

#define MAX_PATH_LEN        64          /* 单条路径最大点数 */
#define MAX_TIMESTEP        64          /* CBS最大时间步 */
#define MAX_CT_NODES        128         /* CBS约束树节点池大小 */
#define MAX_TASKS           16          /* 任务队列最大长度 */
#define MAX_CONSTRAINTS     16          /* 单CT节点最大约束数 */

/* 网络端口 */
#define PORT_AGV_STATUS     5000        /* AGV状态上报端口 (UDP) */
#define PORT_AGV_CMD        5001        /* 服务器指令下发端口 (UDP) */
#define PORT_WEB_HTTP       8080        /* Web监控HTTP端口 */

/* 心跳与超时 */
#define HEARTBEAT_INTERVAL_MS   500     /* 心跳检测间隔 */
#define HEARTBEAT_TIMEOUT_MS    2000    /* 心跳超时阈值 (4个周期) */
#define HEARTBEAT_MISS_MAX      4       /* 最大丢失次数 */

/* 限流参数 */
#define MAX_PKT_PER_SEC     200         /* 每AGV每秒最大包数 */
#define MAX_UNKNOWN_PER_SEC 50          /* 每秒最大未知源包数 */
#define RATE_WINDOW_MS      1000        /* 限流窗口 */

/* WebSocket推送间隔 */
#define WS_PUSH_INTERVAL_MS 200         /* 200ms = 5fps */

/* ============================================================
 * 2. 协议帧格式
 * ============================================================
 *
 * 帧结构:
 * +--------+--------+--------+-----------+---------+--------+
 * | Magic  | SeqNo  |MsgType | PayloadLen| Payload | CRC16  |
 * | 2B     | 1B     | 1B     |   2B      |   NB    |  2B    |
 * +--------+--------+--------+-----------+---------+--------+
 *
 * 总开销: 8字节头尾 + Payload
 */

/* 消息类型定义 */
typedef enum {
    MSG_AGV_STATUS      = 0x01,     /* AGV -> Server: 状态上报 */
    MSG_SERVER_CMD      = 0x02,     /* Server -> AGV: 指令下发 */
    MSG_HEARTBEAT_REQ   = 0x03,     /* Server -> AGV: 心跳请求 */
    MSG_HEARTBEAT_RESP  = 0x04,     /* AGV -> Server: 心跳响应 */
    MSG_EMERGENCY_STOP  = 0x05,     /* Server -> AGV: 急停(广播) */
    MSG_TASK_ACK        = 0x06,     /* AGV -> Server: 任务确认 */
    MSG_CONFIG_UPDATE   = 0x07,     /* Server -> AGV: 配置更新 */
} msg_type_t;

/* 帧头 (固定8字节) */
typedef struct {
    uint8_t  magic[2];              /* 0xAA 0x55 */
    uint8_t  seq_no;                /* 序列号 0-255 循环 */
    uint8_t  msg_type;              /* msg_type_t */
    uint16_t payload_len;           /* Payload长度 (小端) */
} __attribute__((packed)) frame_header_t;

/* ============================================================
 * 3. AGV状态上报 (AGV -> Server, MSG_AGV_STATUS)
 * ============================================================ */

/* AGV运行状态 */
typedef enum {
    AGV_STATUS_IDLE     = 0,        /* 空闲 */
    AGV_STATUS_MOVING   = 1,        /* 移动中 */
    AGV_STATUS_CHARGING = 2,        /* 充电中 */
    AGV_STATUS_ERROR    = 3,        /* 故障 */
} agv_run_status_t;

/* AGV状态上报Payload (19字节) */
typedef struct {
    uint8_t  agv_id;                /* AGV编号 1-MAX_AGVS */
    int16_t  pos_x;                 /* X坐标 (栅格) */
    int16_t  pos_y;                 /* Y坐标 (栅格) */
    int16_t  velocity_x;            /* X速度 mm/s */
    int16_t  velocity_y;            /* Y速度 mm/s */
    uint16_t heading;               /* 朝向 0-359度 */
    uint8_t  battery_pct;           /* 电量百分比 0-100 */
    uint8_t  motor_status;          /* bit0=左电机OK, bit1=右电机OK */
    uint8_t  obstacle;              /* bit0=前, bit1=左, bit2=右 */
    uint8_t  current_task_id;       /* 当前任务ID, 0=空闲 */
    uint32_t odometer;              /* 累计里程 mm */
} __attribute__((packed)) agv_status_payload_t;

/* ============================================================
 * 4. 服务器指令 (Server -> AGV, MSG_SERVER_CMD)
 * ============================================================ */

/* 指令类型 */
typedef enum {
    CMD_STOP        = 0,            /* 停止 */
    CMD_MOVE_TO     = 1,            /* 移动到目标点 */
    CMD_FOLLOW_PATH = 2,            /* 沿路径点序列移动 */
    CMD_DOCK_CHARGE = 3,            /* 去充电 */
} cmd_type_t;

/* 服务器指令Payload */
typedef struct {
    uint8_t  agv_id;                /* 目标AGV编号 */
    uint8_t  cmd_type;              /* cmd_type_t */
    int16_t  target_x;              /* 目标X坐标 */
    int16_t  target_y;              /* 目标Y坐标 */
    uint8_t  speed_pct;             /* 速度百分比 0-100 */
    uint8_t  path_len;              /* 路径点数量 (follow_path模式) */
    /* 后续跟 path_len 个 path_point_xy_t */
} __attribute__((packed)) server_cmd_payload_t;

/* 路径点 (仅x,y坐标) */
typedef struct {
    int16_t x;
    int16_t y;
} __attribute__((packed)) path_point_xy_t;

/* ============================================================
 * 5. 共享内存数据结构 (K230三进程共享)
 * ============================================================ */

/* 栅格地图单元类型 */
typedef enum {
    CELL_EMPTY      = 0,            /* 空地 */
    CELL_OBSTACLE   = 1,            /* 障碍物 */
    CELL_CHARGER    = 2,            /* 充电站 */
    CELL_SHELF      = 3,            /* 货架 */
} cell_type_t;

/* AGV实时状态 (写入共享内存) */
typedef struct {
    uint8_t  agv_id;
    uint8_t  status;                /* agv_run_status_t */
    int16_t  pos_x;
    int16_t  pos_y;
    uint16_t heading;               /* 0/90/180/270 */
    uint16_t speed;                 /* mm/s */
    uint8_t  battery;               /* % */
    uint32_t timestamp;             /* 最后更新时间 ms */
    uint8_t  task_id;               /* 当前任务ID */
    uint8_t  waypoint_idx;          /* 当前路径点索引 */
    uint8_t  online;                /* 1=在线, 0=离线 */
    uint8_t  heartbeat_miss;        /* 连续心跳丢失次数 */
} agv_state_t;

/* 任务定义 */
typedef struct {
    uint8_t  task_id;
    uint8_t  agv_id;                /* 分配给哪台AGV, 0=未分配 */
    int16_t  start_x, start_y;      /* 起点 */
    int16_t  goal_x, goal_y;        /* 终点 */
    uint8_t  priority;              /* 优先级 0-255 */
    uint8_t  state;                 /* task_state_t */
} task_t;

/* 任务状态 */
typedef enum {
    TASK_PENDING    = 0,            /* 等待调度 */
    TASK_PLANNING   = 1,            /* CBS规划中 */
    TASK_EXECUTING  = 2,            /* 执行中 */
    TASK_DONE       = 3,            /* 完成 */
    TASK_FAILED     = 4,            /* 失败 */
} task_state_t;

/* 全局共享状态 (三进程通过共享内存访问) */
typedef struct {
    agv_state_t  agvs[MAX_AGVS];    /* AGV实时状态表 */
    task_t       task_queue[MAX_TASKS]; /* 任务队列 */
    uint8_t      grid_map[GRID_H][GRID_W]; /* 栅格地图 */
    uint32_t     tick;              /* 全局时钟 ms */
    uint8_t      emergency_stop;    /* 急停标志 1=急停 */
    uint8_t      num_agvs;          /* 当前AGV数量 */
    uint8_t      num_tasks;         /* 当前任务数量 */
} shared_state_t;

/* 共享内存包装 (含互斥锁和版本号) */
typedef struct {
    pthread_mutex_t  lock;          /* 进程间互斥锁 */
    shared_state_t   state;         /* 核心状态数据 */
    uint64_t         version;       /* 版本号, 每次写入+1 */
} shm_agv_t;

#define SHM_NAME "/agv_state"       /* 共享内存名称 */

/* ============================================================
 * 6. CBS算法数据结构
 * ============================================================ */

/* 时空路径点 */
typedef struct {
    int8_t   x, y;                  /* 栅格坐标 */
    uint8_t  t;                     /* 时间步 */
} __attribute__((packed)) spacetime_point_t;

/* 单台AGV的时空路径 */
typedef struct {
    uint8_t          agv_id;
    uint8_t          len;            /* 路径点数量 */
    spacetime_point_t points[MAX_PATH_LEN];
} spacetime_path_t;

/* 冲突描述 */
typedef struct {
    uint8_t  agv_a, agv_b;          /* 冲突双方 */
    int8_t   x, y;                  /* 冲突位置 */
    uint8_t  timestep;              /* 冲突时间步 */
} conflict_t;

/* 约束: 某AGV在某时间步不能在某位置 */
typedef struct {
    uint8_t  agv_id;
    int8_t   x, y;
    uint8_t  timestep;
} constraint_t;

/* CBS约束树节点 */
typedef struct {
    constraint_t     constraints[MAX_CONSTRAINTS];
    uint8_t          num_constraints;
    spacetime_path_t paths[MAX_AGVS];
    uint8_t          num_agents;
    int32_t          cost;           /* 总路径代价 */
    uint16_t         parent;         /* 父节点索引 */
    uint8_t          valid;          /* 是否有效 */
} ct_node_t;

/* CBS节点池 (静态分配) */
typedef struct {
    ct_node_t  nodes[MAX_CT_NODES];
    uint16_t   open_list[MAX_CT_NODES]; /* 开放列表索引 */
    uint8_t    open_count;
    uint16_t   next_free;           /* 下一个空闲节点索引 */
} ct_pool_t;

/* ============================================================
 * 7. 告警与日志
 * ============================================================ */

/* 告警类型 */
typedef enum {
    ALERT_AGV_OFFLINE   = 0x01,     /* AGV离线 */
    ALERT_LOW_BATTERY   = 0x02,     /* 低电量 */
    ALERT_OBSTACLE      = 0x03,     /* 前方障碍 */
    ALERT_COLLISION_RISK= 0x04,     /* 碰撞风险 */
    ALERT_CBS_TIMEOUT   = 0x05,     /* CBS规划超时 */
    ALERT_EMERGENCY     = 0x06,     /* 急停触发 */
} alert_type_t;

/* 告警消息 (通过消息队列从scheduler发往web_monitor) */
typedef struct {
    uint8_t  alert_type;            /* alert_type_t */
    uint8_t  agv_id;                /* 相关AGV, 0=全局 */
    uint32_t timestamp;
    char     message[64];           /* 可读描述 */
} __attribute__((packed)) alert_msg_t;

/* ============================================================
 * 8. 工具函数声明
 * ============================================================ */

/**
 * CRC16-CCITT校验
 * @param data  数据指针
 * @param len   数据长度
 * @return      16位CRC值
 */
uint16_t crc16_ccitt(const uint8_t *data, uint16_t len);

/**
 * 获取当前时间戳 (毫秒)
 * @return      系统启动后的毫秒数
 */
uint32_t get_tick_ms(void);

/**
 * 计算曼哈顿距离
 * @return      |x1-x2| + |y1-y2|
 */
static inline int manhattan_dist(int x1, int y1, int x2, int y2) {
    return abs(x1 - x2) + abs(y1 - y2);
}

/**
 * 获取AGV在指定时间步的位置 (超出路径长度则停留在终点)
 */
void get_position_at_time(const spacetime_path_t *path, uint8_t t,
                          int8_t *out_x, int8_t *out_y);

#ifdef __cplusplus
}
#endif

#endif /* AGV_PROTOCOL_H */
