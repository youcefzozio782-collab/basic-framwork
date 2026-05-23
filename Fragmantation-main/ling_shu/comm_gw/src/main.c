/**
 * comm_gw - 云仓灵枢 通信网关进程
 *
 * 运行在K230 (RT-Thread Smart) 上，负责：
 * 1. UDP收发：接收AGV状态上报，下发服务器指令
 * 2. 协议解析：帧校验(Magic+CRC16)、消息类型分发
 * 3. 心跳检测：500ms周期检测，超时标记AGV离线
 * 4. 包防御：IP白名单、限流、畸形包过滤
 * 5. 共享内存：将AGV状态写入共享内存供scheduler和web_monitor读取
 */

#include <rtthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <pthread.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <errno.h>

#include "agv_protocol.h"

/* ============================================================
 * 全局变量
 * ============================================================ */

static shm_agv_t *g_shm = NULL;            /* 共享内存指针 */
static int g_udp_status_fd = -1;            /* 状态接收socket */
static int g_udp_cmd_fd = -1;               /* 指令发送socket */
static volatile int g_running = 1;          /* 运行标志 */

/* AGV IP白名单 */
static const char *g_agv_ips[MAX_AGVS] = {
    "192.168.4.10",     /* AGV #1 */
    "192.168.4.11",     /* AGV #2 */
    "192.168.4.12",     /* AGV #3 */
};

/* 限流状态 */
typedef struct {
    struct in_addr ip;
    uint32_t pkt_count;
    uint32_t window_start;
} rate_limit_t;

static rate_limit_t g_agv_rate[MAX_AGVS];
static uint32_t g_unknown_pkt_count = 0;
static uint32_t g_unknown_window_start = 0;

/* 统计 */
static uint32_t g_stat_rx_ok = 0;
static uint32_t g_stat_rx_drop = 0;
static uint32_t g_stat_tx = 0;

/* ============================================================
 * CRC16-CCITT
 * ============================================================ */

uint16_t crc16_ccitt(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc <<= 1;
        }
    }
    return crc;
}

/* ============================================================
 * 时间戳
 * ============================================================ */

uint32_t get_tick_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)(ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
}

/* ============================================================
 * 共享内存操作
 * ============================================================ */

static int shm_init(void)
{
    int fd = shm_open(SHM_NAME, O_CREAT | O_RDWR, 0666);
    if (fd < 0) {
        printf("[comm_gw] shm_open failed: %s\n", strerror(errno));
        return -1;
    }

    if (ftruncate(fd, sizeof(shm_agv_t)) < 0) {
        printf("[comm_gw] ftruncate failed\n");
        close(fd);
        return -1;
    }

    g_shm = (shm_agv_t *)mmap(NULL, sizeof(shm_agv_t),
                               PROT_READ | PROT_WRITE,
                               MAP_SHARED, fd, 0);
    if (g_shm == MAP_FAILED) {
        printf("[comm_gw] mmap failed\n");
        close(fd);
        return -1;
    }

    /* 初始化互斥锁 (进程间共享) */
    pthread_mutexattr_t attr;
    pthread_mutexattr_init(&attr);
    pthread_mutexattr_setpshared(&attr, PTHREAD_PROCESS_SHARED);
    pthread_mutex_init(&g_shm->lock, &attr);
    pthread_mutexattr_destroy(&attr);

    /* 清零状态 */
    memset(&g_shm->state, 0, sizeof(shared_state_t));
    g_shm->version = 0;

    /* 初始化默认地图 */
    for (int y = 0; y < GRID_H; y++)
        for (int x = 0; x < GRID_W; x++)
            g_shm->state.grid_map[y][x] = CELL_EMPTY;

    /* 添加示例障碍物 (货架) */
    for (int y = 4; y < 8; y++)
        for (int x = 4; x < 8; x++)
            g_shm->state.grid_map[y][x] = CELL_OBSTACLE;
    for (int y = 4; y < 8; y++)
        for (int x = 24; x < 28; x++)
            g_shm->state.grid_map[y][x] = CELL_OBSTACLE;
    for (int y = 14; y < 18; y++)
        for (int x = 12; x < 20; x++)
            g_shm->state.grid_map[y][x] = CELL_OBSTACLE;

    g_shm->state.num_agvs = ACTIVE_AGVS;

    close(fd);  /* mmap后可关闭fd */
    printf("[comm_gw] 共享内存初始化完成\n");
    return 0;
}

/* 写入AGV状态到共享内存 */
static void shm_update_agv(uint8_t agv_id, const agv_status_payload_t *status)
{
    if (agv_id < 1 || agv_id > MAX_AGVS) return;

    pthread_mutex_lock(&g_shm->lock);

    agv_state_t *agv = &g_shm->state.agvs[agv_id - 1];
    agv->agv_id = agv_id;
    agv->pos_x = status->pos_x;
    agv->pos_y = status->pos_y;
    agv->heading = status->heading;
    agv->speed = (uint16_t)(abs(status->velocity_x) + abs(status->velocity_y));
    agv->battery = status->battery_pct;
    agv->timestamp = get_tick_ms();
    agv->task_id = status->current_task_id;
    agv->online = 1;
    agv->heartbeat_miss = 0;

    /* 根据速度判断状态 */
    if (agv->speed > 0)
        agv->status = AGV_STATUS_MOVING;
    else if (agv->task_id == 0)
        agv->status = AGV_STATUS_IDLE;

    g_shm->version++;
    g_shm->state.tick = get_tick_ms();

    pthread_mutex_unlock(&g_shm->lock);
}

/* ============================================================
 * 包防御
 * ============================================================ */

/* IP转AGV索引，-1表示未知 */
static int ip_to_agv_index(struct in_addr ip)
{
    for (int i = 0; i < MAX_AGVS; i++) {
        if (g_agv_rate[i].ip.s_addr == ip.s_addr)
            return i;
    }
    return -1;
}

/* 注册AGV IP */
static void register_agv_ip(int idx, struct in_addr ip)
{
    if (idx >= 0 && idx < MAX_AGVS) {
        g_agv_rate[idx].ip = ip;
    }
}

/* 包防御检查，返回0表示通过 */
static int packet_defense(struct sockaddr_in *src, uint8_t *buf, size_t len)
{
    uint32_t now = get_tick_ms();

    /* 1. 大小检查 */
    if (len < 8 || len > 512)
        return -1;

    /* 2. Magic检查 */
    if (buf[0] != AGV_MAGIC_0 || buf[1] != AGV_MAGIC_1)
        return -2;

    /* 3. 来源IP白名单检查 */
    int agv_idx = ip_to_agv_index(src->sin_addr);

    if (agv_idx < 0) {
        /* 检查是否在白名单中 */
        for (int i = 0; i < ACTIVE_AGVS; i++) {
            struct in_addr expected;
            inet_aton(g_agv_ips[i], &expected);
            if (expected.s_addr == src->sin_addr.s_addr) {
                register_agv_ip(i, src->sin_addr);
                agv_idx = i;
                break;
            }
        }
    }

    if (agv_idx < 0) {
        /* 未知来源，限流 */
        if (now - g_unknown_window_start > RATE_WINDOW_MS) {
            g_unknown_pkt_count = 0;
            g_unknown_window_start = now;
        }
        if (++g_unknown_pkt_count > MAX_UNKNOWN_PER_SEC)
            return -3;
        return -4;  /* 未知来源，丢弃 */
    }

    /* 4. 已知AGV限流 */
    if (now - g_agv_rate[agv_idx].window_start > RATE_WINDOW_MS) {
        g_agv_rate[agv_idx].pkt_count = 0;
        g_agv_rate[agv_idx].window_start = now;
    }
    if (++g_agv_rate[agv_idx].pkt_count > MAX_PKT_PER_SEC)
        return -5;

    /* 5. PayloadLen校验 */
    uint16_t payload_len = buf[4] | (buf[5] << 8);
    if (payload_len > 400)
        return -6;

    /* 6. CRC校验 */
    uint16_t frame_body_len = 4 + payload_len;  /* SeqNo+MsgType+PayloadLen+Payload */
    if (len < (size_t)(8 + payload_len))
        return -7;

    uint16_t recv_crc = buf[6 + payload_len] | (buf[7 + payload_len] << 8);
    uint16_t calc_crc = crc16_ccitt(buf + 2, frame_body_len);
    if (recv_crc != calc_crc)
        return -8;

    return 0;  /* 通过 */
}

/* ============================================================
 * 协议解析与处理
 * ============================================================ */

static void handle_status_report(struct sockaddr_in *src,
                                 uint8_t *payload, uint16_t len)
{
    if (len < sizeof(agv_status_payload_t)) return;

    agv_status_payload_t *status = (agv_status_payload_t *)payload;
    uint8_t agv_id = status->agv_id;

    if (agv_id < 1 || agv_id > MAX_AGVS) return;

    /* 注册IP (首次收到) */
    int idx = agv_id - 1;
    if (g_agv_rate[idx].ip.s_addr == 0) {
        register_agv_ip(idx, src->sin_addr);
        printf("[comm_gw] AGV#%d 已注册 IP: %s\n",
               agv_id, inet_ntoa(src->sin_addr));
    }

    /* 更新共享内存 */
    shm_update_agv(agv_id, status);
    g_stat_rx_ok++;
}

static void handle_heartbeat_resp(struct sockaddr_in *src,
                                  uint8_t *payload, uint16_t len)
{
    if (len < 1) return;
    uint8_t agv_id = payload[0];
    if (agv_id < 1 || agv_id > MAX_AGVS) return;

    /* 重置心跳丢失计数 */
    pthread_mutex_lock(&g_shm->lock);
    g_shm->state.agvs[agv_id - 1].heartbeat_miss = 0;
    g_shm->state.agvs[agv_id - 1].online = 1;
    pthread_mutex_unlock(&g_shm->lock);
}

static void process_frame(struct sockaddr_in *src, uint8_t *buf, size_t len)
{
    uint8_t seq_no = buf[2];
    uint8_t msg_type = buf[3];
    uint16_t payload_len = buf[4] | (buf[5] << 8);
    uint8_t *payload = buf + 6;

    (void)seq_no;  /* TODO: 用于检测乱序/丢包 */

    switch (msg_type) {
    case MSG_AGV_STATUS:
        handle_status_report(src, payload, payload_len);
        break;

    case MSG_HEARTBEAT_RESP:
        handle_heartbeat_resp(src, payload, payload_len);
        break;

    case MSG_TASK_ACK:
        /* TODO: 通知scheduler任务确认 */
        break;

    default:
        break;
    }
}

/* ============================================================
 * 接收线程
 * ============================================================ */

static void *recv_thread(void *arg)
{
    (void)arg;
    uint8_t buf[1024];
    struct sockaddr_in src;
    socklen_t src_len;

    printf("[comm_gw] 接收线程启动, 监听 UDP:%d\n", PORT_AGV_STATUS);

    while (g_running) {
        src_len = sizeof(src);
        ssize_t n = recvfrom(g_udp_status_fd, buf, sizeof(buf), 0,
                             (struct sockaddr *)&src, &src_len);
        if (n < 0) {
            if (errno == EINTR) continue;
            break;
        }

        /* 包防御检查 */
        int rc = packet_defense(&src, buf, (size_t)n);
        if (rc != 0) {
            g_stat_rx_drop++;
            continue;
        }

        /* 协议处理 */
        process_frame(&src, buf, (size_t)n);
    }

    printf("[comm_gw] 接收线程退出\n");
    return NULL;
}

/* ============================================================
 * 心跳检测线程
 * ============================================================ */

static void *heartbeat_thread(void *arg)
{
    (void)arg;
    printf("[comm_gw] 心跳检测线程启动, 间隔 %dms\n",
           HEARTBEAT_INTERVAL_MS);

    while (g_running) {
        usleep(HEARTBEAT_INTERVAL_MS * 1000);

        pthread_mutex_lock(&g_shm->lock);

        for (int i = 0; i < MAX_AGVS; i++) {
            agv_state_t *agv = &g_shm->state.agvs[i];
            if (agv->agv_id == 0) continue;  /* 未注册 */

            agv->heartbeat_miss++;

            if (agv->heartbeat_miss >= HEARTBEAT_MISS_MAX) {
                if (agv->online) {
                    agv->online = 0;
                    agv->status = AGV_STATUS_ERROR;
                    printf("[comm_gw] AGV#%d 离线! (心跳超时)\n",
                           agv->agv_id);
                }
            }
        }

        g_shm->version++;
        pthread_mutex_unlock(&g_shm->lock);

        /* 发送心跳请求给所有在线AGV */
        for (int i = 0; i < ACTIVE_AGVS; i++) {
            struct in_addr ip;
            inet_aton(g_agv_ips[i], &ip);
            if (g_agv_rate[i].ip.s_addr == 0) continue;

            uint8_t payload[1] = { (uint8_t)(i + 1) };
            uint8_t frame[16];
            frame[0] = AGV_MAGIC_0;
            frame[1] = AGV_MAGIC_1;
            frame[2] = 0;  /* seq_no */
            frame[3] = MSG_HEARTBEAT_REQ;
            frame[4] = 1;  /* payload_len low */
            frame[5] = 0;  /* payload_len high */
            frame[6] = payload[0];
            uint16_t crc = crc16_ccitt(frame + 2, 5);
            frame[7] = crc & 0xFF;
            frame[8] = (crc >> 8) & 0xFF;

            struct sockaddr_in dst;
            memset(&dst, 0, sizeof(dst));
            dst.sin_family = AF_INET;
            dst.sin_addr = g_agv_rate[i].ip;
            dst.sin_port = htons(PORT_AGV_STATUS);

            sendto(g_udp_cmd_fd, frame, 9, 0,
                   (struct sockaddr *)&dst, sizeof(dst));
        }
    }

    printf("[comm_gw] 心跳检测线程退出\n");
    return NULL;
}

/* ============================================================
 * 统计打印线程
 * ============================================================ */

static void *stats_thread(void *arg)
{
    (void)arg;

    while (g_running) {
        sleep(5);
        printf("[comm_gw] 统计: RX_OK=%u RX_DROP=%u TX=%u\n",
               g_stat_rx_ok, g_stat_rx_drop, g_stat_tx);

        /* 打印AGV状态 */
        pthread_mutex_lock(&g_shm->lock);
        for (int i = 0; i < MAX_AGVS; i++) {
            agv_state_t *agv = &g_shm->state.agvs[i];
            if (agv->agv_id == 0) continue;
            printf("  AGV#%d: pos=(%d,%d) spd=%u bat=%u%% %s\n",
                   agv->agv_id, agv->pos_x, agv->pos_y,
                   agv->speed, agv->battery,
                   agv->online ? "ONLINE" : "OFFLINE");
        }
        pthread_mutex_unlock(&g_shm->lock);
    }

    return NULL;
}

/* ============================================================
 * 指令发送接口 (供scheduler通过消息队列调用)
 * ============================================================ */

int comm_gw_send_command(const agv_command_t *cmd)
{
    if (!cmd || cmd->agv_id < 1 || cmd->agv_id > MAX_AGVS)
        return -1;

    int idx = cmd->agv_id - 1;
    if (g_agv_rate[idx].ip.s_addr == 0) {
        printf("[comm_gw] AGV#%d 未注册, 无法发送指令\n", cmd->agv_id);
        return -1;
    }

    /* 构建指令帧 */
    uint8_t frame[512];
    uint16_t payload_len = 8 + cmd->path_len * 4;

    frame[0] = AGV_MAGIC_0;
    frame[1] = AGV_MAGIC_1;
    frame[2] = 0;
    frame[3] = MSG_SERVER_CMD;
    frame[4] = payload_len & 0xFF;
    frame[5] = (payload_len >> 8) & 0xFF;

    /* Payload */
    memcpy(frame + 6, cmd, 8);  /* agv_id + cmd_type + target + speed + path_len */
    if (cmd->path_len > 0) {
        memcpy(frame + 14, cmd->path, cmd->path_len * sizeof(path_point_xy_t));
    }

    /* CRC */
    uint16_t crc = crc16_ccitt(frame + 2, 4 + payload_len);
    frame[6 + payload_len] = crc & 0xFF;
    frame[7 + payload_len] = (crc >> 8) & 0xFF;

    /* 发送 */
    struct sockaddr_in dst;
    memset(&dst, 0, sizeof(dst));
    dst.sin_family = AF_INET;
    dst.sin_addr = g_agv_rate[idx].ip;
    dst.sin_port = htons(PORT_AGV_STATUS);

    ssize_t sent = sendto(g_udp_cmd_fd, frame, 8 + payload_len, 0,
                          (struct sockaddr *)&dst, sizeof(dst));

    if (sent > 0) g_stat_tx++;

    return (sent > 0) ? 0 : -1;
}

/* ============================================================
 * 主函数
 * ============================================================ */

int main(int argc, char *argv[])
{
    printf("╔══════════════════════════════════════════════╗\n");
    printf("║    云仓灵枢 - 通信网关 (comm_gw)             ║\n");
    printf("╚══════════════════════════════════════════════╝\n");

    /* 初始化共享内存 */
    if (shm_init() != 0) {
        printf("[comm_gw] 共享内存初始化失败!\n");
        return -1;
    }

    /* 创建UDP socket - 状态接收 */
    g_udp_status_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (g_udp_status_fd < 0) {
        printf("[comm_gw] 创建socket失败\n");
        return -1;
    }

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(PORT_AGV_STATUS);

    if (bind(g_udp_status_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        printf("[comm_gw] bind失败: %s\n", strerror(errno));
        return -1;
    }

    /* 创建UDP socket - 指令发送 */
    g_udp_cmd_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (g_udp_cmd_fd < 0) {
        printf("[comm_gw] 创建指令socket失败\n");
        return -1;
    }

    printf("[comm_gw] 初始化完成, 启动线程...\n");

    /* 启动线程 */
    pthread_t tid_recv, tid_heartbeat, tid_stats;
    pthread_create(&tid_recv, NULL, recv_thread, NULL);
    pthread_create(&tid_heartbeat, NULL, heartbeat_thread, NULL);
    pthread_create(&tid_stats, NULL, stats_thread, NULL);

    /* 主线程等待 */
    while (g_running) {
        sleep(1);
    }

    /* 清理 */
    pthread_join(tid_recv, NULL);
    pthread_join(tid_heartbeat, NULL);
    pthread_join(tid_stats, NULL);

    close(g_udp_status_fd);
    close(g_udp_cmd_fd);
    munmap(g_shm, sizeof(shm_agv_t));

    printf("[comm_gw] 已退出\n");
    return 0;
}
