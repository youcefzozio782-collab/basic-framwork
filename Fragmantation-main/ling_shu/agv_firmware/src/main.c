/**
 * AGV固件 - 云仓灵枢 AGV小车端
 *
 * 运行在STM32F103 (RT-Thread Nano) 上，负责：
 * 1. ESP8266 WiFi通信：AT指令 + UDP收发
 * 2. 电机PID控制：左轮/右轮独立PID
 * 3. 编码器读取：里程计计算
 * 4. 协议帧编解码：与K230服务器通信
 * 5. 路径跟踪：沿CBS规划的路径点移动
 * 6. 避障：超声波/红外传感器
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <string.h>
#include <stdlib.h>

/* ============================================================
 * 硬件引脚定义 (根据实际硬件修改)
 * ============================================================ */

/* 电机PWM */
#define PWM_DEV_NAME        "pwm1"
#define PWM_CH_LEFT         1       /* 左电机PWM通道 */
#define PWM_CH_RIGHT        2       /* 右电机PWM通道 */
#define PWM_PERIOD          1000    /* PWM周期 us */

/* 电机方向GPIO */
#define DIR_LEFT_FWD        GET_PIN(A, 0)
#define DIR_LEFT_REV        GET_PIN(A, 1)
#define DIR_RIGHT_FWD       GET_PIN(A, 2)
#define DIR_RIGHT_REV       GET_PIN(A, 3)

/* 编码器 */
#define ENCODER_LEFT_TIM    "tim2"
#define ENCODER_RIGHT_TIM   "tim3"

/* 超声波 */
#define TRIG_PIN            GET_PIN(B, 0)
#define ECHO_PIN            GET_PIN(B, 1)

/* ESP8266 UART */
#define ESP_UART_NAME       "uart2"
#define ESP_BAUDRATE        115200

/* ============================================================
 * AGV配置
 * ============================================================ */

#define AGV_ID              1               /* 本机AGV编号 */
#define SERVER_IP           "192.168.4.1"   /* K230服务器IP */
#define SERVER_PORT         5000            /* 服务器端口 */
#define LOCAL_PORT          5001            /* 本地端口 */
#define WIFI_SSID           "AGV_Network"
#define WIFI_PASS           "password123"

#define STATUS_INTERVAL_MS  100             /* 状态上报间隔 */
#define CMD_TIMEOUT_MS      5000            /* 指令超时 */

/* PID参数 */
#define PID_KP              2.0f
#define PID_KI              0.5f
#define PID_KD              0.1f
#define PID_MAX_OUTPUT      900.0f
#define PID_MIN_OUTPUT      50.0f

/* ============================================================
 * 协议帧格式 (与 agv_protocol.h 一致)
 * ============================================================ */

#define AGV_MAGIC_0         0xAA
#define AGV_MAGIC_1         0x55
#define MSG_AGV_STATUS      0x01
#define MSG_SERVER_CMD      0x02
#define MSG_HEARTBEAT_REQ   0x03
#define MSG_HEARTBEAT_RESP  0x04
#define MSG_EMERGENCY_STOP  0x05

/* ============================================================
 * 数据结构
 * ============================================================ */

typedef struct {
    float kp, ki, kd;
    float integral;
    float prev_error;
    float output;
} pid_t;

typedef struct {
    int16_t x, y;           /* 目标坐标 */
    uint8_t speed_pct;      /* 速度百分比 */
} waypoint_t;

typedef struct {
    /* 位置 */
    int16_t pos_x;
    int16_t pos_y;
    int16_t velocity_x;
    int16_t velocity_y;
    uint16_t heading;
    uint32_t odometer;

    /* 传感器 */
    uint8_t battery;
    uint8_t motor_status;   /* bit0=左OK, bit1=右OK */
    uint8_t obstacle;       /* bit0=前, bit1=左, bit2=右 */

    /* 任务 */
    uint8_t task_id;
    uint8_t status;         /* 0=idle, 1=moving, 2=charging, 3=error */

    /* 路径跟踪 */
    waypoint_t path[32];
    uint8_t path_len;
    uint8_t waypoint_idx;

    /* PID */
    pid_t pid_left;
    pid_t pid_right;

    /* 通信 */
    uint8_t seq_no;
} agv_state_t;

static agv_state_t g_agv;
static rt_device_t g_uart_esp = RT_NULL;

/* ============================================================
 * PID控制器
 * ============================================================ */

static void pid_init(pid_t *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0;
    pid->prev_error = 0;
    pid->output = 0;
}

static float pid_update(pid_t *pid, float error)
{
    pid->integral += error;
    /* 积分限幅 */
    if (pid->integral > 1000) pid->integral = 1000;
    if (pid->integral < -1000) pid->integral = -1000;

    float derivative = error - pid->prev_error;
    pid->output = pid->kp * error +
                  pid->ki * pid->integral +
                  pid->kd * derivative;

    /* 输出限幅 */
    if (pid->output > PID_MAX_OUTPUT) pid->output = PID_MAX_OUTPUT;
    if (pid->output < -PID_MAX_OUTPUT) pid->output = -PID_MAX_OUTPUT;

    pid->prev_error = error;
    return pid->output;
}

/* ============================================================
 * CRC16
 * ============================================================ */

static uint16_t crc16_ccitt(const uint8_t *data, uint16_t len)
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
 * ESP8266 AT指令驱动
 * ============================================================ */

static int esp_send_cmd(const char *cmd, char *resp, size_t resp_len,
                        uint32_t timeout_ms)
{
    /* 清空接收缓冲 */
    while (rt_device_read(g_uart_esp, 0, resp, resp_len) > 0) {}

    /* 发送AT指令 */
    rt_device_write(g_uart_esp, 0, cmd, strlen(cmd));

    /* 等待响应 */
    uint32_t start = rt_tick_get();
    size_t received = 0;

    while ((rt_tick_get() - start) < rt_tick_from_millisecond(timeout_ms)) {
        ssize_t n = rt_device_read(g_uart_esp, 0, resp + received,
                                   resp_len - received - 1);
        if (n > 0) {
            received += n;
            resp[received] = '\0';
            /* 检查是否收到完整响应 */
            if (strstr(resp, "OK") || strstr(resp, "ERROR") ||
                strstr(resp, ">")) {
                return received;
            }
        }
        rt_thread_mdelay(10);
    }

    return received;
}

static int esp_init(void)
{
    char resp[256];

    /* 查找UART设备 */
    g_uart_esp = rt_device_find(ESP_UART_NAME);
    if (!g_uart_esp) {
        rt_kprintf("[AGV] 找不到ESP UART设备\n");
        return -1;
    }

    /* 配置UART */
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
    config.baud_rate = BAUD_RATE_115200;
    rt_device_control(g_uart_esp, RT_DEVICE_CTRL_CONFIG, &config);
    rt_device_open(g_uart_esp, RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX);

    rt_thread_mdelay(1000);

    /* 测试AT */
    esp_send_cmd("AT\r\n", resp, sizeof(resp), 2000);
    if (!strstr(resp, "OK")) {
        rt_kprintf("[AGV] ESP8266 AT测试失败\n");
        return -1;
    }

    /* 设置Station模式 */
    esp_send_cmd("AT+CWMODE=1\r\n", resp, sizeof(resp), 2000);

    /* 连接WiFi */
    rt_kprintf("[AGV] 连接WiFi: %s\n", WIFI_SSID);
    char cmd[128];
    rt_sprintf(cmd, "AT+CWJAP=\"%s\",\"%s\"\r\n", WIFI_SSID, WIFI_PASS);
    esp_send_cmd(cmd, resp, sizeof(resp), 15000);
    if (!strstr(resp, "OK")) {
        rt_kprintf("[AGV] WiFi连接失败\n");
        return -1;
    }

    /* 建立UDP连接 */
    rt_sprintf(cmd, "AT+CIPSTART=\"UDP\",\"%s\",%d,%d,0\r\n",
               SERVER_IP, SERVER_PORT, LOCAL_PORT);
    esp_send_cmd(cmd, resp, sizeof(resp), 5000);

    rt_kprintf("[AGV] ESP8266 初始化完成\n");
    return 0;
}

static int esp_send_udp(const uint8_t *data, uint16_t len)
{
    char cmd[32];
    char resp[128];

    rt_sprintf(cmd, "AT+CIPSEND=%d\r\n", len);
    esp_send_cmd(cmd, resp, sizeof(resp), 1000);

    if (strstr(resp, ">")) {
        rt_device_write(g_uart_esp, 0, data, len);
        return 0;
    }

    return -1;
}

/* ============================================================
 * 协议帧构建与解析
 * ============================================================ */

static uint8_t build_status_frame(uint8_t *buf)
{
    /* Payload: 19字节 */
    uint8_t payload[19];
    payload[0] = AGV_ID;
    payload[1] = g_agv.pos_x & 0xFF;
    payload[2] = (g_agv.pos_x >> 8) & 0xFF;
    payload[3] = g_agv.pos_y & 0xFF;
    payload[4] = (g_agv.pos_y >> 8) & 0xFF;
    payload[5] = g_agv.velocity_x & 0xFF;
    payload[6] = (g_agv.velocity_x >> 8) & 0xFF;
    payload[7] = g_agv.velocity_y & 0xFF;
    payload[8] = (g_agv.velocity_y >> 8) & 0xFF;
    payload[9] = g_agv.heading & 0xFF;
    payload[10] = (g_agv.heading >> 8) & 0xFF;
    payload[11] = g_agv.battery;
    payload[12] = g_agv.motor_status;
    payload[13] = g_agv.obstacle;
    payload[14] = g_agv.task_id;
    payload[15] = (g_agv.odometer) & 0xFF;
    payload[16] = (g_agv.odometer >> 8) & 0xFF;
    payload[17] = (g_agv.odometer >> 16) & 0xFF;
    payload[18] = (g_agv.odometer >> 24) & 0xFF;

    /* 帧头 */
    buf[0] = AGV_MAGIC_0;
    buf[1] = AGV_MAGIC_1;
    buf[2] = g_agv.seq_no++;
    buf[3] = MSG_AGV_STATUS;
    buf[4] = 19;  /* payload_len low */
    buf[5] = 0;   /* payload_len high */

    /* Payload */
    memcpy(buf + 6, payload, 19);

    /* CRC */
    uint16_t crc = crc16_ccitt(buf + 2, 23);  /* SeqNo + MsgType + PayloadLen + Payload */
    buf[25] = crc & 0xFF;
    buf[26] = (crc >> 8) & 0xFF;

    return 27;  /* 总帧长 */
}

static void parse_server_frame(const uint8_t *buf, uint16_t len)
{
    if (len < 8) return;
    if (buf[0] != AGV_MAGIC_0 || buf[1] != AGV_MAGIC_1) return;

    uint8_t msg_type = buf[3];
    uint16_t payload_len = buf[4] | (buf[5] << 8);
    const uint8_t *payload = buf + 6;

    /* CRC校验 */
    uint16_t recv_crc = buf[6 + payload_len] | (buf[7 + payload_len] << 8);
    uint16_t calc_crc = crc16_ccitt(buf + 2, 4 + payload_len);
    if (recv_crc != calc_crc) return;

    switch (msg_type) {
    case MSG_SERVER_CMD:
        if (payload_len >= 8) {
            uint8_t cmd_type = payload[1];
            int16_t tx = payload[2] | (payload[3] << 8);
            int16_t ty = payload[4] | (payload[5] << 8);
            uint8_t speed = payload[6];
            uint8_t path_len = payload[7];

            rt_kprintf("[AGV] 收到指令: type=%d target=(%d,%d) path=%d\n",
                       cmd_type, tx, ty, path_len);

            if (cmd_type == 0) {
                /* STOP */
                g_agv.status = 0;
                g_agv.task_id = 0;
                g_agv.path_len = 0;
            }
            else if (cmd_type == 2 && path_len > 0) {
                /* FOLLOW_PATH */
                g_agv.status = 1;
                g_agv.path_len = path_len > 32 ? 32 : path_len;
                g_agv.waypoint_idx = 0;
                for (uint8_t i = 0; i < g_agv.path_len; i++) {
                    uint16_t offset = 8 + i * 4;
                    if (offset + 4 <= payload_len) {
                        g_agv.path[i].x = payload[offset] |
                                          (payload[offset+1] << 8);
                        g_agv.path[i].y = payload[offset+2] |
                                          (payload[offset+3] << 8);
                        g_agv.path[i].speed_pct = speed;
                    }
                }
            }
        }
        break;

    case MSG_HEARTBEAT_REQ:
        {
            /* 回复心跳 */
            uint8_t resp[9];
            resp[0] = AGV_MAGIC_0;
            resp[1] = AGV_MAGIC_1;
            resp[2] = g_agv.seq_no++;
            resp[3] = MSG_HEARTBEAT_RESP;
            resp[4] = 1;
            resp[5] = 0;
            resp[6] = AGV_ID;
            uint16_t crc = crc16_ccitt(resp + 2, 5);
            resp[7] = crc & 0xFF;
            resp[8] = (crc >> 8) & 0xFF;
            esp_send_udp(resp, 9);
        }
        break;

    case MSG_EMERGENCY_STOP:
        g_agv.status = 0;
        g_agv.task_id = 0;
        g_agv.path_len = 0;
        g_agv.waypoint_idx = 0;
        rt_kprintf("[AGV] 紧急停止!\n");
        break;
    }
}

/* ============================================================
 * 电机控制
 * ============================================================ */

static void motor_set_speed(int16_t left, int16_t right)
{
    /* 左电机 */
    if (left >= 0) {
        rt_pin_write(DIR_LEFT_FWD, PIN_HIGH);
        rt_pin_write(DIR_LEFT_REV, PIN_LOW);
    } else {
        rt_pin_write(DIR_LEFT_FWD, PIN_LOW);
        rt_pin_write(DIR_LEFT_REV, PIN_HIGH);
        left = -left;
    }

    /* 右电机 */
    if (right >= 0) {
        rt_pin_write(DIR_RIGHT_FWD, PIN_HIGH);
        rt_pin_write(DIR_RIGHT_REV, PIN_LOW);
    } else {
        rt_pin_write(DIR_RIGHT_FWD, PIN_LOW);
        rt_pin_write(DIR_RIGHT_REV, PIN_HIGH);
        right = -right;
    }

    /* 设置PWM占空比 */
    struct rt_device_pwm *pwm = (struct rt_device_pwm *)
                                rt_device_find(PWM_DEV_NAME);
    if (pwm) {
        rt_pwm_set(pwm, PWM_CH_LEFT, PWM_PERIOD, left);
        rt_pwm_set(pwm, PWM_CH_RIGHT, PWM_PERIOD, right);
    }
}

static void motor_stop(void)
{
    motor_set_speed(0, 0);
}

/* ============================================================
 * 编码器读取
 * ============================================================ */

static int32_t encoder_read_left(void)
{
    /* TODO: 读取TIM2编码器计数值 */
    return 0;
}

static int32_t encoder_read_right(void)
{
    /* TODO: 读取TIM3编码器计数值 */
    return 0;
}

/* ============================================================
 * 超声波测距
 * ============================================================ */

static uint16_t ultrasonic_distance_cm(void)
{
    /* TODO: Trig脉冲 + Echo高电平时间计算距离 */
    return 999;  /* 默认无障碍 */
}

/* ============================================================
 * 路径跟踪
 * ============================================================ */

static void path_tracker_update(void)
{
    if (g_agv.status != 1 || g_agv.path_len == 0) {
        motor_stop();
        return;
    }

    if (g_agv.waypoint_idx >= g_agv.path_len) {
        /* 路径完成 */
        g_agv.status = 0;
        g_agv.task_id = 0;
        g_agv.path_len = 0;
        motor_stop();
        rt_kprintf("[AGV] 路径完成!\n");
        return;
    }

    /* 当前目标点 */
    waypoint_t *wp = &g_agv.path[g_agv.waypoint_idx];

    /* 计算距离和角度误差 */
    float dx = (float)(wp->x - g_agv.pos_x);
    float dy = (float)(wp->y - g_agv.pos_y);
    float dist = sqrtf(dx * dx + dy * dy);

    /* 到达当前路径点 */
    if (dist < 1.5f) {
        g_agv.waypoint_idx++;
        rt_kprintf("[AGV] 到达路径点 %d/%d\n",
                   g_agv.waypoint_idx, g_agv.path_len);
        return;
    }

    /* 计算目标角度 */
    float target_angle = atan2f(dy, dx) * 180.0f / 3.14159f;
    float angle_error = target_angle - (float)g_agv.heading;

    /* 角度归一化到 -180 ~ 180 */
    while (angle_error > 180) angle_error -= 360;
    while (angle_error < -180) angle_error += 360;

    /* PID控制 */
    float base_speed = (float)wp->speed_pct * 9.0f;  /* 映射到0-900 */
    float turn = pid_update(&g_agv.pid_left, angle_error);

    int16_t left_speed = (int16_t)(base_speed + turn);
    int16_t right_speed = (int16_t)(base_speed - turn);

    /* 限幅 */
    if (left_speed > 900) left_speed = 900;
    if (left_speed < -900) left_speed = -900;
    if (right_speed > 900) right_speed = 900;
    if (right_speed < -900) right_speed = -900;

    motor_set_speed(left_speed, right_speed);
}

/* ============================================================
 * 里程计更新
 * ============================================================ */

static void odometry_update(void)
{
    static int32_t last_left = 0, last_right = 0;

    int32_t left = encoder_read_left();
    int32_t right = encoder_read_right();

    int32_t d_left = left - last_left;
    int32_t d_right = right - last_right;

    last_left = left;
    last_right = right;

    /* 差速模型 (假设轮距150mm, 每脉冲0.5mm) */
    float dl = (float)d_left * 0.5f;
    float dr = (float)d_right * 0.5f;
    float dc = (dl + dr) / 2.0f;
    float dtheta = (dr - dl) / 150.0f;  /* 弧度 */

    g_agv.odometer += (uint32_t)(dc > 0 ? dc : -dc);

    /* 更新位置 (简化: 直接映射到栅格) */
    float rad = (float)g_agv.heading * 3.14159f / 180.0f;
    g_agv.pos_x += (int16_t)(dc * cosf(rad) / 50.0f);  /* 50mm/格 */
    g_agv.pos_y += (int16_t)(dc * sinf(rad) / 50.0f);

    /* 更新朝向 */
    g_agv.heading = (uint16_t)((int16_t)g_agv.heading +
                               (int16_t)(dtheta * 180.0f / 3.14159f)) % 360;

    /* 更新速度 */
    g_agv.velocity_x = (int16_t)(dc * cosf(rad) * 10.0f);  /* mm/s */
    g_agv.velocity_y = (int16_t)(dc * sinf(rad) * 10.0f);
}

/* ============================================================
 * 线程定义
 * ============================================================ */

/* 状态上报线程 */
static void status_thread_entry(void *param)
{
    uint8_t frame[32];

    while (1) {
        uint8_t len = build_status_frame(frame);
        esp_send_udp(frame, len);

        rt_thread_mdelay(STATUS_INTERVAL_MS);
    }
}

/* 接收处理线程 */
static void recv_thread_entry(void *param)
{
    uint8_t buf[512];

    while (1) {
        ssize_t n = rt_device_read(g_uart_esp, 0, buf, sizeof(buf));
        if (n > 0) {
            /* 尝试解析帧 */
            if (n >= 8 && buf[0] == AGV_MAGIC_0 && buf[1] == AGV_MAGIC_1) {
                parse_server_frame(buf, (uint16_t)n);
            }
        }
        rt_thread_mdelay(10);
    }
}

/* 控制循环线程 */
static void control_thread_entry(void *param)
{
    while (1) {
        odometry_update();
        path_tracker_update();

        /* 避障检查 */
        uint16_t dist = ultrasonic_distance_cm();
        g_agv.obstacle = 0;
        if (dist < 20) {
            g_agv.obstacle = 0x01;  /* 前方障碍 */
            if (g_agv.status == 1) {
                motor_stop();
                rt_kprintf("[AGV] 前方障碍! 距离=%dcm\n", dist);
            }
        }

        rt_thread_mdelay(50);  /* 20Hz */
    }
}

/* ============================================================
 * 应用入口
 * ============================================================ */

int main(void)
{
    rt_kprintf("╔══════════════════════════════════════╗\n");
    rt_kprintf("║  云仓灵枢 - AGV#%d 固件              ║\n", AGV_ID);
    rt_kprintf("╚══════════════════════════════════════╝\n");

    /* 初始化 */
    memset(&g_agv, 0, sizeof(g_agv));
    pid_init(&g_agv.pid_left, PID_KP, PID_KI, PID_KD);
    pid_init(&g_agv.pid_right, PID_KP, PID_KI, PID_KD);

    /* GPIO初始化 */
    rt_pin_mode(DIR_LEFT_FWD, PIN_MODE_OUTPUT);
    rt_pin_mode(DIR_LEFT_REV, PIN_MODE_OUTPUT);
    rt_pin_mode(DIR_RIGHT_FWD, PIN_MODE_OUTPUT);
    rt_pin_mode(DIR_RIGHT_REV, PIN_MODE_OUTPUT);
    rt_pin_mode(TRIG_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(ECHO_PIN, PIN_MODE_INPUT);

    /* ESP8266初始化 */
    if (esp_init() != 0) {
        rt_kprintf("[AGV] ESP8266初始化失败!\n");
        return -1;
    }

    rt_kprintf("[AGV] 启动线程...\n");

    /* 创建线程 */
    rt_thread_t tid;

    tid = rt_thread_create("status", status_thread_entry, RT_NULL,
                           1024, 20, 10);
    if (tid) rt_thread_startup(tid);

    tid = rt_thread_create("recv", recv_thread_entry, RT_NULL,
                           2048, 15, 10);
    if (tid) rt_thread_startup(tid);

    tid = rt_thread_create("ctrl", control_thread_entry, RT_NULL,
                           1024, 10, 10);
    if (tid) rt_thread_startup(tid);

    return 0;
}
