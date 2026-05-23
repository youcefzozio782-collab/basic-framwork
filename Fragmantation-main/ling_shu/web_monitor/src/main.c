/**
 * web_monitor - 云仓灵枢 Web监控进程
 *
 * 运行在K230 (RT-Thread Smart) 上，负责：
 * 1. Mongoose HTTP服务器：提供Web页面和REST API
 * 2. WebSocket服务器：每200ms推送AGV实时状态
 * 3. 操作员接口：接收任务提交和急停指令
 * 4. 通过管道与scheduler通信
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

/* Mongoose头文件 (需要单独获取mongoose.c/mongoose.h) */
/* #include "mongoose.h" */

/* ============================================================
 * 注意：本文件使用伪代码标注Mongoose集成点
 * 实际使用需要：
 * 1. 从 https://github.com/cesanta/mongoose 下载 mongoose.c + mongoose.h
 * 2. 将mongoose.c加入编译
 * 3. 取消注释上面的 #include "mongoose.h"
 * ============================================================ */

/* ============================================================
 * 全局变量
 * ============================================================ */

static shm_agv_t *g_shm = NULL;
static volatile int g_running = 1;
static int g_task_pipe_fd = -1;         /* 写端: web -> scheduler */

/* ============================================================
 * 共享内存连接
 * ============================================================ */

static int shm_connect(void)
{
    int fd = shm_open(SHM_NAME, O_RDWR, 0);
    if (fd < 0) {
        printf("[web] 无法连接共享内存\n");
        return -1;
    }

    g_shm = (shm_agv_t *)mmap(NULL, sizeof(shm_agv_t),
                               PROT_READ | PROT_WRITE,
                               MAP_SHARED, fd, 0);
    if (g_shm == MAP_FAILED) {
        close(fd);
        return -1;
    }

    close(fd);
    return 0;
}

/* ============================================================
 * JSON序列化
 * ============================================================ */

static int format_status_json(char *buf, size_t buf_size)
{
    pthread_mutex_lock(&g_shm->lock);

    int n = snprintf(buf, buf_size,
        "{\"tick\":%u,\"emergency\":%d,\"num_agvs\":%d,\"agvs\":[",
        g_shm->state.tick,
        g_shm->state.emergency_stop,
        g_shm->state.num_agvs);

    for (int i = 0; i < MAX_AGVS; i++) {
        agv_state_t *a = &g_shm->state.agvs[i];
        if (a->agv_id == 0) continue;

        n += snprintf(buf + n, buf_size - n,
            "%s{\"id\":%d,\"x\":%d,\"y\":%d,\"heading\":%d,"
            "\"speed\":%d,\"battery\":%d,\"status\":%d,"
            "\"task_id\":%d,\"online\":%d}",
            (i > 0 && n > 20) ? "," : "",
            a->agv_id, a->pos_x, a->pos_y, a->heading,
            a->speed, a->battery, a->status,
            a->task_id, a->online);
    }

    n += snprintf(buf + n, buf_size - n, "],\"tasks\":[");

    int first_task = 1;
    for (int i = 0; i < MAX_TASKS; i++) {
        task_t *t = &g_shm->state.task_queue[i];
        if (t->task_id == 0) continue;

        n += snprintf(buf + n, buf_size - n,
            "%s{\"id\":%d,\"agv\":%d,\"goal\":[%d,%d],\"state\":%d}",
            first_task ? "" : ",",
            t->task_id, t->agv_id, t->goal_x, t->goal_y, t->state);
        first_task = 0;
    }

    n += snprintf(buf + n, buf_size - n, "],\"grid\":[");

    /* 只发送非空格子 (压缩JSON大小) */
    for (int y = 0; y < GRID_H; y++) {
        n += snprintf(buf + n, buf_size - n, "%s[", y > 0 ? "," : "");
        for (int x = 0; x < GRID_W; x++) {
            n += snprintf(buf + n, buf_size - n, "%s%d",
                         x > 0 ? "," : "",
                         g_shm->state.grid_map[y][x]);
        }
        n += snprintf(buf + n, buf_size - n, "]");
    }

    n += snprintf(buf + n, buf_size - n, "]}");

    pthread_mutex_unlock(&g_shm->lock);
    return n;
}

/* ============================================================
 * 内嵌HTML前端
 * ============================================================ */

static const char index_html[] =
"<!DOCTYPE html>"
"<html>"
"<head>"
"<meta charset='utf-8'>"
"<title>云仓灵枢 - AGV调度监控</title>"
"<style>"
"body{margin:0;background:#1a1a2e;color:#e0e0e0;font-family:monospace;}"
"#map{border:1px solid #333;}"
".panel{display:flex;gap:20px;padding:10px;}"
".card{background:#16213e;padding:10px;border-radius:8px;min-width:120px;}"
".dot{width:10px;height:10px;border-radius:50%;display:inline-block;}"
".on{background:#00ff88;}.off{background:#ff4444;}"
"button{padding:8px 16px;margin:4px;cursor:pointer;}"
"</style>"
"</head>"
"<body>"
"<h2>云仓灵枢 - AGV调度监控</h2>"
"<div class='panel'>"
"<div id='cards'></div>"
"<canvas id='map' width='640' height='640'></canvas>"
"<div>"
"<button onclick='estop()'>紧急停止</button>"
"<button onclick='addTask()'>添加任务</button>"
"<div id='log' style='max-height:400px;overflow-y:auto;font-size:12px;'></div>"
"</div>"
"</div>"
"<script>"
"const G=32,C=20,cv=document.getElementById('map'),cx=cv.getContext('2d');"
"let ws;"
"function connect(){"
"ws=new WebSocket('ws://'+location.host+'/ws');"
"ws.onmessage=e=>{const d=JSON.parse(e.data);draw(d);cards(d);};"
"ws.onclose=()=>setTimeout(connect,2000);"
"}"
"function draw(d){"
"if(!d.grid)return;"
"for(let y=0;y<G;y++)for(let x=0;x<G;x++){"
"cx.fillStyle=d.grid[y][x]===1?'#333':d.grid[y][x]===2?'#0066ff':d.grid[y][x]===3?'#664400':'#222';"
"cx.fillRect(x*C,y*C,C-1,C-1);}"
"const cl=['#ff6b6b','#4ecdc4','#45b7d1','#96ceb4','#ffeaa7'];"
"d.agvs.forEach(a=>{"
"cx.fillStyle=cl[(a.id-1)%5];"
"cx.beginPath();"
"cx.arc(a.x*C+C/2,a.y*C+C/2,C/2-2,0,Math.PI*2);"
"cx.fill();"
"cx.fillStyle='#fff';cx.font='12px monospace';"
"cx.fillText('A'+a.id,a.x*C+3,a.y*C+C/2+4);"
"});"
"}"
"function cards(d){"
"let h='';"
"d.agvs.forEach(a=>{"
"h+=`<div class='card'><span class='dot ${a.online?'on':'off'}'></span> `;"
"h+=`AGV#${a.id} [${a.x},${a.y}]<br>`;"
"h+=`电量:${a.battery}% 速度:${a.speed}<br>`;"
"h+=`状态:${['空闲','移动','充电','故障'][a.status]}`;
"h+=`</div>`;});"
"document.getElementById('cards').innerHTML=h;"
"}"
"function estop(){fetch('/api/emergency',{method:'POST'});}"
"function addTask(){"
"const gx=prompt('目标X坐标(0-31):');"
"const gy=prompt('目标Y坐标(0-31):');"
"if(gx!==null&&gy!==null){"
"fetch('/api/task',{method:'POST',"
"headers:{'Content-Type':'application/json'},"
"body:JSON.stringify({goal_x:+gx,goal_y:+gy})});}}"
"connect();"
"</script>"
"</body>"
"</html>";

/* ============================================================
 * Mongoose HTTP/WebSocket处理 (伪代码框架)
 * ============================================================ */

/*
 * 以下是Mongoose集成的伪代码框架。
 * 实际实现需要：
 * 1. 下载mongoose.c + mongoose.h
 * 2. 在SConscript中加入mongoose.c
 * 3. 实现以下处理函数
 */

/*
static void mongoose_handler(struct mg_connection *c, int ev, void *ev_data) {
    if (ev == MG_EV_HTTP_MSG) {
        struct mg_http_message *hm = (struct mg_http_message *)ev_data;

        if (mg_match(hm->uri, mg_str("/"), NULL)) {
            mg_http_reply(c, 200, "Content-Type: text/html\r\n",
                         "%s", index_html);
        }
        else if (mg_match(hm->uri, mg_str("/api/status"), NULL)) {
            char json[4096];
            format_status_json(json, sizeof(json));
            mg_http_reply(c, 200,
                         "Content-Type: application/json\r\n",
                         "%s", json);
        }
        else if (mg_match(hm->uri, mg_str("/api/task"), NULL)) {
            // 解析JSON, 通过管道发送给scheduler
            mg_http_reply(c, 200, NULL, "{\"ok\":true}");
        }
        else if (mg_match(hm->uri, mg_str("/api/emergency"), NULL)) {
            // 设置急停标志
            pthread_mutex_lock(&g_shm->lock);
            g_shm->state.emergency_stop = 1;
            g_shm->version++;
            pthread_mutex_unlock(&g_shm->lock);
            mg_http_reply(c, 200, NULL, "{\"ok\":true}");
        }
        else {
            mg_http_reply(c, 404, NULL, "Not Found");
        }
    }
    else if (ev == MG_EV_WS_OPEN) {
        // 新WebSocket连接
    }
    else if (ev == MG_EV_WS_MSG) {
        // 处理WebSocket消息
    }
}

static void ws_broadcast_thread(void *arg) {
    // 每200ms广播状态给所有WebSocket客户端
    while (g_running) {
        char json[4096];
        format_status_json(json, sizeof(json));
        // mg_ws_broadcast(mgr, json, strlen(json));
        usleep(WS_PUSH_INTERVAL_MS * 1000);
    }
}
*/

/* ============================================================
 * 简化HTTP服务器 (不依赖Mongoose, 使用原始socket)
 * ============================================================ */

static const char *http_response_200 =
    "HTTP/1.1 200 OK\r\n"
    "Content-Type: text/html\r\n"
    "Connection: close\r\n\r\n";

static const char *http_response_json =
    "HTTP/1.1 200 OK\r\n"
    "Content-Type: application/json\r\n"
    "Access-Control-Allow-Origin: *\r\n"
    "Connection: close\r\n\r\n";

static const char *http_response_404 =
    "HTTP/1.1 404 Not Found\r\n"
    "Connection: close\r\n\r\nNot Found";

static void handle_http_client(int client_fd)
{
    char buf[2048];
    ssize_t n = recv(client_fd, buf, sizeof(buf) - 1, 0);
    if (n <= 0) { close(client_fd); return; }
    buf[n] = '\0';

    /* 解析请求行 */
    char method[8], path[256];
    sscanf(buf, "%s %s", method, path);

    if (strcmp(path, "/") == 0) {
        /* 返回主页 */
        send(client_fd, http_response_200, strlen(http_response_200), 0);
        send(client_fd, index_html, strlen(index_html), 0);
    }
    else if (strcmp(path, "/api/status") == 0) {
        /* 返回JSON状态 */
        char json[8192];
        format_status_json(json, sizeof(json));
        send(client_fd, http_response_json, strlen(http_response_json), 0);
        send(client_fd, json, strlen(json), 0);
    }
    else if (strcmp(path, "/api/emergency") == 0 &&
             strncmp(method, "POST", 4) == 0) {
        /* 急停 */
        pthread_mutex_lock(&g_shm->lock);
        g_shm->state.emergency_stop = 1;
        g_shm->version++;
        pthread_mutex_unlock(&g_shm->lock);
        printf("[web] 急停触发!\n");
        const char *resp = "{\"ok\":true}";
        send(client_fd, http_response_json, strlen(http_response_json), 0);
        send(client_fd, resp, strlen(resp), 0);
    }
    else if (strcmp(path, "/api/task") == 0 &&
             strncmp(method, "POST", 4) == 0) {
        /* 提取JSON body */
        char *body = strstr(buf, "\r\n\r\n");
        if (body) {
            body += 4;
            /* 简单解析: {"goal_x":X,"goal_y":Y} */
            int gx = -1, gy = -1;
            sscanf(body, "{\"goal_x\":%d,\"goal_y\":%d}", &gx, &gy);
            if (gx >= 0 && gx < GRID_W && gy >= 0 && gy < GRID_H) {
                /* 通过管道发送给scheduler */
                task_t task;
                memset(&task, 0, sizeof(task));
                task.goal_x = gx;
                task.goal_y = gy;
                task.priority = 128;
                if (g_task_pipe_fd >= 0) {
                    write(g_task_pipe_fd, &task, sizeof(task));
                }
                printf("[web] 新任务请求: (%d,%d)\n", gx, gy);
            }
        }
        const char *resp = "{\"ok\":true}";
        send(client_fd, http_response_json, strlen(http_response_json), 0);
        send(client_fd, resp, strlen(resp), 0);
    }
    else {
        send(client_fd, http_response_404, strlen(http_response_404), 0);
    }

    close(client_fd);
}

/* HTTP服务器线程 */
static void *http_server_thread(void *arg)
{
    (void)arg;

    int server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd < 0) {
        printf("[web] 创建socket失败\n");
        return NULL;
    }

    int opt = 1;
    setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(PORT_WEB_HTTP);

    if (bind(server_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        printf("[web] bind失败\n");
        close(server_fd);
        return NULL;
    }

    listen(server_fd, 5);
    printf("[web] HTTP服务器启动, 端口 %d\n", PORT_WEB_HTTP);

    while (g_running) {
        struct sockaddr_in client_addr;
        socklen_t client_len = sizeof(client_addr);
        int client_fd = accept(server_fd,
                               (struct sockaddr *)&client_addr,
                               &client_len);
        if (client_fd >= 0) {
            handle_http_client(client_fd);
        }
    }

    close(server_fd);
    printf("[web] HTTP服务器退出\n");
    return NULL;
}

/* 状态刷新线程 (用于WebSocket推送, 简化版用轮询) */
static void *status_refresh_thread(void *arg)
{
    (void)arg;

    while (g_running) {
        usleep(WS_PUSH_INTERVAL_MS * 1000);

        /* 更新全局状态快照 (供WebSocket使用) */
        /* 在完整版中, 这里会广播给所有WebSocket客户端 */
    }

    return NULL;
}

/* ============================================================
 * 主函数
 * ============================================================ */

int main(int argc, char *argv[])
{
    printf("╔══════════════════════════════════════════════╗\n");
    printf("║    云仓灵枢 - Web监控 (web_monitor)          ║\n");
    printf("╚══════════════════════════════════════════════╝\n");

    /* 连接共享内存 */
    if (shm_connect() != 0) {
        printf("[web] 无法连接共享内存\n");
        return -1;
    }

    /* 打开管道 (读端, scheduler写入) */
    int pipe_fds[2];
    if (pipe(pipe_fds) == 0) {
        g_task_pipe_fd = pipe_fds[1];  /* 写端 */
        /* pipe_fds[0] 是读端, 由scheduler读取 */
    }

    printf("[web] 初始化完成\n");

    /* 启动线程 */
    pthread_t tid_http, tid_refresh;
    pthread_create(&tid_http, NULL, http_server_thread, NULL);
    pthread_create(&tid_refresh, NULL, status_refresh_thread, NULL);

    /* 主线程 */
    while (g_running) {
        sleep(1);
    }

    pthread_join(tid_http, NULL);
    pthread_join(tid_refresh, NULL);

    munmap(g_shm, sizeof(shm_agv_t));
    if (g_task_pipe_fd >= 0) close(g_task_pipe_fd);

    printf("[web] 已退出\n");
    return 0;
}
