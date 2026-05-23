"""
protocol_test.py - 云仓灵枢 协议测试工具

PC端模拟AGV，用于测试K230通信网关(comm_gw)的协议解析能力。
可以在没有实物AGV的情况下验证通信链路。

用法:
    python protocol_test.py                    # 模拟1台AGV发送状态
    python protocol_test.py --agvs 3           # 模拟3台AGV
    python protocol_test.py --server 192.168.4.1  # 指定服务器IP
"""

import socket
import struct
import time
import threading
import argparse
import random

# ============================================================
# 协议常量 (与 agv_protocol.h 一致)
# ============================================================

MAGIC = b'\xAA\x55'

MSG_AGV_STATUS      = 0x01
MSG_SERVER_CMD      = 0x02
MSG_HEARTBEAT_REQ   = 0x03
MSG_HEARTBEAT_RESP  = 0x04
MSG_EMERGENCY_STOP  = 0x05
MSG_TASK_ACK        = 0x06

CMD_STOP        = 0
CMD_MOVE_TO     = 1
CMD_FOLLOW_PATH = 2
CMD_DOCK_CHARGE = 3

AGV_STATUS_IDLE     = 0
AGV_STATUS_MOVING   = 1
AGV_STATUS_CHARGING = 2
AGV_STATUS_ERROR    = 3


# ============================================================
# CRC16-CCITT
# ============================================================

def crc16_ccitt(data: bytes) -> int:
    """CRC16-CCITT校验 (与C实现一致)"""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc <<= 1
            crc &= 0xFFFF
    return crc


# ============================================================
# 帧构建
# ============================================================

def build_frame(seq_no: int, msg_type: int, payload: bytes) -> bytes:
    """构建完整协议帧"""
    header = MAGIC + struct.pack('<BBH', seq_no, msg_type, len(payload))
    body = header + payload
    crc = crc16_ccitt(body[2:])  # CRC从SeqNo开始计算
    return body + struct.pack('<H', crc)


def build_status_payload(agv_id: int, x: int, y: int,
                         vx: int = 0, vy: int = 0,
                         heading: int = 0, battery: int = 100,
                         motor: int = 0x03, obstacle: int = 0,
                         task_id: int = 0, odometer: int = 0) -> bytes:
    """构建AGV状态上报Payload (19字节)"""
    return struct.pack('<BhhhhBBBBI',
                       agv_id, x, y, vx, vy,
                       heading, battery, motor, obstacle,
                       task_id, odometer)


def build_heartbeat_resp(agv_id: int) -> bytes:
    """构建心跳响应Payload"""
    return struct.pack('<B', agv_id)


def parse_frame(data: bytes) -> dict:
    """解析接收到的帧"""
    if len(data) < 8:
        return None

    if data[0] != 0xAA or data[1] != 0x55:
        return None

    seq_no, msg_type, payload_len = struct.unpack('<BBH', data[2:6])

    if len(data) < 8 + payload_len:
        return None

    payload = data[6:6+payload_len]
    recv_crc = struct.unpack('<H', data[6+payload_len:8+payload_len])[0]
    calc_crc = crc16_ccitt(data[2:6+payload_len])

    return {
        'seq_no': seq_no,
        'msg_type': msg_type,
        'payload_len': payload_len,
        'payload': payload,
        'crc_valid': recv_crc == calc_crc,
        'recv_crc': recv_crc,
        'calc_crc': calc_crc,
    }


def parse_server_cmd(payload: bytes) -> dict:
    """解析服务器指令Payload"""
    if len(payload) < 6:
        return None
    agv_id, cmd_type, tx, ty, speed, path_len = struct.unpack(
        '<BBhhBB', payload[:8])
    result = {
        'agv_id': agv_id,
        'cmd_type': cmd_type,
        'target_x': tx,
        'target_y': ty,
        'speed_pct': speed,
        'path_len': path_len,
    }
    # 解析路径点
    if path_len > 0 and len(payload) >= 8 + path_len * 4:
        points = []
        for i in range(path_len):
            offset = 8 + i * 4
            px, py = struct.unpack('<hh', payload[offset:offset+4])
            points.append((px, py))
        result['path'] = points
    return result


# ============================================================
# AGV模拟器
# ============================================================

class AGVSimulator:
    """模拟一台AGV的行为"""

    def __init__(self, agv_id: int, server_ip: str,
                 start_x: int = 0, start_y: int = 0):
        self.agv_id = agv_id
        self.server_ip = server_ip
        self.x = start_x
        self.y = start_y
        self.heading = 0
        self.battery = 100
        self.status = AGV_STATUS_IDLE
        self.task_id = 0
        self.odometer = 0
        self.seq_no = 0

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(1.0)
        self.running = False

        # 统计
        self.tx_count = 0
        self.rx_count = 0
        self.crc_errors = 0

    def next_seq(self) -> int:
        seq = self.seq_no
        self.seq_no = (self.seq_no + 1) & 0xFF
        return seq

    def send_status(self):
        """发送状态上报"""
        payload = build_status_payload(
            self.agv_id, self.x, self.y,
            heading=self.heading, battery=self.battery,
            task_id=self.task_id, odometer=self.odometer
        )
        frame = build_frame(self.next_seq(), MSG_AGV_STATUS, payload)
        self.sock.sendto(frame, (self.server_ip, 5000))
        self.tx_count += 1

    def send_heartbeat_resp(self):
        """发送心跳响应"""
        payload = build_heartbeat_resp(self.agv_id)
        frame = build_frame(self.next_seq(), MSG_HEARTBEAT_RESP, payload)
        self.sock.sendto(frame, (self.server_ip, 5000))
        self.tx_count += 1

    def handle_command(self, cmd: dict):
        """处理服务器指令"""
        cmd_type = cmd.get('cmd_type', CMD_STOP)
        if cmd_type == CMD_STOP:
            self.status = AGV_STATUS_IDLE
            print(f"  [AGV#{self.agv_id}] 收到STOP指令")
        elif cmd_type == CMD_MOVE_TO:
            self.status = AGV_STATUS_MOVING
            self.task_id = 1
            print(f"  [AGV#{self.agv_id}] 收到MOVE_TO指令: "
                  f"({cmd['target_x']},{cmd['target_y']})")
        elif cmd_type == CMD_FOLLOW_PATH:
            self.status = AGV_STATUS_MOVING
            self.task_id = 1
            path = cmd.get('path', [])
            print(f"  [AGV#{self.agv_id}] 收到FOLLOW_PATH指令: "
                  f"{len(path)}个路径点")
        elif cmd_type == CMD_DOCK_CHARGE:
            self.status = AGV_STATUS_CHARGING
            print(f"  [AGV#{self.agv_id}] 收到DOCK_CHARGE指令")

    def simulate_movement(self):
        """模拟AGV随机移动"""
        if self.status == AGV_STATUS_MOVING:
            dx = random.choice([-1, 0, 1])
            dy = random.choice([-1, 0, 1])
            self.x = max(0, min(31, self.x + dx))
            self.y = max(0, min(31, self.y + dy))
            self.odometer += abs(dx) + abs(dy)

            # 随机更新朝向
            if dx > 0: self.heading = 0
            elif dx < 0: self.heading = 180
            if dy > 0: self.heading = 90
            elif dy < 0: self.heading = 270

    def recv_loop(self):
        """接收线程"""
        while self.running:
            try:
                data, addr = self.sock.recvfrom(1024)
                frame = parse_frame(data)
                if frame is None:
                    continue

                self.rx_count += 1

                if not frame['crc_valid']:
                    self.crc_errors += 1
                    print(f"  [AGV#{self.agv_id}] CRC错误! "
                          f"recv={frame['recv_crc']:04X} "
                          f"calc={frame['calc_crc']:04X}")
                    continue

                msg_type = frame['msg_type']
                if msg_type == MSG_HEARTBEAT_REQ:
                    self.send_heartbeat_resp()
                elif msg_type == MSG_SERVER_CMD:
                    cmd = parse_server_cmd(frame['payload'])
                    if cmd and cmd['agv_id'] == self.agv_id:
                        self.handle_command(cmd)
                elif msg_type == MSG_EMERGENCY_STOP:
                    self.status = AGV_STATUS_IDLE
                    print(f"  [AGV#{self.agv_id}] 收到EMERGENCY_STOP!")

            except socket.timeout:
                continue
            except Exception as e:
                if self.running:
                    print(f"  [AGV#{self.agv_id}] 接收错误: {e}")

    def run(self, duration: float = 30.0):
        """运行AGV模拟器"""
        self.running = True

        # 启动接收线程
        recv_thread = threading.Thread(target=self.recv_loop, daemon=True)
        recv_thread.start()

        print(f"  [AGV#{self.agv_id}] 启动, 位置: ({self.x},{self.y})")

        end_time = time.time() + duration
        while time.time() < end_time and self.running:
            self.simulate_movement()
            self.send_status()
            time.sleep(0.1)  # 10Hz

        self.running = False
        print(f"  [AGV#{self.agv_id}] 停止. TX={self.tx_count} "
              f"RX={self.rx_count} CRC_err={self.crc_errors}")


# ============================================================
# 测试场景
# ============================================================

def test_single_agv(server_ip: str, duration: float = 10.0):
    """测试单台AGV通信"""
    print("=" * 50)
    print("测试: 单台AGV通信")
    print("=" * 50)

    agv = AGVSimulator(1, server_ip, start_x=0, start_y=0)
    agv.run(duration)

    print(f"\n结果: TX={agv.tx_count}, RX={agv.rx_count}")
    return agv.tx_count > 0


def test_multi_agv(server_ip: str, num_agvs: int = 3, duration: float = 10.0):
    """测试多台AGV并发通信"""
    print("=" * 50)
    print(f"测试: {num_agvs}台AGV并发通信")
    print("=" * 50)

    agvs = []
    threads = []

    for i in range(num_agvs):
        agv = AGVSimulator(i + 1, server_ip,
                           start_x=random.randint(0, 31),
                           start_y=random.randint(0, 31))
        agvs.append(agv)
        t = threading.Thread(target=agv.run, args=(duration,), daemon=True)
        threads.append(t)
        t.start()

    for t in threads:
        t.join()

    print(f"\n汇总:")
    total_tx = sum(a.tx_count for a in agvs)
    total_rx = sum(a.rx_count for a in agvs)
    total_err = sum(a.crc_errors for a in agvs)
    print(f"  总TX: {total_tx}, 总RX: {total_rx}, CRC错误: {total_err}")
    return total_tx > 0


# ============================================================
# 入口
# ============================================================

def main():
    parser = argparse.ArgumentParser(description='云仓灵枢 协议测试工具')
    parser.add_argument('--server', default='127.0.0.1',
                        help='服务器IP (默认127.0.0.1)')
    parser.add_argument('--agvs', type=int, default=1,
                        help='模拟AGV数量 (默认1)')
    parser.add_argument('--duration', type=float, default=10.0,
                        help='测试时长秒 (默认10)')
    parser.add_argument('--test', action='store_true',
                        help='运行所有测试')
    args = parser.parse_args()

    print("╔══════════════════════════════════════════════╗")
    print("║    云仓灵枢 - 协议测试工具 (PC端AGV模拟)    ║")
    print("╚══════════════════════════════════════════════╝")
    print(f"服务器: {args.server}")
    print()

    if args.test:
        test_single_agv(args.server, 5.0)
        print()
        test_multi_agv(args.server, 3, 5.0)
    elif args.agvs == 1:
        test_single_agv(args.server, args.duration)
    else:
        test_multi_agv(args.server, args.agvs, args.duration)


if __name__ == '__main__':
    main()
