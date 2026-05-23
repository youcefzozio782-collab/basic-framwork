"""
chaos_test.py - 云仓灵枢 混沌工程测试脚本

PC端运行，对K230边缘服务器进行各种极端条件测试。
验证系统在网络风暴、畸形包、指令洪泛等场景下的鲁棒性。

用法:
    python chaos_test.py                      # 运行所有测试
    python chaos_test.py --test storm         # 只运行网络风暴测试
    python chaos_test.py --server 192.168.4.1 # 指定服务器IP
"""

import socket
import struct
import time
import threading
import random
import argparse
import urllib.request
import json

# ============================================================
# 协议常量
# ============================================================

MAGIC = b'\xAA\x55'
MSG_AGV_STATUS = 0x01


def crc16_ccitt(data: bytes) -> int:
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
# 混沌测试类
# ============================================================

class ChaosTest:
    def __init__(self, server_ip: str, udp_port: int = 5000,
                 http_port: int = 8080):
        self.server_ip = server_ip
        self.udp_port = udp_port
        self.http_port = http_port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.results = {}

    def log(self, category: str, message: str):
        ts = time.strftime("%H:%M:%S")
        print(f"  [{ts}] [{category}] {message}")

    # ----------------------------------------------------------
    # 测试1: 网络风暴
    # ----------------------------------------------------------
    def test_network_storm(self, duration_s: float = 10.0,
                           pps: int = 1000):
        """以高速率发送随机UDP包"""
        self.log("STORM", f"发送 {pps} pkt/s, 持续 {duration_s}s...")
        count = 0
        start = time.time()
        end_time = start + duration_s

        while time.time() < end_time:
            size = random.randint(10, 512)
            data = bytes(random.getrandbits(8) for _ in range(size))
            try:
                self.sock.sendto(data, (self.server_ip, self.udp_port))
                count += 1
            except Exception as e:
                self.log("STORM", f"发送错误: {e}")
            time.sleep(1.0 / pps)

        elapsed = time.time() - start
        actual_pps = count / elapsed
        self.log("STORM", f"完成: 发送 {count} 包, "
                 f"实际速率 {actual_pps:.0f} pkt/s")
        self.results['network_storm'] = {
            'packets': count,
            'pps': actual_pps,
            'duration': elapsed,
        }

    # ----------------------------------------------------------
    # 测试2: 畸形包洪泛
    # ----------------------------------------------------------
    def test_malformed_packets(self, duration_s: float = 10.0):
        """发送有正确Magic但格式错误的包"""
        self.log("FUZZ", f"发送畸形包, 持续 {duration_s}s...")
        count = 0
        start = time.time()
        end_time = start + duration_s

        while time.time() < end_time:
            # 各种畸形包
            variant = random.randint(0, 4)
            if variant == 0:
                # 正确Magic + 随机垃圾
                data = MAGIC + bytes(random.getrandbits(8)
                                     for _ in range(random.randint(1, 100)))
            elif variant == 1:
                # 正确Magic + 超大PayloadLen
                data = MAGIC + struct.pack('<BBH', 0, 0x01, 65535)
                data += bytes(100)
            elif variant == 2:
                # 正确Magic + 零长度
                data = MAGIC + struct.pack('<BBH', 0, 0x01, 0)
            elif variant == 3:
                # 正确帧格式但CRC错误
                payload = bytes(10)
                data = MAGIC + struct.pack('<BBH', 0, 0x01, len(payload))
                data += payload + struct.pack('<H', 0xFFFF)  # 错误CRC
            else:
                # 只有Magic
                data = MAGIC

            try:
                self.sock.sendto(data, (self.server_ip, self.udp_port))
                count += 1
            except Exception:
                pass
            time.sleep(0.01)  # 100 pkt/s

        elapsed = time.time() - start
        self.log("FUZZ", f"完成: 发送 {count} 畸形包")
        self.results['malformed'] = {'packets': count, 'duration': elapsed}

    # ----------------------------------------------------------
    # 测试3: AGV身份伪造
    # ----------------------------------------------------------
    def test_agv_impersonation(self, fake_id: int = 99,
                               duration_s: float = 10.0):
        """用非法AGV ID发送状态报告"""
        self.log("SPOOF", f"伪造 AGV#{fake_id}, 持续 {duration_s}s...")
        count = 0
        seq = 0
        start = time.time()
        end_time = start + duration_s

        while time.time() < end_time:
            payload = struct.pack('<BhhhhBBBBI',
                                  fake_id,
                                  random.randint(0, 31),
                                  random.randint(0, 31),
                                  random.randint(-100, 100),
                                  random.randint(-100, 100),
                                  random.randint(0, 359),
                                  random.randint(20, 100),
                                  0x03, 0x00, 0, 0)

            header = MAGIC + struct.pack('<BBH', seq, MSG_AGV_STATUS,
                                         len(payload))
            body = header + payload
            crc = crc16_ccitt(body[2:])
            frame = body + struct.pack('<H', crc)

            try:
                self.sock.sendto(frame, (self.server_ip, self.udp_port))
                count += 1
            except Exception:
                pass

            seq = (seq + 1) & 0xFF
            time.sleep(0.05)  # 20Hz

        elapsed = time.time() - start
        self.log("SPOOF", f"完成: 发送 {count} 伪造包")
        self.results['impersonation'] = {'packets': count, 'duration': elapsed}

    # ----------------------------------------------------------
    # 测试4: HTTP指令洪泛
    # ----------------------------------------------------------
    def test_http_flood(self, count: int = 100):
        """通过Web API快速提交大量任务"""
        self.log("FLOOD", f"提交 {count} 个HTTP任务请求...")
        success = 0
        errors = 0
        start = time.time()

        for i in range(count):
            try:
                data = json.dumps({
                    "agv_id": (i % 3) + 1,
                    "goal_x": random.randint(0, 31),
                    "goal_y": random.randint(0, 31)
                }).encode()
                req = urllib.request.Request(
                    f"http://{self.server_ip}:{self.http_port}/api/task",
                    data=data,
                    headers={'Content-Type': 'application/json'}
                )
                resp = urllib.request.urlopen(req, timeout=1)
                if resp.status == 200:
                    success += 1
                else:
                    errors += 1
            except urllib.error.HTTPError as e:
                if e.code == 429:  # 限流
                    success += 1  # 限流是预期行为
                else:
                    errors += 1
            except Exception:
                errors += 1

        elapsed = time.time() - start
        self.log("FLOOD", f"完成: 成功={success}, 错误={errors}, "
                 f"耗时={elapsed:.2f}s")
        self.results['http_flood'] = {
            'success': success, 'errors': errors, 'duration': elapsed
        }

    # ----------------------------------------------------------
    # 测试5: 急停压力测试
    # ----------------------------------------------------------
    def test_emergency_stop_stress(self, cycles: int = 10):
        """连续发送急停/恢复交替"""
        self.log("ESTOP", f"急停压力测试: {cycles} 次循环...")
        count = 0
        start = time.time()

        for i in range(cycles):
            # 发送急停 (通过UDP)
            payload = b'\x01'  # emergency_stop = 1
            header = MAGIC + struct.pack('<BBH', i & 0xFF, 0x05,
                                         len(payload))
            body = header + payload
            crc = crc16_ccitt(body[2:])
            frame = body + struct.pack('<H', crc)

            try:
                # 发送3次确保到达
                for _ in range(3):
                    self.sock.sendto(frame,
                                     (self.server_ip, self.udp_port))
                count += 3
            except Exception:
                pass

            time.sleep(0.1)

            # 发送恢复 (通过HTTP)
            try:
                req = urllib.request.Request(
                    f"http://{self.server_ip}:{self.http_port}/api/emergency",
                    data=json.dumps({"stop": False}).encode(),
                    headers={'Content-Type': 'application/json'},
                    method='POST'
                )
                urllib.request.urlopen(req, timeout=1)
            except Exception:
                pass

            time.sleep(0.1)

        elapsed = time.time() - start
        self.log("ESTOP", f"完成: 发送 {count} 急停包, "
                 f"耗时={elapsed:.2f}s")
        self.results['emergency'] = {'packets': count, 'duration': elapsed}

    # ----------------------------------------------------------
    # 测试6: 混合流量
    # ----------------------------------------------------------
    def test_mixed_traffic(self, duration_s: float = 15.0):
        """同时发送正常AGV状态+网络风暴+畸形包"""
        self.log("MIXED", f"混合流量测试, 持续 {duration_s}s...")
        start = time.time()
        counters = {'normal': 0, 'storm': 0, 'malformed': 0}

        def normal_traffic():
            """正常AGV状态上报 (10Hz)"""
            seq = 0
            end = time.time() + duration_s
            while time.time() < end:
                payload = struct.pack('<BhhhhBBBBI',
                                      1, 0, 0, 0, 0, 0, 100, 3, 0, 0, 0)
                header = MAGIC + struct.pack('<BBH', seq,
                                             MSG_AGV_STATUS, len(payload))
                body = header + payload
                crc = crc16_ccitt(body[2:])
                frame = body + struct.pack('<H', crc)
                try:
                    self.sock.sendto(frame,
                                     (self.server_ip, self.udp_port))
                    counters['normal'] += 1
                except Exception:
                    pass
                seq = (seq + 1) & 0xFF
                time.sleep(0.1)

        def storm_traffic():
            """风暴流量 (200 pkt/s)"""
            end = time.time() + duration_s
            while time.time() < end:
                data = bytes(random.getrandbits(8)
                             for _ in range(random.randint(10, 200)))
                try:
                    self.sock.sendto(data,
                                     (self.server_ip, self.udp_port))
                    counters['storm'] += 1
                except Exception:
                    pass
                time.sleep(0.005)

        def malformed_traffic():
            """畸形包 (50 pkt/s)"""
            end = time.time() + duration_s
            while time.time() < end:
                data = MAGIC + bytes(random.getrandbits(8)
                                     for _ in range(random.randint(1, 50)))
                try:
                    self.sock.sendto(data,
                                     (self.server_ip, self.udp_port))
                    counters['malformed'] += 1
                except Exception:
                    pass
                time.sleep(0.02)

        # 并行运行三种流量
        threads = [
            threading.Thread(target=normal_traffic, daemon=True),
            threading.Thread(target=storm_traffic, daemon=True),
            threading.Thread(target=malformed_traffic, daemon=True),
        ]
        for t in threads:
            t.start()
        for t in threads:
            t.join()

        elapsed = time.time() - start
        self.log("MIXED", f"完成: 正常={counters['normal']}, "
                 f"风暴={counters['storm']}, 畸形={counters['malformed']}")
        self.results['mixed'] = {**counters, 'duration': elapsed}

    # ----------------------------------------------------------
    # 运行所有测试
    # ----------------------------------------------------------
    def run_all(self):
        """依次运行所有混沌测试"""
        print("╔══════════════════════════════════════════════╗")
        print("║    云仓灵枢 - 混沌工程测试套件              ║")
        print("╚══════════════════════════════════════════════╝")
        print(f"目标服务器: {self.server_ip}:{self.udp_port}")
        print()

        tests = [
            ("网络风暴", lambda: self.test_network_storm(10, 500)),
            ("畸形包洪泛", lambda: self.test_malformed_packets(10)),
            ("AGV身份伪造", lambda: self.test_agv_impersonation(99, 10)),
            ("HTTP指令洪泛", lambda: self.test_http_flood(50)),
            ("急停压力", lambda: self.test_emergency_stop_stress(10)),
            ("混合流量", lambda: self.test_mixed_traffic(10)),
        ]

        for name, test_fn in tests:
            print(f"\n{'='*50}")
            print(f"测试: {name}")
            print(f"{'='*50}")
            try:
                test_fn()
            except Exception as e:
                print(f"  [错误] {e}")
                self.results[name] = {'error': str(e)}
            print(f"  等待5s系统恢复...")
            time.sleep(5)

        # 汇总
        print(f"\n{'='*50}")
        print("测试汇总:")
        print(f"{'='*50}")
        for name, result in self.results.items():
            if 'error' in result:
                print(f"  [{name}] 错误: {result['error']}")
            else:
                print(f"  [{name}] 完成")

        return self.results


# ============================================================
# 入口
# ============================================================

def main():
    parser = argparse.ArgumentParser(description='云仓灵枢 混沌工程测试')
    parser.add_argument('--server', default='127.0.0.1',
                        help='服务器IP (默认127.0.0.1)')
    parser.add_argument('--udp-port', type=int, default=5000,
                        help='UDP端口 (默认5000)')
    parser.add_argument('--http-port', type=int, default=8080,
                        help='HTTP端口 (默认8080)')
    parser.add_argument('--test', choices=[
        'storm', 'malformed', 'spoof', 'flood', 'estop', 'mixed', 'all'
    ], default='all', help='指定测试项')
    args = parser.parse_args()

    chaos = ChaosTest(args.server, args.udp_port, args.http_port)

    if args.test == 'all':
        chaos.run_all()
    elif args.test == 'storm':
        chaos.test_network_storm()
    elif args.test == 'malformed':
        chaos.test_malformed_packets()
    elif args.test == 'spoof':
        chaos.test_agv_impersonation()
    elif args.test == 'flood':
        chaos.test_http_flood()
    elif args.test == 'estop':
        chaos.test_emergency_stop_stress()
    elif args.test == 'mixed':
        chaos.test_mixed_traffic()


if __name__ == '__main__':
    main()
