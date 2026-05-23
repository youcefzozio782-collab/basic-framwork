/// 云仓灵枢 - Flutter数字孪生大屏入口

import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'models/agv_state.dart';
import 'services/ws_service.dart';
import 'views/twin_map_view.dart';

void main() {
  runApp(const LingShuTwinApp());
}

class LingShuTwinApp extends StatelessWidget {
  const LingShuTwinApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: '云仓灵枢 - 数字孪生',
      theme: ThemeData.dark().copyWith(
        scaffoldBackgroundColor: const Color(0xFF1a1a2e),
        cardColor: const Color(0xFF16213e),
      ),
      home: const TwinDashboard(),
    );
  }
}

class TwinDashboard extends StatefulWidget {
  const TwinDashboard({super.key});

  @override
  State<TwinDashboard> createState() => _TwinDashboardState();
}

class _TwinDashboardState extends State<TwinDashboard> {
  final WsService _wsService = WsService();
  FleetState _fleetState = FleetState.empty;
  final TextEditingController _serverController =
      TextEditingController(text: '192.168.4.1:8080');

  @override
  void initState() {
    super.initState();
    _connect();
  }

  void _connect() {
    final server = _serverController.text;
    _wsService.connect('ws://$server/ws');
    _wsService.stateStream.listen((state) {
      setState(() => _fleetState = state);
    });
  }

  @override
  void dispose() {
    _wsService.dispose();
    _serverController.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text('云仓灵枢 - AGV数字孪生'),
        actions: [
          // 连接状态指示
          Padding(
            padding: const EdgeInsets.all(8.0),
            child: Row(
              children: [
                Icon(
                  _wsService.isConnected ? Icons.circle : Icons.circle_outlined,
                  color: _wsService.isConnected ? Colors.green : Colors.red,
                  size: 12,
                ),
                const SizedBox(width: 4),
                Text(_wsService.isConnected ? '已连接' : '未连接'),
              ],
            ),
          ),
          // 服务器地址
          SizedBox(
            width: 200,
            child: TextField(
              controller: _serverController,
              style: const TextStyle(fontSize: 12),
              decoration: const InputDecoration(
                hintText: '服务器地址',
                isDense: true,
                contentPadding: EdgeInsets.symmetric(horizontal: 8, vertical: 4),
              ),
              onSubmitted: (_) => _connect(),
            ),
          ),
          IconButton(
            icon: const Icon(Icons.refresh),
            onPressed: _connect,
            tooltip: '重连',
          ),
        ],
      ),
      body: Row(
        children: [
          // 左侧: AGV状态面板
          SizedBox(
            width: 250,
            child: _buildAgvPanel(),
          ),

          // 中间: 地图
          Expanded(
            child: Center(
              child: TwinMapView(state: _fleetState),
            ),
          ),

          // 右侧: 任务和控制
          SizedBox(
            width: 300,
            child: _buildRightPanel(),
          ),
        ],
      ),
    );
  }

  Widget _buildAgvPanel() {
    return Card(
      margin: const EdgeInsets.all(8),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          const Padding(
            padding: EdgeInsets.all(12),
            child: Text('AGV状态', style: TextStyle(
              fontSize: 16, fontWeight: FontWeight.bold)),
          ),
          Expanded(
            child: ListView.builder(
              itemCount: _fleetState.agvs.length,
              itemBuilder: (context, index) {
                final agv = _fleetState.agvs[index];
                return _buildAgvCard(agv);
              },
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildAgvCard(AgvState agv) {
    final color = MapPainter.agvColors[(agv.id - 1) % 5];

    return Card(
      margin: const EdgeInsets.symmetric(horizontal: 8, vertical: 4),
      color: agv.online ? null : Colors.grey[900],
      child: Padding(
        padding: const EdgeInsets.all(12),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Row(
              children: [
                Icon(Icons.circle, color: agv.online ? color : Colors.grey,
                     size: 12),
                const SizedBox(width: 8),
                Text('AGV #${agv.id}', style: const TextStyle(
                  fontWeight: FontWeight.bold, fontSize: 14)),
                const Spacer(),
                Text(agv.statusText, style: TextStyle(
                  color: agv.status == 3 ? Colors.red : Colors.white70,
                  fontSize: 12,
                )),
              ],
            ),
            const SizedBox(height: 8),
            Row(
              children: [
                _infoChip('位置', '(${agv.posX},${agv.posY})'),
                const SizedBox(width: 8),
                _infoChip('速度', '${agv.speed}'),
              ],
            ),
            const SizedBox(height: 4),
            Row(
              children: [
                _infoChip('电量', '${agv.battery}%'),
                const SizedBox(width: 8),
                _infoChip('任务', agv.taskId > 0 ? '#${agv.taskId}' : '无'),
              ],
            ),
            // 电量条
            const SizedBox(height: 8),
            LinearProgressIndicator(
              value: agv.battery / 100,
              backgroundColor: Colors.grey[800],
              valueColor: AlwaysStoppedAnimation(
                agv.battery > 50 ? Colors.green :
                agv.battery > 20 ? Colors.orange : Colors.red,
              ),
            ),
          ],
        ),
      ),
    );
  }

  Widget _infoChip(String label, String value) {
    return Expanded(
      child: Container(
        padding: const EdgeInsets.symmetric(horizontal: 8, vertical: 4),
        decoration: BoxDecoration(
          color: Colors.white.withOpacity(0.05),
          borderRadius: BorderRadius.circular(4),
        ),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Text(label, style: const TextStyle(
              fontSize: 10, color: Colors.white54)),
            Text(value, style: const TextStyle(
              fontSize: 12, fontWeight: FontWeight.bold)),
          ],
        ),
      ),
    );
  }

  Widget _buildRightPanel() {
    return Column(
      children: [
        // 控制按钮
        Card(
          margin: const EdgeInsets.all(8),
          child: Padding(
            padding: const EdgeInsets.all(12),
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.stretch,
              children: [
                const Text('控制', style: TextStyle(
                  fontSize: 16, fontWeight: FontWeight.bold)),
                const SizedBox(height: 8),
                ElevatedButton.icon(
                  icon: const Icon(Icons.warning, color: Colors.red),
                  label: const Text('紧急停止',
                      style: TextStyle(color: Colors.red)),
                  style: ElevatedButton.styleFrom(
                    backgroundColor: Colors.red.withOpacity(0.2),
                  ),
                  onPressed: () => _wsService.emergencyStop(),
                ),
                const SizedBox(height: 8),
                ElevatedButton.icon(
                  icon: const Icon(Icons.add_location),
                  label: const Text('添加任务'),
                  onPressed: _showAddTaskDialog,
                ),
              ],
            ),
          ),
        ),

        // 任务列表
        Expanded(
          child: Card(
            margin: const EdgeInsets.all(8),
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                const Padding(
                  padding: EdgeInsets.all(12),
                  child: Text('任务队列', style: TextStyle(
                    fontSize: 16, fontWeight: FontWeight.bold)),
                ),
                Expanded(
                  child: ListView.builder(
                    itemCount: _fleetState.tasks.length,
                    itemBuilder: (context, index) {
                      final task = _fleetState.tasks[index];
                      return ListTile(
                        dense: true,
                        leading: Icon(
                          task.state == 3 ? Icons.check_circle :
                          task.state == 4 ? Icons.error :
                          task.state == 2 ? Icons.play_circle :
                          Icons.schedule,
                          color: task.state == 3 ? Colors.green :
                                 task.state == 4 ? Colors.red :
                                 Colors.blue,
                          size: 20,
                        ),
                        title: Text('任务 #${task.id}',
                            style: const TextStyle(fontSize: 13)),
                        subtitle: Text(
                          'AGV#${task.agvId} -> (${task.goalX},${task.goalY})',
                          style: const TextStyle(fontSize: 11),
                        ),
                        trailing: Text(task.stateText,
                            style: const TextStyle(fontSize: 11)),
                      );
                    },
                  ),
                ),
              ],
            ),
          ),
        ),

        // 事件日志
        Card(
          margin: const EdgeInsets.all(8),
          child: Container(
            height: 150,
            padding: const EdgeInsets.all(12),
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                const Text('事件日志', style: TextStyle(
                  fontSize: 16, fontWeight: FontWeight.bold)),
                const SizedBox(height: 8),
                Expanded(
                  child: Container(
                    padding: const EdgeInsets.all(8),
                    decoration: BoxDecoration(
                      color: Colors.black.withOpacity(0.3),
                      borderRadius: BorderRadius.circular(4),
                    ),
                    child: const SingleChildScrollView(
                      child: Text(
                        '[系统] 数字孪生已连接\n'
                        '[系统] 等待AGV数据...',
                        style: TextStyle(
                          fontFamily: 'monospace',
                          fontSize: 11,
                          color: Colors.white70,
                        ),
                      ),
                    ),
                  ),
                ),
              ],
            ),
          ),
        ),
      ],
    );
  }

  void _showAddTaskDialog() {
    final gxController = TextEditingController();
    final gyController = TextEditingController();

    showDialog(
      context: context,
      builder: (context) => AlertDialog(
        title: const Text('添加任务'),
        content: Column(
          mainAxisSize: MainAxisSize.min,
          children: [
            TextField(
              controller: gxController,
              decoration: const InputDecoration(
                labelText: '目标X坐标 (0-31)',
              ),
              keyboardType: TextInputType.number,
            ),
            TextField(
              controller: gyController,
              decoration: const InputDecoration(
                labelText: '目标Y坐标 (0-31)',
              ),
              keyboardType: TextInputType.number,
            ),
          ],
        ),
        actions: [
          TextButton(
            onPressed: () => Navigator.pop(context),
            child: const Text('取消'),
          ),
          ElevatedButton(
            onPressed: () {
              final gx = int.tryParse(gxController.text) ?? 0;
              final gy = int.tryParse(gyController.text) ?? 0;
              _wsService.submitTask(gx, gy);
              Navigator.pop(context);
            },
            child: const Text('提交'),
          ),
        ],
      ),
    );
  }
}
