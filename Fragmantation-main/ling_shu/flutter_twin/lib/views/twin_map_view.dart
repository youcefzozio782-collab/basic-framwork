/// 云仓灵枢 - 数字孪生地图视图

import 'dart:math';
import 'package:flutter/material.dart';
import '../models/agv_state.dart';

class TwinMapView extends StatelessWidget {
  final FleetState state;

  const TwinMapView({super.key, required this.state});

  @override
  Widget build(BuildContext context) {
    return Container(
      decoration: BoxDecoration(
        border: Border.all(color: Colors.grey[800]!),
        borderRadius: BorderRadius.circular(8),
      ),
      child: CustomPaint(
        painter: MapPainter(state: state),
        size: const Size(640, 640),
      ),
    );
  }
}

class MapPainter extends CustomPainter {
  final FleetState state;

  MapPainter({required this.state});

  static const int gridSize = 32;
  static const List<Color> agvColors = [
    Color(0xFFFF6B6B), // 红
    Color(0xFF4ECDC4), // 青
    Color(0xFF45B7D1), // 蓝
    Color(0xFF96ceb4), // 绿
    Color(0xFFffeaa7), // 黄
  ];

  @override
  void paint(Canvas canvas, Size size) {
    final cellW = size.width / gridSize;
    final cellH = size.height / gridSize;

    // 背景
    canvas.drawRect(
      Rect.fromLTWH(0, 0, size.width, size.height),
      Paint()..color = const Color(0xFF16213e),
    );

    // 绘制栅格
    if (state.gridMap.isNotEmpty) {
      for (int y = 0; y < gridSize && y < state.gridMap.length; y++) {
        for (int x = 0; x < gridSize && x < state.gridMap[y].length; x++) {
          final cell = state.gridMap[y][x];
          Color color;
          switch (cell) {
            case 1: color = const Color(0xFF333333); break; // 障碍
            case 2: color = const Color(0xFF0066ff); break; // 充电站
            case 3: color = const Color(0xFF664400); break; // 货架
            default: color = const Color(0xFF222222); break; // 空地
          }

          canvas.drawRect(
            Rect.fromLTWH(x * cellW, y * cellH, cellW - 1, cellH - 1),
            Paint()..color = color,
          );
        }
      }
    }

    // 绘制网格线
    final gridPaint = Paint()
      ..color = Colors.white.withOpacity(0.05)
      ..strokeWidth = 0.5;

    for (int i = 0; i <= gridSize; i++) {
      canvas.drawLine(
        Offset(i * cellW, 0), Offset(i * cellW, size.height), gridPaint);
      canvas.drawLine(
        Offset(0, i * cellH), Offset(size.width, i * cellH), gridPaint);
    }

    // 绘制AGV路径 (如果有)
    for (final agv in state.agvs) {
      if (agv.taskId > 0) {
        // 画当前位置到目标的虚线 (简化)
        final task = state.tasks.where((t) => t.id == agv.taskId).firstOrNull;
        if (task != null) {
          final pathPaint = Paint()
            ..color = agvColors[(agv.id - 1) % 5].withOpacity(0.3)
            ..strokeWidth = 2
            ..style = PaintingStyle.stroke;

          final path = Path();
          path.moveTo(agv.posX * cellW + cellW / 2,
                      agv.posY * cellH + cellH / 2);
          path.lineTo(task.goalX * cellW + cellW / 2,
                      task.goalY * cellH + cellH / 2);
          canvas.drawPath(path, pathPaint);
        }
      }
    }

    // 绘制AGV
    for (final agv in state.agvs) {
      if (!agv.online) continue;

      final cx = agv.posX * cellW + cellW / 2;
      final cy = agv.posY * cellH + cellH / 2;
      final color = agvColors[(agv.id - 1) % 5];

      // AGV圆形
      canvas.drawCircle(
        Offset(cx, cy),
        cellW / 2 - 2,
        Paint()..color = color,
      );

      // 朝向箭头
      final rad = agv.heading * pi / 180;
      final arrowLen = cellW * 0.6;
      final arrowPaint = Paint()
        ..color = Colors.white
        ..strokeWidth = 2;

      canvas.drawLine(
        Offset(cx, cy),
        Offset(cx + arrowLen * cos(rad), cy + arrowLen * sin(rad)),
        arrowPaint,
      );

      // AGV编号
      final textPainter = TextPainter(
        text: TextSpan(
          text: 'A${agv.id}',
          style: const TextStyle(
            color: Colors.white,
            fontSize: 10,
            fontWeight: FontWeight.bold,
          ),
        ),
        textDirection: TextDirection.ltr,
      )..layout();
      textPainter.paint(
        canvas,
        Offset(cx - textPainter.width / 2, cy - textPainter.height / 2),
      );

      // 电量指示 (底部小条)
      final batteryWidth = cellW - 4;
      final batteryHeight = 3.0;
      final batteryX = cx - batteryWidth / 2;
      final batteryY = cy + cellW / 2 - 1;

      canvas.drawRect(
        Rect.fromLTWH(batteryX, batteryY, batteryWidth, batteryHeight),
        Paint()..color = Colors.grey[800]!,
      );

      final batteryColor = agv.battery > 50 ? Colors.green :
                          agv.battery > 20 ? Colors.orange : Colors.red;
      canvas.drawRect(
        Rect.fromLTWH(batteryX, batteryY,
                      batteryWidth * agv.battery / 100, batteryHeight),
        Paint()..color = batteryColor,
      );
    }

    // 急停警告
    if (state.emergencyStop) {
      final warningPaint = Paint()
        ..color = Colors.red.withOpacity(0.3);
      canvas.drawRect(
        Rect.fromLTWH(0, 0, size.width, size.height),
        warningPaint,
      );

      final warningText = TextPainter(
        text: const TextSpan(
          text: '紧急停止',
          style: TextStyle(
            color: Colors.red,
            fontSize: 48,
            fontWeight: FontWeight.bold,
          ),
        ),
        textDirection: TextDirection.ltr,
      )..layout();
      warningText.paint(
        canvas,
        Offset(
          (size.width - warningText.width) / 2,
          (size.height - warningText.height) / 2,
        ),
      );
    }
  }

  @override
  bool shouldRepaint(covariant MapPainter oldDelegate) => true;
}
