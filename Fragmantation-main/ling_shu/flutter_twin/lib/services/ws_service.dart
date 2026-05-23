/// 云仓灵枢 - WebSocket连接服务

import 'dart:async';
import 'dart:convert';
import 'package:web_socket_channel/web_socket_channel.dart';
import '../models/agv_state.dart';

class WsService {
  WebSocketChannel? _channel;
  final _stateController = StreamController<FleetState>.broadcast();
  Timer? _heartbeatTimer;
  String _url = '';
  bool _connected = false;

  Stream<FleetState> get stateStream => _stateController.stream;
  bool get isConnected => _connected;

  void connect(String url) {
    _url = url;
    _doConnect();
  }

  void _doConnect() {
    try {
      _channel = WebSocketChannel.connect(Uri.parse(_url));
      _connected = true;

      _channel!.stream.listen(
        (data) {
          try {
            final json = jsonDecode(data as String);
            final state = FleetState.fromJson(json);
            _stateController.add(state);
          } catch (e) {
            print('JSON解析错误: $e');
          }
        },
        onError: (e) {
          print('WebSocket错误: $e');
          _connected = false;
          _reconnect();
        },
        onDone: () {
          print('WebSocket连接关闭');
          _connected = false;
          _reconnect();
        },
      );

      // 心跳
      _heartbeatTimer?.cancel();
      _heartbeatTimer = Timer.periodic(Duration(seconds: 30), (_) {
        _channel?.sink.add('ping');
      });

      print('WebSocket已连接: $_url');
    } catch (e) {
      print('WebSocket连接失败: $e');
      _reconnect();
    }
  }

  void _reconnect() {
    Future.delayed(Duration(seconds: 2), () {
      print('WebSocket重连中...');
      _doConnect();
    });
  }

  void sendCommand(Map<String, dynamic> cmd) {
    if (_connected) {
      _channel?.sink.add(jsonEncode(cmd));
    }
  }

  void submitTask(int goalX, int goalY) {
    sendCommand({
      'type': 'task',
      'goal_x': goalX,
      'goal_y': goalY,
    });
  }

  void emergencyStop() {
    sendCommand({'type': 'emergency'});
  }

  void dispose() {
    _heartbeatTimer?.cancel();
    _channel?.sink.close();
    _stateController.close();
  }
}
