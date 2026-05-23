/// 云仓灵枢 - AGV状态数据模型

class AgvState {
  final int id;
  final int posX;
  final int posY;
  final int heading;
  final int speed;
  final int battery;
  final int status; // 0=idle, 1=moving, 2=charging, 3=error
  final int taskId;
  final bool online;

  const AgvState({
    required this.id,
    this.posX = 0,
    this.posY = 0,
    this.heading = 0,
    this.speed = 0,
    this.battery = 100,
    this.status = 0,
    this.taskId = 0,
    this.online = true,
  });

  factory AgvState.fromJson(Map<String, dynamic> json) => AgvState(
    id: json['id'] ?? 0,
    posX: json['x'] ?? 0,
    posY: json['y'] ?? 0,
    heading: json['heading'] ?? 0,
    speed: json['speed'] ?? 0,
    battery: json['battery'] ?? 0,
    status: json['status'] ?? 0,
    taskId: json['task_id'] ?? 0,
    online: json['online'] == 1,
  );

  String get statusText {
    switch (status) {
      case 0: return '空闲';
      case 1: return '移动中';
      case 2: return '充电中';
      case 3: return '故障';
      default: return '未知';
    }
  }

  @override
  String toString() => 'AGV#$id($posX,$posY) $statusText bat=$battery%';
}

class TaskInfo {
  final int id;
  final int agvId;
  final int goalX;
  final int goalY;
  final int state; // 0=pending, 1=planning, 2=executing, 3=done, 4=failed

  const TaskInfo({
    required this.id,
    this.agvId = 0,
    this.goalX = 0,
    this.goalY = 0,
    this.state = 0,
  });

  factory TaskInfo.fromJson(Map<String, dynamic> json) {
    final goal = json['goal'] as List? ?? [0, 0];
    return TaskInfo(
      id: json['id'] ?? 0,
      agvId: json['agv'] ?? 0,
      goalX: goal.length > 0 ? goal[0] : 0,
      goalY: goal.length > 1 ? goal[1] : 0,
      state: json['state'] ?? 0,
    );
  }

  String get stateText {
    switch (state) {
      case 0: return '等待';
      case 1: return '规划中';
      case 2: return '执行中';
      case 3: return '完成';
      case 4: return '失败';
      default: return '未知';
    }
  }
}

class FleetState {
  final List<AgvState> agvs;
  final List<List<int>> gridMap;
  final List<TaskInfo> tasks;
  final int timestamp;
  final bool emergencyStop;

  const FleetState({
    this.agvs = const [],
    this.gridMap = const [],
    this.tasks = const [],
    this.timestamp = 0,
    this.emergencyStop = false,
  });

  factory FleetState.fromJson(Map<String, dynamic> json) {
    final agvList = (json['agvs'] as List? ?? [])
        .map((a) => AgvState.fromJson(a))
        .toList();

    final grid = (json['grid'] as List? ?? [])
        .map((row) => (row as List).map((c) => c as int).toList())
        .toList();

    final taskList = (json['tasks'] as List? ?? [])
        .map((t) => TaskInfo.fromJson(t))
        .toList();

    return FleetState(
      agvs: agvList,
      gridMap: grid,
      tasks: taskList,
      timestamp: json['tick'] ?? 0,
      emergencyStop: json['emergency'] == 1,
    );
  }

  static const empty = FleetState();
}
