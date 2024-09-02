# 이동 명령

## 목적지 정보 출력

다음 명령어를 입력하면 목적지의 정보를 출력한다.

```bash
rostopic echo /move_base/goal
```

해당 명령 실행 후 navigation에서 2D Nav Goal을 설정하면 아래와 같은 정보가 출력 된다.

```yaml
header: 
  seq: 1
  stamp: 
    secs: 1724895114
    nsecs: 413630846
  frame_id: ''
goal_id: 
  stamp: 
    secs: 0
    nsecs:         0
  id: ''
goal: 
  target_pose: 
    header: 
      seq: 1
      stamp: 
        secs: 1724895114
        nsecs: 413409736
      frame_id: "map"
    pose: 
      position: 
        x: -0.011982561089098454
        y: -0.09414872527122498
        z: 0.0
      orientation: 
        x: 0.0
        y: 0.0
        z: 0.7097600795970498
        w: 0.7044434891532391
```

- pose
    - position (Vector3)
        - x: 원점 기준 X 좌표 (meter)
        - y: 원점 기준 Y 좌표 (meter)
        - z: 원점 기준 Z 좌표 (2D map에서 사용하지 않음)
        - 터틀봇 명령시 x, y 사용
    - orientation (Quaternion)
        - x: X 방향값
        - y: Y 방향값
        - z: Z 방향값
        - w: W 방향값
        - 터틀봇 명령 시 w 사용

## 목적지 정보 전송

명령어로 목적지를 설정하려면 다음을 입력한다.

```bash
rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped '{header: {stamp: now, frame_id: "map"}, pose: {position: { x: 2.95, y: 10.45, z: -0.67}, orientation: { w: 0.7365651243434441}}}'
```

- /move_base_simple/goal
    - geometry_msgs/PostStamped 메시지를 이용하여 목표 지점으로 이동시키는 topic
- geometry_msgs/PostStamped
    - 위치와 방향을 담고 있는 메시지
    - 시간 정보와 프레임 ID를 포함하고 있음