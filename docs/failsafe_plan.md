# Fail-safe 상태 publish 계획

## 목적

ADAS node가 UI에서 감시할 수 있는 시스템 상태를 주기적으로 publish한다.

이 기능의 1차 목표는 UI를 직접 구현하는 것이 아니라,
UI가 구독할 수 있는 상태 topic을 ADAS pipeline에서 제공하는 것이다.

## 상태 정의

- `OK`
  - ADAS node가 실행 중이고 LiDAR frame이 정상적으로 들어오는 상태

- `SENSOR_TIMEOUT`
  - ADAS node는 실행 중이지만, 설정한 시간 동안 LiDAR frame이 들어오지 않은 상태
  - UI는 운전자에게 센서 이상 및 직접 판단 필요 메시지를 표시해야 한다

- `SYSTEM_NOT_RUNNING`
  - ADAS node 또는 컴퓨터가 정상 작동하지 않아 UI가 heartbeat message를 받지 못하는 상태
  - 이 상태는 ADAS node가 직접 publish할 수 없다
  - UI가 `/adas/system_status` message timeout을 보고 판단해야 한다

## 제안 ROS Topic

- Topic: `/adas/system_status`
- Message type: `std_msgs/msg/String`
- Payload: JSON string

초기 구현에서는 custom ROS message를 만들지 않고 `std_msgs/msg/String`에 JSON을 담는다.
이 방식은 UI와의 contract를 빠르게 검증하기 쉽고, 기존 ROS package 구조 변경을 최소화할 수 있다.

## 제안 Payload

정상 상태:

```json
{
  "status": "OK",
  "system_alive": true,
  "lidar_alive": true,
  "message": "ADAS system operating.",
  "last_lidar_age_sec": 0.12
}
```

LiDAR timeout 상태:

```json
{
  "status": "SENSOR_TIMEOUT",
  "system_alive": true,
  "lidar_alive": false,
  "message": "LiDAR data timeout. Driver takeover required.",
  "last_lidar_age_sec": 1.54
}
```

## 초기 기준값

- status publish 주기: `0.5 sec`
- LiDAR timeout 기준: `1.0 sec`
- UI heartbeat timeout 기준: `1.0 sec`

## 판단 책임 분리

### ADAS node가 판단하는 것

- LiDAR frame이 최근 timeout 안에 들어왔는지
- `OK` 또는 `SENSOR_TIMEOUT` 상태

### UI가 판단하는 것

- `/adas/system_status` message가 계속 들어오는지
- message가 일정 시간 이상 끊기면 `SYSTEM_NOT_RUNNING`으로 판단

## 테스트 계획

1. ADAS node를 실행한다.
2. `ros2 topic echo /adas/system_status`로 status message가 publish되는지 확인한다.
3. rosbag을 재생하고 `OK` 상태가 나오는지 확인한다.
4. rosbag을 중지하고 timeout 이후 `SENSOR_TIMEOUT`으로 바뀌는지 확인한다.
5. ADAS node를 종료하고 status message가 끊기는지 확인한다.

## 주의사항

- `SYSTEM_NOT_RUNNING`은 node가 죽은 상태이므로 node가 직접 publish할 수 없다.
- 따라서 UI는 heartbeat message가 끊기는 상황을 별도로 감시해야 한다.
- 이 작업은 실제 차량 제어 command를 만들지 않는다.
- 기존 perception, tracking, prediction 로직은 변경하지 않고 status publish만 추가한다.