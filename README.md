# 광주 과학관 SLAM & Navigation 매뉴얼

작성 일시: 2026-07-23 09:45 KST

## 1. NoMachine 연결 방법

NoMachine은 원격 접속을 위해 사용하는 프로그램입니다.  
`go2_edu`에는 기본적으로 NoMachine이 설치되어 있습니다.

원격 접속을 하기 위한 조건은 다음과 같습니다.

1. `go2`에 더미 HDMI 또는 디스플레이가 연결되어 있어야 합니다.
2. `go2` 제어기와 원격 접속 PC가 같은 Wi-Fi에 연결되어 있어야 합니다.
3. `go2` 제어기의 IP를 알고 있어야 합니다.

IP 주소는 다음 경로에서 확인할 수 있습니다.

```text
Settings -> Wi-Fi -> 연결된 Wi-Fi 설정 버튼
```

## 2. 센서 브링업

센서와 로봇 드라이버만 먼저 실행할 때 사용합니다.

```bash
ros2 launch wego teleop_launch.py
```

RViz까지 함께 확인하려면 `gui:=true` 옵션을 사용합니다.

```bash
ros2 launch wego teleop_launch.py gui:=true
```

## 3. SLAM 실행 및 맵 저장

SLAM은 아래 명령으로 실행합니다.

```bash
ros2 launch wego slam_launch.py
```

참고 사항:

- `slam_launch.py`는 기본값으로 센서 브링업을 함께 실행합니다.
- 따라서 SLAM을 실행할 때 센서 브링업 명령을 별도로 실행하지 않아도 됩니다.
- 매핑 시 로봇을 천천히 조작해야 합니다.
- 로봇을 빠르게 조작하면 생성되는 맵이 흔들릴 수 있습니다.

SLAM으로 맵을 만든 뒤, 새 터미널에서 맵 저장 명령어를 입력합니다.

```bash
ros2 run nav2_map_server map_saver_cli -f <저장할_경로/맵_파일_이름>
```

예시:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/wego_ws/src/go2-foxy/wego_2d_nav/maps/my_map
```

## 4. 저장한 맵 불러오기 설정

맵을 저장한 뒤 Navigation에서 해당 맵을 불러오기 위해 아래 파일을 수정합니다.

```text
~/wego_ws/src/go2-foxy/wego/launch/nav2_bringup_launch.py
```

수정 위치:

```text
line 54
```

현재 설정된 `map1`을 저장한 맵 이름으로 수정합니다.

예를 들어 저장한 맵 이름이 `my_map`이라면 `map1` 대신 `my_map`을 사용합니다.

## 5. SLAM 맵 편집

맵 편집은 GIMP 툴로 진행합니다.

```bash
gimp <맵_경로/맵_파일_이름.pgm>
```

예시:

```bash
gimp ~/wego_ws/src/go2-foxy/wego_2d_nav/maps/my_map.pgm
```

## 6. Navigation 실행

다음 명령어로 Navigation을 실행합니다.

```bash
ros2 launch wego nav2_bringup_launch.py
```

## 7. Navigation 사용 절차

Navigation 실행 후 다음 순서로 주행을 진행합니다.

1. 로봇의 초기 위치와 방향을 지정합니다.
2. 로봇 초기 설정을 완료한 뒤 `Goal Pose`를 지정합니다.
3. 지정한 포인트로 로봇이 주행합니다.

다중 waypoint를 사용하는 경우 다음 순서로 진행합니다.

1. `Waypoint mode`로 진입합니다.
2. 다중 `Goal point`를 지정합니다.
3. `Start Navigation`으로 주행을 시작합니다.

## 8. Navigation 재실행 시 주의사항

Navigation을 재실행할 때 노드가 꼬일 수 있으므로, 사전에 초기화를 진행해야 합니다.

```bash
ros2 daemon stop
```

초기화 후 Navigation을 다시 실행합니다.

```bash
ros2 launch wego nav2_bringup_launch.py
```

## 전체 실행 흐름 요약

1. NoMachine으로 `go2`에 원격 접속합니다.
2. SLAM을 실행합니다.

```bash
ros2 launch wego slam_launch.py
```

3. 로봇을 천천히 조작하며 맵을 생성합니다.
4. 새 터미널에서 맵을 저장합니다.

```bash
ros2 run nav2_map_server map_saver_cli -f ~/wego_ws/src/go2-foxy/wego_2d_nav/maps/my_map
```

5. `nav2_bringup_launch.py`에서 사용할 맵 이름을 수정합니다.
6. 필요하면 GIMP로 `.pgm` 맵을 편집합니다.
7. Navigation을 실행합니다.

```bash
ros2 launch wego nav2_bringup_launch.py
```

8. RViz에서 초기 위치, 방향, 목표 지점을 지정해 주행합니다.
