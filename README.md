
#### feature 매칭 + uwb 거리 기반 맵 머징 (앵커 한개, 태그 로봇별 2개)

+ **시뮬레이션 사용 시**


```bash
ros2 launch frontier_ws turtlebot3_house_multi.launch.py
# TODO: 수정 필요
```

```bash
ros2 launch frontier_ws multi_toolbox.launch.py
# TODO: 수정 필요
```

```bash
ros2 launch frontier_ws merge_map_uwb.launch.py
# TODO: 수정 필요
```

+ 로봇 위치
```python
{'ns': 'tb3_0', 'x': '6.0', 'y': '1.0'},
{'ns': 'tb3_1', 'x': '-4.0', 'y': '4.5'},
{'ns': 'tb3_2', 'x': '-6.0', 'y': '0.0'},
```


+ **하드웨어 사용 시** 
    + ns_multi_toolbox.launch.py과 uwb_range_node.py(front, back 태그 각각 실행)는 각 로봇에서 실행
    + merge_map_uwb.launch.py는 PC에서 실행

    + 로봇 0
      ```bash
      python3 ~/ros2_ws/src/frontier_ws/scripts/uwb_range_node_uwb.py \
        --ros-args \
        -r __node:=uwb_range_tb3_0_front \
        -p serial_port:=/dev/ttyUSB_tb3_0_front \
        -p anchor_index:=0 \
        -p tag_name:=front \
        -r uwb/front/range:=/tb3_0/uwb/front/range
      ```
      ```bash
      python3 ~/ros2_ws/src/frontier_ws/scripts/uwb_range_node_uwb.py \
        --ros-args \
        -r __node:=uwb_range_tb3_0_back \
        -p serial_port:=/dev/ttyUSB_tb3_0_back \
        -p anchor_index:=0 \
        -p tag_name:=back \
        -r uwb/back/range:=/tb3_0/uwb/back/range
      ```
      ```bash
      ros2 launch turtlebot3_bringup robot.launch.py namespace:=tb3_0
      ```
      ```bash
      ros2 launch frontier_ws ns_multi_toolbox.launch.py robot_namespace:=tb3_0
      ```

      + 로봇 1
      ```bash
      python3 ~/ros2_ws/src/frontier_ws/scripts/uwb_range_node_uwb.py \
        --ros-args \
        -r __node:=uwb_range_tb3_1_front \
        -p serial_port:=/dev/ttyUSB_tb3_0_front \
        -p anchor_index:=0 \
        -p tag_name:=front \
        -r uwb/front/range:=/tb3_1/uwb/front/range
      ```
      ```bash
      python3 ~/ros2_ws/src/frontier_ws/scripts/uwb_range_node_uwb.py \
        --ros-args \
        -r __node:=uwb_range_tb3_1_back \
        -p serial_port:=/dev/ttyUSB_tb3_0_back \
        -p anchor_index:=0 \
        -p tag_name:=back \
        -r uwb/back/range:=/tb3_1/uwb/back/range
      ```
      ```bash
      ros2 launch turtlebot3_bringup robot.launch.py namespace:=tb3_1
      ```
      ```bash
      ros2 launch frontier_ws ns_multi_toolbox.launch.py robot_namespace:=tb3_1
      ```

      + 로봇 2
      ```bash
      python3 ~/ros2_ws/src/frontier_ws/scripts/uwb_range_node_uwb.py \
        --ros-args \
        -r __node:=uwb_range_tb3_2_front \
        -p serial_port:=/dev/ttyUSB_tb3_2_front \
        -p anchor_index:=0 \
        -p tag_name:=front \
        -r uwb/front/range:=/tb3_2/uwb/front/range
      ```
      ```bash
      python3 ~/ros2_ws/src/frontier_ws/scripts/uwb_range_node_uwb.py \
        --ros-args \
        -r __node:=uwb_range_tb3_2_back \
        -p serial_port:=/dev/ttyUSB_tb3_2_back \
        -p anchor_index:=0 \
        -p tag_name:=back \
        -r uwb/back/range:=/tb3_2/uwb/back/range
      ```
      ```bash
      ros2 launch turtlebot3_bringup robot.launch.py namespace:=tb3_2
      ```
      ```bash
      ros2 launch frontier_ws ns_multi_toolbox.launch.py robot_namespace:=tb3_2
      ```

    + PC
      ```bash
      ros2 launch frontier_ws merge_map_uwb.launch.py
      # TODO: 수정 필요
      ```
 

---
#### local map(.pgm, .yaml 파일) 각각 2개로 feature 기반 맵 머징 확인 가능

+ 맵 저장
  ```bash
  ros2 run nav2_map_server map_saver_cli \
    -f ~/<원하는 맵 이름> \
    --ros-args -r map:=<저장할 맵 토픽>
  ```

+ 맵 회전
  + 코드에서 회전할 맵 이름 수정 후
  ```bash
  python3 rotate_map.py
  ```

+ featuer만 사용해 맵 머징
  ```bash
  python3 scripts/offline_merge_map_uwb.py \
  <파일 이름>.yaml \
  <파일 이름>.yaml
  ```
