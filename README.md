
#### feature 매칭 + uwb 거리 기반 맵 머징 (앵커 한개, 태그 로봇별 2개)

+ **시뮬레이션 사용 시**

gazebo 실행 후 


```bash
ros2 launch frontier_ws multi_toolbox.launch.py
```

```bash
ros2 launch frontier_ws merge_map_uwb.launch.py
```


+ **하드웨어 사용 시** 
    + merge_map_uwb.launch.py 파일에서 시뮬레이션 부분 주석 처리
    + ns_multi_toolbox.launch.py과 uwb_range_node.py(front, back 태그 각각 실행)는 각 로봇에서 실행
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
      ros2 launch frontier_ws ns_multi_toolbox.launch.py robot_namespace:=tb3_0
      ```
    + merge_map_uwb.launch.py은 중앙 pc에서 실행
      ```bash
      ros2 launch frontier_ws merge_map_uwb.launch.py
 

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

------
#### 프론티어 노드 관련
알고리즘이 더 정확한지 확인하기 위해 실험하는 거라 home에 새 패키지(test_ws/src/frontier_ws)를 추가하여 클론하였음

# 터틀봇 ros2_ws와 test_ws에 동일한 frontier_ws가 있기 때문에 이 패키지를 실험하기 위해서는 test_ws를 소싱해주는 것이 중요함

```bash
# 터틀봇
sbt 로 소싱 후 진행
ros2 launch turtlebot3_bringup robot.launch.py namespace:=tb3_0
ros2 launch frontier_ws ns_multi_toolbox.launch.py robot_namespace:=tb3_0

# pc
ros2 launch merge_map merge_map_launch.py
```
  
