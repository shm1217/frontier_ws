#### feature 매칭 + uwb 거리 기반 맵 머징

+ **시뮬레이션 사용 시**

gazebo 실행 후 


```bash
ros2 launch frontier_ws multi_toolbox.launch.py
```

```bash
ros2 launch frontier_ws merge_map_uwb.launch.py
```


+ **하드웨어 사용 시** 
    + merge_map_uwb.launch.py 파일에서 시뮬레이션 부분은 주석 처리, 하드웨어 부분은 주석 풀고 진행 
    + ns_toolbox.launch.py는 각 로봇에서 실행
    + merge_map_uwb.launch.py은 중앙 pc에서 실행
 

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
  

