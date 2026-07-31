알고리즘이 더 정확한지 확인하기 위해 실험하는 거라 home에 새 패키지를 추가하여 클론하였음

```bash
# 터틀봇
sbt 로 소싱 후 진행
ros2 launch turtlebot3_bringup robot.launch.py namespace:=tb3_0
ros2 launch frontier_ws ns_multi_toolbox.launch.py robot_namespace:=tb3_0

# pc
ros2 launch merge_map merge_map_launch.py
```
