+ frontier_ws 실행 (yolo_node 제외)

```bash
ros2 launch frontier_ws ns_multi_toolbox.launch.py robot_namespace:=tb3_0
```

+ yolo_node 실행
```bash
ros2 launch yolo_bringup yolo.launch.py   input_image_topic:=/tb3_0/camera/camera/color/image_raw   target_frame:=tb3_0/camera_link   input_depth_topic:=/tb3_0/camera/camera/aligned_depth_to_color/image_raw   input_depth_info_topic:=/tb3_0/camera/camera/color/camera_info   use_3d:=True   use_tracking:=False   use_debug:=False   namespace:=tb3_0/yolo  imgsz_height:=192  imgsz_width:=320  max_det:=10  use_sim_time:=false  half:=false  augment:=false  agnostic_nms:=false  retina_masks:=false  device:=cpu  model:=yolov8n.pt
```

---

* 수정한 부분
  * robot_namespace별로 한번씩만 노드 실행되도록 하는 ns_multi_toolbox.launch.py 생성
  * 대표이미지 경로 상대 경로로 수정  
