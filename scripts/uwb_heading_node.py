#!/usr/bin/env python3
"""
UWB front/back 태그 2개의 위치(PointStamped)를 구독해서
로봇의 heading(yaw)과 base_link 위치를 추정하고,
world(anchor) -> {namespace}/map 의 static TF를 1회 broadcast하는 노드.

원리:
- front tag 위치: (xf, yf)  in world(anchor) frame
- back  tag 위치: (xb, yb)  in world(anchor) frame
- yaw = atan2(yf - yb, xf - xb)   (front-back 벡터 = 로봇 forward 축이라고 가정)
- base_link 위치 = front/back 중점 (두 태그가 base_link 기준 대칭 마운트라고 가정)
  대칭이 아니라면 --tag-offset 으로 back으로부터 base_link까지 거리를 지정해서 보정

노이즈를 줄이기 위해 --num-samples 개의 (front, back) 쌍을 모아 각각 평균낸 뒤
한 번만 static TF를 broadcast하고 노드를 종료합니다 (필요하면 계속 켜둘 수도 있음).

사용 예:
  ros2 run <pkg> uwb_heading_node.py --ros-args \
      -p namespace:=tb3_0 \
      -p front_topic:=/tb3_0/uwb_front/position \
      -p back_topic:=/tb3_0/uwb_back/position \
      -p world_frame:=world \
      -p num_samples:=30 \
      -p tag_offset_from_base:=0.0
"""

import math
from collections import deque

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, TransformStamped
from tf2_ros import StaticTransformBroadcaster


def quaternion_from_yaw(yaw: float):
    return [0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)]


class UwbHeadingNode(Node):
    def __init__(self):
        super().__init__("uwb_heading_node")

        # ---- 파라미터 ----
        self.declare_parameter("namespace", "tb3_0")
        self.declare_parameter("front_topic", "uwb_front/position")
        self.declare_parameter("back_topic", "uwb_back/position")
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("num_samples", 30)  # 평균낼 샘플 개수
        self.declare_parameter(
            "max_sample_age_sec", 0.5
        )  # front/back 짝 맞출 때 허용 시간차
        # 두 태그의 중점이 base_link와 다르면(예: back 태그가 base_link에 더 가까움),
        # back 태그로부터 base_link까지의 forward 방향 거리(m)를 넣어 보정.
        # 0.0이면 "중점 = base_link"로 그대로 사용.
        self.declare_parameter("tag_offset_from_back", 0.0)
        self.declare_parameter(
            "keep_publishing", False
        )  # True면 종료하지 않고 num_samples마다 갱신

        self.namespace = self.get_parameter("namespace").value
        front_topic = self.get_parameter("front_topic").value
        back_topic = self.get_parameter("back_topic").value
        self.world_frame = self.get_parameter("world_frame").value
        self.num_samples = int(self.get_parameter("num_samples").value)
        self.max_sample_age = float(self.get_parameter("max_sample_age_sec").value)
        self.tag_offset_from_back = float(
            self.get_parameter("tag_offset_from_back").value
        )
        self.keep_publishing = bool(self.get_parameter("keep_publishing").value)

        self.map_frame = f"{self.namespace}/map"

        self._latest_front = None  # (x, y, stamp_sec)
        self._latest_back = None

        self._front_samples = deque(maxlen=self.num_samples)
        self._back_samples = deque(maxlen=self.num_samples)

        self._done = False

        self.static_broadcaster = StaticTransformBroadcaster(self)

        self.create_subscription(PointStamped, front_topic, self._front_cb, 10)
        self.create_subscription(PointStamped, back_topic, self._back_cb, 10)

        self.get_logger().info(
            f'[{self.namespace}] front="{front_topic}" back="{back_topic}" '
            f"num_samples={self.num_samples} 대기 중..."
        )

    def _front_cb(self, msg: PointStamped):
        t = self.get_clock().now().nanoseconds / 1e9
        self._latest_front = (msg.point.x, msg.point.y, t)
        self._try_pair()

    def _back_cb(self, msg: PointStamped):
        t = self.get_clock().now().nanoseconds / 1e9
        self._latest_back = (msg.point.x, msg.point.y, t)
        self._try_pair()

    def _try_pair(self):
        if self._done and not self.keep_publishing:
            return
        if self._latest_front is None or self._latest_back is None:
            return

        xf, yf, tf_time = self._latest_front
        xb, yb, tb_time = self._latest_back

        if abs(tf_time - tb_time) > self.max_sample_age:
            return

        self._front_samples.append((xf, yf))
        self._back_samples.append((xb, yb))

        if len(self._front_samples) >= self.num_samples:
            self._compute_and_broadcast()
            if not self.keep_publishing:
                self._done = True
                self.get_logger().info(
                    f"[{self.namespace}] TF broadcast 완료, 노드 종료합니다."
                )
                # static TF가 이미 latched 되었으니 노드가 죽어도 TF는 유지됨
                rclpy.shutdown()
            else:
                self._front_samples.clear()
                self._back_samples.clear()

    def _compute_and_broadcast(self):
        xf_avg = sum(p[0] for p in self._front_samples) / len(self._front_samples)
        yf_avg = sum(p[1] for p in self._front_samples) / len(self._front_samples)
        xb_avg = sum(p[0] for p in self._back_samples) / len(self._back_samples)
        yb_avg = sum(p[1] for p in self._back_samples) / len(self._back_samples)

        dx = xf_avg - xb_avg
        dy = yf_avg - yb_avg
        baseline = math.hypot(dx, dy)

        if baseline < 1e-3:
            self.get_logger().warn(
                f"[{self.namespace}] front/back 태그 거리가 거의 0입니다 "
                f"(baseline={baseline:.3f}m). yaw 계산이 불안정할 수 있습니다."
            )

        yaw = math.atan2(dy, dx)

        # 중점 (기본: base_link 위치로 사용)
        mid_x = (xf_avg + xb_avg) / 2.0
        mid_y = (yf_avg + yb_avg) / 2.0

        # 중점이 아니라 back 기준 offset으로 base_link를 지정하고 싶다면 보정
        if abs(self.tag_offset_from_back) > 1e-9:
            base_x = xb_avg + self.tag_offset_from_back * math.cos(yaw)
            base_y = yb_avg + self.tag_offset_from_back * math.sin(yaw)
        else:
            base_x = mid_x
            base_y = mid_y

        # world -> {namespace}/map 변환 계산
        # 가정: 로봇이 SLAM(map) 시작 시점에 map 프레임 원점(0,0,yaw=0)에서 시작
        # => world 기준 base_link 위치/자세 = world -> map 변환 그 자체
        map_tx = base_x
        map_ty = base_y
        map_yaw = yaw

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.world_frame
        t.child_frame_id = self.map_frame
        t.transform.translation.x = float(map_tx)
        t.transform.translation.y = float(map_ty)
        t.transform.translation.z = 0.0
        q = quaternion_from_yaw(map_yaw)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.static_broadcaster.sendTransform(t)

        self.get_logger().info(
            f"[{self.namespace}] baseline={baseline:.3f}m, "
            f"yaw={math.degrees(yaw):.1f}deg, "
            f"{self.world_frame} -> {self.map_frame} = "
            f"(x={map_tx:.3f}, y={map_ty:.3f}, yaw={math.degrees(map_yaw):.1f}deg) publish 완료"
        )

        if baseline < 0.15:
            self.get_logger().warn(
                f"[{self.namespace}] baseline이 {baseline:.2f}m로 짧습니다. "
                f"UWB 위치 오차(±10~30cm 수준) 대비 yaw 오차가 커질 수 있으니 "
                f"가능하면 태그 간격을 더 벌리는 것을 권장합니다."
            )


def main(args=None):
    rclpy.init(args=args)
    node = UwbHeadingNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
