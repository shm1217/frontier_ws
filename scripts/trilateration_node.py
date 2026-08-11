#!/usr/bin/env python3
"""
UWB DW3000 multi-anchor tag의 시리얼 출력을 읽어서
삼변측량(trilateration)으로 태그의 2D 위치(x, y)를 계산하고
ROS2 topic으로 publish하는 노드.

기대하는 시리얼 라인 포맷 (Makerfabs_Multi_Anchor_Multi_Tag_Tag.ino 의 print_ranging_result() 출력):
    T0,mask:7,seq:5,beacon:3,range:(120,340,200,0,0,0,0,0),ancid:(0,1,2,-1,-1,-1,-1,-1)

range:(...) 안의 값은 각 앵커까지의 거리 (cm 단위, ranges_cm 배열 인덱스 = anchor_id - 0xA0).
mask 는 응답받은 앵커를 나타내는 비트마스크 (bit0=anchor0, bit1=anchor1, ...).
"""

import re
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float32MultiArray

try:
    import serial
except ImportError:
    serial = None


# 시리얼 출력 파싱용 정규식
LINE_RE = re.compile(
    r"T(?P<tag_id>\d+),mask:(?P<mask>[0-9A-Fa-f]+),seq:(?P<seq>\d+),"
    r"beacon:(?P<beacon>\d+),range:\((?P<ranges>[-\d,]+)\),ancid:\((?P<ancid>[-\d,]+)\)"
)


class TrilaterationNode(Node):
    def __init__(self):
        super().__init__("uwb_trilateration_node")

        # ---- 파라미터 선언 ----
        self.declare_parameter("serial_port", "/dev/ttyUSB0")
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("frame_id", "map")

        # 앵커 좌표 (미터 단위). 기본값은 한 변 3m 정삼각형 예시 배치.
        # 실측 후 launch 파일이나 커맨드라인에서 덮어쓰세요.
        # anchor_x[i], anchor_y[i] 는 anchor_id = 0xA0 + i 에 대응됩니다.
        self.declare_parameter("anchor_x", [0.0, 3.0, 1.5])
        self.declare_parameter("anchor_y", [0.0, 0.0, 2.6])

        self.serial_port = self.get_parameter("serial_port").value
        self.baud_rate = self.get_parameter("baud_rate").value
        self.frame_id = self.get_parameter("frame_id").value
        self.anchor_x = list(self.get_parameter("anchor_x").value)
        self.anchor_y = list(self.get_parameter("anchor_y").value)

        if len(self.anchor_x) != len(self.anchor_y):
            self.get_logger().error(
                "anchor_x, anchor_y 길이가 다릅니다. 파라미터를 확인하세요."
            )

        self.num_anchors = len(self.anchor_x)
        self.get_logger().info(
            f"앵커 {self.num_anchors}개 좌표: "
            + ", ".join(
                f"A{i}=({x:.2f},{y:.2f})"
                for i, (x, y) in enumerate(zip(self.anchor_x, self.anchor_y))
            )
        )

        # ---- Publisher ----
        self.position_pub = self.create_publisher(PointStamped, "uwb/position", 10)
        self.ranges_pub = self.create_publisher(Float32MultiArray, "uwb/ranges", 10)

        # ---- 시리얼 연결 ----
        if serial is None:
            self.get_logger().error(
                "pyserial이 설치되어 있지 않습니다. 'pip install pyserial --break-system-packages' 로 설치하세요."
            )
            self.ser = None
        else:
            try:
                self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1.0)
                self.get_logger().info(
                    f"{self.serial_port} @ {self.baud_rate} 연결 완료"
                )
            except serial.SerialException as e:
                self.get_logger().error(f"시리얼 포트 열기 실패: {e}")
                self.ser = None

        # 주기적으로 시리얼 버퍼 확인 (50Hz 폴링, 실제 갱신 속도는 TDMA 프레임 주기를 따름)
        self.timer = self.create_timer(0.02, self.read_serial_callback)

    def read_serial_callback(self):
        if self.ser is None or self.ser.in_waiting == 0:
            return

        try:
            raw_line = self.ser.readline().decode("utf-8", errors="ignore").strip()
        except Exception as e:
            self.get_logger().warn(f"시리얼 읽기 에러: {e}")
            return

        if not raw_line:
            return

        self.process_line(raw_line)

    def process_line(self, line: str):
        match = LINE_RE.search(line)
        if not match:
            # 예상 포맷이 아닌 라인(부팅 로그 등)은 조용히 무시
            return

        ranges_cm = [int(v) for v in match.group("ranges").split(",")]
        mask = int(match.group("mask"), 16)

        # cm -> m 변환, 필요한 앵커 개수만큼만 사용
        ranges_m = [v / 100.0 for v in ranges_cm[: self.num_anchors]]

        # ranges 디버그 topic publish
        ranges_msg = Float32MultiArray()
        ranges_msg.data = ranges_m
        self.ranges_pub.publish(ranges_msg)

        # 필요한 모든 앵커로부터 응답을 받았는지 확인 (mask의 하위 num_anchors 비트가 다 서있어야 함)
        required_mask = (1 << self.num_anchors) - 1
        if (mask & required_mask) != required_mask:
            self.get_logger().debug(
                f"일부 앵커 응답 누락 (mask={mask:#x}), 이번 프레임 스킵"
            )
            return

        position = self.trilaterate(ranges_m)
        if position is None:
            return

        x, y = position
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.point.x = x
        msg.point.y = y
        msg.point.z = 0.0
        self.position_pub.publish(msg)

    def trilaterate(self, ranges_m):
        """
        3개 이상의 앵커 거리로 2D 위치 계산.
        3개면 선형화된 연립방정식을 정확히 풀고,
        4개 이상이면 least-squares로 근사.
        """
        n = self.num_anchors
        if n < 3:
            self.get_logger().warn("앵커가 3개 미만이라 2D 위치를 확정할 수 없습니다.")
            return None

        x1, y1 = self.anchor_x[0], self.anchor_y[0]
        d1 = ranges_m[0]

        # 기준 앵커(0번) 대비 나머지 앵커들로 선형 방정식 구성
        # 2*(xi-x1)*x + 2*(yi-y1)*y = d1^2 - di^2 + xi^2 - x1^2 + yi^2 - y1^2
        A = []
        b = []
        for i in range(1, n):
            xi, yi = self.anchor_x[i], self.anchor_y[i]
            di = ranges_m[i]
            A.append([2 * (xi - x1), 2 * (yi - y1)])
            b.append(d1**2 - di**2 + xi**2 - x1**2 + yi**2 - y1**2)

        try:
            if n == 3:
                # 2x2 연립방정식 직접 풀이 (Cramer's rule)
                (a11, a12), (a21, a22) = A
                det = a11 * a22 - a12 * a21
                if abs(det) < 1e-9:
                    self.get_logger().warn(
                        "앵커들이 일직선에 가까워 위치 계산이 불안정합니다."
                    )
                    return None
                x = (b[0] * a22 - b[1] * a12) / det
                y = (a11 * b[1] - a21 * b[0]) / det
            else:
                # 4개 이상: least squares (외부 라이브러리 없이 정규방정식으로 직접 계산)
                x, y = self._least_squares_2x2(A, b)
        except ZeroDivisionError:
            self.get_logger().warn("위치 계산 중 0으로 나누기 발생, 이번 프레임 스킵")
            return None

        return x, y

    @staticmethod
    def _least_squares_2x2(A, b):
        # A^T A x = A^T b 를 직접 전개해서 계산 (numpy 없이도 동작하도록)
        s_xx = sum(row[0] * row[0] for row in A)
        s_xy = sum(row[0] * row[1] for row in A)
        s_yy = sum(row[1] * row[1] for row in A)
        s_xb = sum(row[0] * bi for row, bi in zip(A, b))
        s_yb = sum(row[1] * bi for row, bi in zip(A, b))

        det = s_xx * s_yy - s_xy * s_xy
        if abs(det) < 1e-9:
            raise ZeroDivisionError("정규방정식 행렬이 특이(singular)합니다.")

        x = (s_xb * s_yy - s_yb * s_xy) / det
        y = (s_xx * s_yb - s_xy * s_xb) / det
        return x, y


def main(args=None):
    rclpy.init(args=args)
    node = TrilaterationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
