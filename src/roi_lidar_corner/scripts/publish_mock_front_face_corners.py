#!/usr/bin/env python3
"""Standalone mock publisher for /roi_lidar_corner/front_face_corners.

Zero external dependencies — all constants, corner coordinates, and message
structure are hardcoded.  Only requires: ROS2 sourced + the external
FrontFaceCorners message package available (rclpy hard requirement).

Usage:
    python3 publish_mock_front_face_corners.py --solution-state tracking
    python3 publish_mock_front_face_corners.py --solution-state 0
    python3 publish_mock_front_face_corners.py --solution-state lost --rate 5

Runtime keys (press in the terminal):
    t  → tracking
    l  → lost
    i  → invalid
    q  → quit
"""

from __future__ import annotations

import argparse
import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from rotor_swarm_msgs.msg import FrontFaceCorners

SOLUTION_INVALID = 0
SOLUTION_TRACKING = 1
SOLUTION_LOST = 2
CORNER_OBSERVED = 0
CORNER_INFERRED = 1
CORNER_INVALID = 2

_STATE_NAMES: dict[int, str] = {
    SOLUTION_INVALID: "invalid",
    SOLUTION_TRACKING: "tracking",
    SOLUTION_LOST: "lost",
}

# FLU body frame (x=forward, y=left, z=up).  Top edge user-computed.
# Bottom edge Z=0 ground plane — X/Y same as top (structure vertical).

_TRACKING: tuple[tuple[float, float, float], ...] = (
    (3.388,  0.286, 1.236),  # top_left
    (3.442, -0.765, 1.236),  # top_right
    (3.388,  0.286, 0.0),    # bottom_left
    (3.442, -0.765, 0.0),    # bottom_right
)

_LOST: tuple[tuple[float, float, float], ...] = (
    (3.400,  0.300, 1.250),  # top_left     (drifted)
    (3.460, -0.780, 1.250),  # top_right    (drifted)
    (3.400,  0.300, 0.0),    # bottom_left  (drifted)
    (3.460, -0.780, 0.0),    # bottom_right (drifted)
)

_INVALID: tuple[tuple[float, float, float], ...] = (
    (0.0, 0.0, 0.0),
    (0.0, 0.0, 0.0),
    (0.0, 0.0, 0.0),
    (0.0, 0.0, 0.0),
)

_SOLUTION_TO_CORNERS: dict[int, tuple[tuple[float, float, float], ...]] = {
    SOLUTION_INVALID: _INVALID,
    SOLUTION_TRACKING: _TRACKING,
    SOLUTION_LOST: _LOST,
}


def _parse_solution_state(raw: str) -> int:
    raw_lower = raw.strip().lower()
    named: dict[str, int] = {
        "invalid": SOLUTION_INVALID,
        "0": SOLUTION_INVALID,
        "tracking": SOLUTION_TRACKING,
        "1": SOLUTION_TRACKING,
        "lost": SOLUTION_LOST,
        "2": SOLUTION_LOST,
    }
    if raw_lower not in named:
        raise argparse.ArgumentTypeError(
            f"invalid solution_state '{raw}'. Use one of: invalid|0, tracking|1, lost|2"
        )
    return named[raw_lower]


def _corner_status(solution_state: int) -> int:
    if solution_state == SOLUTION_TRACKING:
        return CORNER_OBSERVED
    if solution_state == SOLUTION_LOST:
        return CORNER_INFERRED
    return CORNER_INVALID


class MockFrontFaceCornerPublisher(Node):

    def __init__(
        self,
        *,
        solution_state: int,
        rate_hz: float,
        frame_id: str,
        topic: str,
    ) -> None:
        super().__init__("mock_front_face_corner_publisher")
        self._solution_state = solution_state
        self._frame_id = frame_id
        self._corners = _SOLUTION_TO_CORNERS[solution_state]
        self._pub = self.create_publisher(FrontFaceCorners, topic, 10)
        self._timer = self.create_timer(1.0 / rate_hz, self._tick)
        self.get_logger().info(
            f"publishing to '{topic}' "
            f"solution_state={solution_state} "
            f"frame_id='{frame_id}' "
            f"rate={rate_hz} Hz"
        )
        self.get_logger().info("keys: t=tracking  l=lost  i=invalid  q=quit")

    def set_state(self, state: int) -> None:
        self._solution_state = state
        self._corners = _SOLUTION_TO_CORNERS[state]
        self.get_logger().info(f"solution_state -> {_STATE_NAMES[state]} ({state})")

    def _tick(self) -> None:
        msg = FrontFaceCorners()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id

        msg.solution_state = self._solution_state
        msg.valid = self._solution_state == SOLUTION_TRACKING
        msg.tracking_confidence = 0.85 if msg.valid else 0.0
        msg.top_source = "mock" if self._solution_state != SOLUTION_INVALID else "none"

        tl, tr, bl, br = self._corners
        msg.top_left.x, msg.top_left.y, msg.top_left.z = tl
        msg.top_right.x, msg.top_right.y, msg.top_right.z = tr
        msg.bottom_left.x, msg.bottom_left.y, msg.bottom_left.z = bl
        msg.bottom_right.x, msg.bottom_right.y, msg.bottom_right.z = br

        status = _corner_status(self._solution_state)
        msg.top_left_status = status
        msg.top_right_status = status
        msg.bottom_left_status = status
        msg.bottom_right_status = status

        self._pub.publish(msg)


_KEY_MAP: dict[str, int] = {
    "t": SOLUTION_TRACKING,
    "l": SOLUTION_LOST,
    "i": SOLUTION_INVALID,
}


def _read_key() -> str | None:
    if select.select([sys.stdin], [], [], 0.0)[0]:
        return sys.stdin.read(1)
    return None


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Publish synthetic FrontFaceCorners messages for debugging.",
    )
    parser.add_argument(
        "--solution-state",
        default="tracking",
        type=_parse_solution_state,
        help="One of: invalid|0, tracking|1, lost|2 (default: tracking)",
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=10.0,
        help="Publish frequency in Hz (default: 10.0)",
    )
    parser.add_argument(
        "--frame-id",
        default="body",
        help="frame_id for the message header (default: body)",
    )
    parser.add_argument(
        "--topic",
        default="/roi_lidar_corner/front_face_corners",
        help="Topic to publish on (default: /roi_lidar_corner/front_face_corners)",
    )
    args = parser.parse_args()

    rclpy.init(args=sys.argv)
    node = MockFrontFaceCornerPublisher(
        solution_state=args.solution_state,
        rate_hz=args.rate,
        frame_id=args.frame_id,
        topic=args.topic,
    )

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    tty.setcbreak(fd)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            key = _read_key()
            if key is None:
                continue
            if key == "q":
                break
            new_state = _KEY_MAP.get(key)
            if new_state is not None:
                node.set_state(new_state)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
