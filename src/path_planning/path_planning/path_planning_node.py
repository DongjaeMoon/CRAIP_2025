#!/usr/bin/env python3

import math
import heapq
import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import OccupancyGrid, Path


class PathPlanningNode(Node):
    def __init__(self):
        super().__init__('path_planning_node')

        # 맵 정보
        self.map_msg: OccupancyGrid = None
        self.map_grid = None          # 2차원 numpy 배열 (height, width)
        self.resolution = None
        self.origin_x = None
        self.origin_y = None
        self.width = None
        self.height = None

        # 포즈 정보
        self.start_pose: PoseStamped = None   # /go1_pose
        self.goal_pose: PoseStamped = None    # /goal_pose

        # 로봇 크기 (대략) - 맵에서 margin 줄 때 사용
        self.robot_radius = 0.3  # meter

        # 구독
        self.create_subscription(
            PoseStamped,
            '/go1_pose',
            self.start_callback,
            10
        )

        self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )

        self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        # 경로 발행
        self.path_pub = self.create_publisher(
            Path,
            '/local_path',
            10
        )

        self.get_logger().info('PathPlanningNode 초기화 완료 (/go1_pose, /goal_pose, /map 사용, /local_path 발행)')

    # ==========================
    # 콜백
    # ==========================

    def start_callback(self, msg: PoseStamped):
        self.start_pose = msg
        self.try_plan()

    def goal_callback(self, msg: PoseStamped):
        self.goal_pose = msg
        self.try_plan()

    def map_callback(self, msg: OccupancyGrid):
        self.map_msg = msg
        self.width = msg.info.width
        self.height = msg.info.height
        self.resolution = msg.info.resolution
        self.origin_x = msg.info.origin.position.x
        self.origin_y = msg.info.origin.position.y

        data = np.array(msg.data, dtype=np.int16).reshape((self.height, self.width))
        self.map_grid = data

        # 로봇 크기를 고려한 장애물 팽창 (margin_cells)
        margin_cells = max(1, int(self.robot_radius / self.resolution))
        self.inflate_obstacles(margin_cells=margin_cells)

        self.get_logger().info(
            f'맵 수신 및 팽창 완료 (width={self.width}, height={self.height}, resolution={self.resolution:.3f}, margin_cells={margin_cells})'
        )

    # ==========================
    # 맵 관련 함수
    # ==========================

    def inflate_obstacles(self, margin_cells: int):
        if self.map_grid is None:
            return

        grid = self.map_grid
        inflated = grid.copy()

        obstacle_indices = np.argwhere(grid > 50)  # 50 이상을 장애물로 간주

        for (r, c) in obstacle_indices:
            r_min = max(0, r - margin_cells)
            r_max = min(self.height - 1, r + margin_cells)
            c_min = max(0, c - margin_cells)
            c_max = min(self.width - 1, c + margin_cells)
            inflated[r_min:r_max + 1, c_min:c_max + 1] = 100

        self.map_grid = inflated

    def world_to_grid(self, x: float, y: float):
        if self.resolution is None:
            return None

        gx = int(round((x - self.origin_x) / self.resolution))
        gy = int(round((y - self.origin_y) / self.resolution))

        if 0 <= gx < self.width and 0 <= gy < self.height:
            # numpy 인덱스는 [row, col] = [y, x]
            return gy, gx
        else:
            return None

    def grid_to_world(self, r: int, c: int):
        x = c * self.resolution + self.origin_x
        y = r * self.resolution + self.origin_y
        return x, y

    # ==========================
    # 플래닝 트리거
    # ==========================

    def try_plan(self):
        if self.map_grid is None:
            return
        if self.start_pose is None or self.goal_pose is None:
            return

        self.get_logger().info('경로 계획 시도 중...')

        start_rc = self.world_to_grid(
            self.start_pose.pose.position.x,
            self.start_pose.pose.position.y
        )
        goal_rc = self.world_to_grid(
            self.goal_pose.pose.position.x,
            self.goal_pose.pose.position.y
        )

        if start_rc is None or goal_rc is None:
            self.get_logger().warn('시작 또는 목표가 맵 밖입니다.')
            return

        # 1단계: A* 로 grid 상에서 경로 찾기
        path_rc = self.a_star(start_rc, goal_rc)

        if path_rc is None or len(path_rc) == 0:
            self.get_logger().warn('A* 경로를 찾지 못했습니다.')
            return

        # grid 경로를 world 좌표 polyline 으로 변환
        world_points = [self.grid_to_world(r, c) for (r, c) in path_rc]

        # raw path (A* 결과)를 Path 메시지로 만들어둘 것 (fallback 용)
        raw_path_msg = Path()
        raw_path_msg.header.frame_id = 'map'
        raw_path_msg.header.stamp = self.get_clock().now().to_msg()
        for (x, y) in world_points:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = raw_path_msg.header.stamp
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            raw_path_msg.poses.append(pose)

        # 2단계: move_go1 스타일 spline 으로 부드러운 path 생성
        # (경로 전체 길이 기준으로 포인트 개수 조절)
        total_dist = 0.0
        for i in range(len(world_points) - 1):
            dx = world_points[i + 1][0] - world_points[i][0]
            dy = world_points[i + 1][1] - world_points[i][1]
            total_dist += math.hypot(dx, dy)

        # 너무 작으면 기본값
        if total_dist < 0.1:
            total_dist = 0.1

        # move_go1 과 비슷한 기준: PATH_DENSITY = 20
        PATH_DENSITY = 20.0
        MIN_POINTS = 60
        MAX_POINTS = 200

        num_points = int(total_dist * PATH_DENSITY)
        num_points = max(MIN_POINTS, min(MAX_POINTS, num_points))

        smooth_path_msg = self.generate_smooth_path_from_poses(
            self.start_pose,
            self.goal_pose,
            num_points=num_points
        )

        # 3단계: spline 경로가 충돌 없는지 확인
        if self.is_path_collision_free(smooth_path_msg):
            self.path_pub.publish(smooth_path_msg)
            self.get_logger().info(
                f'부드러운 경로 발행 완료 (points={len(smooth_path_msg.poses)}, length={total_dist:.2f} m)'
            )
        else:
            # spline 이 장애물을 통과하면 raw A* 경로를 사용
            self.path_pub.publish(raw_path_msg)
            self.get_logger().warn(
                'spline 경로가 장애물과 충돌하여, A* raw 경로를 대신 사용합니다.'
            )

    # ==========================
    # A* 알고리즘
    # ==========================

    def a_star(self, start_rc, goal_rc):
        sr, sc = start_rc
        gr, gc = goal_rc

        if self.map_grid[sr, sc] > 50 or self.map_grid[gr, gc] > 50:
            self.get_logger().warn('시작 또는 목표가 장애물 위에 있습니다.')
            return None

        open_heap = []
        heapq.heappush(open_heap, (0.0, (sr, sc)))

        came_from = {}
        g_score = {(sr, sc): 0.0}
        closed = set()

        # 8방향 이웃
        neighbors = [
            (-1, 0), (1, 0),
            (0, -1), (0, 1),
            (-1, -1), (-1, 1),
            (1, -1), (1, 1),
        ]

        def heuristic(r, c):
            return math.hypot(r - gr, c - gc)

        while open_heap:
            _, current = heapq.heappop(open_heap)
            if current in closed:
                continue
            closed.add(current)

            if current == (gr, gc):
                # 경로 복원
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                path.reverse()
                return path

            cr, cc = current
            for dr, dc in neighbors:
                nr, nc = cr + dr, cc + dc

                if not (0 <= nr < self.height and 0 <= nc < self.width):
                    continue
                # 장애물 또는 unknown(-1)은 통과 금지
                if self.map_grid[nr, nc] > 50 or self.map_grid[nr, nc] < 0:
                    continue

                step_cost = math.hypot(dr, dc)
                tentative_g = g_score[current] + step_cost

                if (nr, nc) not in g_score or tentative_g < g_score[(nr, nc)]:
                    g_score[(nr, nc)] = tentative_g
                    f = tentative_g + heuristic(nr, nc)
                    heapq.heappush(open_heap, (f, (nr, nc)))
                    came_from[(nr, nc)] = current

        return None

    # ==========================
    # move_go1 스타일 spline 생성
    # ==========================

    def quaternion_to_yaw(self, q: Quaternion) -> float:
        """
        move_go1.py 에서 사용하던 yaw 추출 함수
        """
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def yaw_to_quaternion(self, yaw: float) -> Quaternion:
        """
        move_go1.py 에서 사용하던 yaw -> quaternion 변환
        """
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q

    def generate_smooth_path_from_poses(self,
                                        start_pose: PoseStamped,
                                        goal_pose: PoseStamped,
                                        num_points: int) -> Path:
        """
        move_go1.py 의 generate_smooth_path 를
        start_pose, goal_pose 를 직접 받아서 쓰는 버전.
        """
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = self.get_clock().now().to_msg()

        # Start
        x0 = start_pose.pose.position.x
        y0 = start_pose.pose.position.y
        z0 = start_pose.pose.position.z
        yaw0 = self.quaternion_to_yaw(start_pose.pose.orientation)

        # Goal
        x1 = goal_pose.pose.position.x
        y1 = goal_pose.pose.position.y
        z1 = goal_pose.pose.position.z
        yaw1 = self.quaternion_to_yaw(goal_pose.pose.orientation)

        # 거리 기반 tangent 스케일
        distance = math.sqrt((x1 - x0) ** 2 + (y1 - y0) ** 2)
        control_scale = min(distance * 0.5, 2.0)

        cx0 = x0 + control_scale * math.cos(yaw0)
        cy0 = y0 + control_scale * math.sin(yaw0)

        cx1 = x1 - control_scale * math.cos(yaw1)
        cy1 = y1 - control_scale * math.sin(yaw1)

        for i in range(num_points + 1):
            t = i / num_points

            # Hermite basis
            h00 = 2 * t ** 3 - 3 * t ** 2 + 1
            h10 = t ** 3 - 2 * t ** 2 + t
            h01 = -2 * t ** 3 + 3 * t ** 2
            h11 = t ** 3 - t ** 2

            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "map"
            pose_stamped.header.stamp = path.header.stamp

            # tangent vectors
            tangent_x0 = cx0 - x0
            tangent_y0 = cy0 - y0
            tangent_x1 = x1 - cx1
            tangent_y1 = y1 - cy1

            # Position
            px = (h00 * x0 + h10 * tangent_x0 +
                  h01 * x1 + h11 * tangent_x1)
            py = (h00 * y0 + h10 * tangent_y0 +
                  h01 * y1 + h11 * tangent_y1)
            pz = z0 + t * (z1 - z0)

            pose_stamped.pose.position.x = px
            pose_stamped.pose.position.y = py
            pose_stamped.pose.position.z = pz

            # Orientation from tangent
            if i < num_points:
                t_next = (i + 1) / num_points
                h00n = 2 * t_next ** 3 - 3 * t_next ** 2 + 1
                h10n = t_next ** 3 - 2 * t_next ** 2 + t_next
                h01n = -2 * t_next ** 3 + 3 * t_next ** 2
                h11n = t_next ** 3 - t_next ** 2

                x_next = (h00n * x0 + h10n * tangent_x0 +
                          h01n * x1 + h11n * tangent_x1)
                y_next = (h00n * y0 + h10n * tangent_y0 +
                          h01n * y1 + h11n * tangent_y1)

                dx = x_next - px
                dy = y_next - py
                yaw = math.atan2(dy, dx)
            else:
                yaw = yaw1

            pose_stamped.pose.orientation = self.yaw_to_quaternion(yaw)
            path.poses.append(pose_stamped)

        return path

    # ==========================
    # 경로 충돌 체크
    # ==========================

    def is_path_collision_free(self, path_msg: Path) -> bool:
        if self.map_grid is None:
            return True

        for pose in path_msg.poses:
            x = pose.pose.position.x
            y = pose.pose.position.y
            rc = self.world_to_grid(x, y)
            if rc is None:
                # 맵 밖으로 나가면 충돌로 간주
                return False
            r, c = rc
            val = self.map_grid[r, c]
            if val > 50 or val < 0:
                # 장애물 또는 unknown
                return False
        return True


def main(args=None):
    rclpy.init(args=args)
    node = PathPlanningNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

