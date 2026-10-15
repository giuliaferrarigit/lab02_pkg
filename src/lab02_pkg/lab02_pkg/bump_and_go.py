import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose, Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import tf_transformations
import numpy as np


class BumpAndGo(Node):
    """ROS2 node: navigazione ibrida spazio-libero + goal in linea di vista.

    - Se il goal NON è visibile (il raggio lidar verso il goal colpisce un muro prima):
      sterza verso il baricentro dello spazio libero (come nei corridoi).
    - Se il goal È visibile (raggio libero fino al goal): sterza verso il goal.
    - ROTATE: se c'è un ostacolo frontale ruota dal lato con più spazio.
    - Il robot si ferma quando l'odometria lo porta vicino al goal.
    """

    def __init__(self):
        super().__init__('bump_and_go_node')

        # Parametri base (angular_speed POSITIVO: il verso lo decide turn_direction)
        self.declare_parameter('linear_speed', 0.2)
        self.declare_parameter('angular_speed', 1.5)
        self.declare_parameter('control_frequency', 10.0)
        self.declare_parameter('min_distance_threshold', 0.5)
        self.declare_parameter('front_half_angle_deg', 15.0)
        self.declare_parameter('side_tie_margin', 0.15)  # m: sotto questa differenza i lati sono "pari"

        # Navigazione verso lo spazio libero
        self.declare_parameter('free_gain', 1.0)
        self.declare_parameter('free_half_angle_deg', 90.0)
        self.declare_parameter('free_range_cap', 2.0)
        self.declare_parameter('free_power', 2.0)
        self.declare_parameter('max_angular_cmd', 0.8)
        self.declare_parameter('wall_gain', 0.5)
        self.declare_parameter('wall_dist', 0.4)

        # Goal in linea di vista
        self.declare_parameter('goal_x', 6.5)
        self.declare_parameter('goal_y', 3.0)
        self.declare_parameter('goal_stop_radius', 0.4)
        self.declare_parameter('goal_gain', 0.8)
        self.declare_parameter('goal_visible_window_deg', 5.0)   # semi-ampiezza del raggio di controllo
        self.declare_parameter('goal_visible_margin', 0.35)      # m: tolleranza (il cubo stesso riflette il raggio)
        self.declare_parameter('goal_approach_dist', 1.0)        # m: sotto questa distanza (con goal visibile)...
        self.declare_parameter('goal_approach_threshold', 0.2)   # ...la soglia ostacolo frontale si abbassa

        # Frame di odom
        self.declare_parameter('odom_offset_x', 0.0)   # posizione di spawn nel frame del goal
        self.declare_parameter('odom_offset_y', 0.0)
        self.declare_parameter('odom_offset_yaw', 0.0)

        self.linear_speed = self.get_parameter('linear_speed').value
        # abs(): il verso lo decide SOLO turn_direction
        self.angular_speed = abs(self.get_parameter('angular_speed').value)
        self.control_frequency = self.get_parameter('control_frequency').value
        self.min_distance_threshold = self.get_parameter('min_distance_threshold').value
        self.front_half_angle = np.deg2rad(self.get_parameter('front_half_angle_deg').value)
        self.side_tie_margin = self.get_parameter('side_tie_margin').value

        self.free_gain = self.get_parameter('free_gain').value
        self.free_half_angle = np.deg2rad(self.get_parameter('free_half_angle_deg').value)
        self.free_range_cap = self.get_parameter('free_range_cap').value
        self.free_power = self.get_parameter('free_power').value
        self.max_angular_cmd = self.get_parameter('max_angular_cmd').value
        self.wall_gain = self.get_parameter('wall_gain').value
        self.wall_dist = self.get_parameter('wall_dist').value

        self.goal_pos = Pose()
        self.goal_pos.position.x = float(self.get_parameter('goal_x').value)
        self.goal_pos.position.y = float(self.get_parameter('goal_y').value)
        self.goal_stop_radius = self.get_parameter('goal_stop_radius').value
        self.goal_gain = self.get_parameter('goal_gain').value
        self.goal_visible_window = np.deg2rad(self.get_parameter('goal_visible_window_deg').value)
        self.goal_visible_margin = self.get_parameter('goal_visible_margin').value
        self.goal_approach_dist = self.get_parameter('goal_approach_dist').value
        self.goal_approach_threshold = self.get_parameter('goal_approach_threshold').value

        self.off_x = self.get_parameter('odom_offset_x').value
        self.off_y = self.get_parameter('odom_offset_y').value
        self.off_yaw = self.get_parameter('odom_offset_yaw').value

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.timer = self.create_timer(1.0 / self.control_frequency, self.control_loop)

        self.scan_data = None
        self.current_yaw = 0.0
        self.state = 'GO_FORWARD'
        self.turn_direction = 1  # 1 = sinistra, -1 = destra
        self.position = Point()
        self.goal_was_visible = False

    # ------------------------------------------------------------------ callbacks
    def odom_callback(self, msg):
        """Porta pose e yaw dal frame odom al frame del goal (offset di spawn)."""
        quat = msg.pose.pose.orientation
        _, _, yaw = tf_transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        p = msg.pose.pose.position
        c, s = np.cos(self.off_yaw), np.sin(self.off_yaw)
        self.position = Point(x=float(self.off_x + c * p.x - s * p.y),
                              y=float(self.off_y + s * p.x + c * p.y),
                              z=0.0)
        self.current_yaw = self.normalize_angle(yaw + self.off_yaw)

    def laser_callback(self, msg):
        self.scan_data = msg

    # ------------------------------------------------------------------ utilità
    def normalize_angle(self, angle):
        return float(np.arctan2(np.sin(angle), np.cos(angle)))

    def scan_arrays(self):
        """Ritorna (distanze, angoli normalizzati in [-pi, pi]); inf/nan -> range_max."""
        scan = self.scan_data
        r = np.array(scan.ranges, dtype=float)
        max_r = scan.range_max if np.isfinite(scan.range_max) else 3.5
        r = np.where(np.isfinite(r) & (r > scan.range_min), r, max_r)
        r = np.minimum(r, max_r)
        angles = scan.angle_min + np.arange(len(r)) * scan.angle_increment
        angles = np.arctan2(np.sin(angles), np.cos(angles))
        return r, angles

    def front_min_distance(self):
        """Distanza minima nel cono frontale, basata sugli ANGOLI (non sugli indici)."""
        r, angles = self.scan_arrays()
        mask = np.abs(angles) <= self.front_half_angle
        return float(np.min(r[mask])) if np.any(mask) else float('inf')

    def sector_mean(self, deg_start, deg_end):
        """Distanza media in un settore angolare (positivo = sinistra)."""
        r, angles = self.scan_arrays()
        mask = (angles >= np.deg2rad(deg_start)) & (angles <= np.deg2rad(deg_end))
        return float(np.mean(r[mask])) if np.any(mask) else 0.0

    def goal_angle_error(self):
        """Errore angolare verso il goal: >0 il goal è a sinistra, <0 a destra."""
        dx = self.goal_pos.position.x - self.position.x
        dy = self.goal_pos.position.y - self.position.y
        return self.normalize_angle(np.arctan2(dy, dx) - self.current_yaw)

    def distance_to_goal(self):
        return float(np.hypot(
            self.position.x - self.goal_pos.position.x,
            self.position.y - self.goal_pos.position.y))

    def goal_visible(self, dist):
        """True se il raggio lidar verso il goal è libero fino (quasi) al goal.

        Un muro in mezzo fa leggere al raggio una distanza minore di quella del goal,
        quindi il goal risulta NON visibile. Il margine tiene conto del fatto che il cubo
        stesso riflette il raggio poco prima del suo centro.
        """
        r, angles = self.scan_arrays()
        max_r = self.scan_data.range_max if np.isfinite(self.scan_data.range_max) else 3.5
        bearing = self.goal_angle_error()
        diff = np.abs(np.arctan2(np.sin(angles - bearing), np.cos(angles - bearing)))
        mask = diff <= self.goal_visible_window
        if not np.any(mask):
            return False  # goal fuori dal campo del lidar
        needed = min(dist - self.goal_visible_margin, max_r - 0.05)
        if needed <= 0.0:
            return True
        return float(np.min(r[mask])) >= needed

    def free_space_heading(self):
        """Direzione (rad, >0 = sinistra) del baricentro dello spazio libero davanti.

        Media degli angoli pesata con distanza^potenza: nessun bias laterale.
        """
        r, angles = self.scan_arrays()
        mask = np.abs(angles) <= self.free_half_angle
        if not np.any(mask):
            return 0.0
        w = np.minimum(r[mask], self.free_range_cap) ** self.free_power
        total = float(np.sum(w))
        if total <= 1e-9:
            return 0.0
        return float(np.sum(w * angles[mask]) / total)

    def wall_avoidance(self):
        """Repulsione laterale simmetrica.
        >0 = gira a sinistra (muro a destra), <0 = gira a destra (muro a sinistra)."""
        r, angles = self.scan_arrays()
        left = r[(angles > np.deg2rad(20)) & (angles < np.deg2rad(80))]
        right = r[(angles < -np.deg2rad(20)) & (angles > -np.deg2rad(80))]
        dl = float(np.min(left)) if left.size else self.wall_dist
        dr = float(np.min(right)) if right.size else self.wall_dist
        push_l = max(0.0, self.wall_dist - dl)  # muro a sinistra -> spinge a destra
        push_r = max(0.0, self.wall_dist - dr)  # muro a destra -> spinge a sinistra
        return self.wall_gain * (push_r - push_l)

    def choose_turn_direction(self):
        """1 = sinistra, -1 = destra.

        1) Se un lato ha chiaramente più spazio, gira da quel lato.
        2) Se i lati sono simili, alterna rispetto all'ultima volta.
        """
        left = self.sector_mean(20, 90)
        right = self.sector_mean(-90, -20)
        diff = left - right

        if abs(diff) > self.side_tie_margin:
            direction = 1 if diff > 0 else -1
            reason = 'più spazio'
        else:
            direction = -self.turn_direction
            reason = 'alternanza'

        self.get_logger().info(
            f'Left: {left:.2f} m, Right: {right:.2f} m -> '
            f'{"SINISTRA" if direction == 1 else "DESTRA"} ({reason})')
        return direction

    # ------------------------------------------------------------------ controllo
    def control_loop(self):
        if self.scan_data is None:
            return

        twist = Twist()

        if self.state == 'STOP':
            self.cmd_vel_pub.publish(twist)
            return

        dist = self.distance_to_goal()
        if dist <= self.goal_stop_radius:
            self.get_logger().info('At goal position! Stopping robot.')
            self.state = 'STOP'
            self.cmd_vel_pub.publish(twist)
            return

        visible = self.goal_visible(dist)
        if visible != self.goal_was_visible:
            self.get_logger().info('Goal visibile: vado verso il goal' if visible
                                   else 'Goal non visibile: seguo lo spazio libero')
            self.goal_was_visible = visible

        # Con il goal visibile e vicino, il cubo stesso è un "ostacolo": abbassa la soglia
        if visible and dist < self.goal_approach_dist:
            threshold = self.goal_approach_threshold
        else:
            threshold = self.min_distance_threshold

        min_front = self.front_min_distance()

        if self.state == 'GO_FORWARD':
            if min_front < threshold:
                self.get_logger().info('Obstacle detected! Switching to ROTATE')
                self.state = 'ROTATE'
                self.turn_direction = self.choose_turn_direction()
            else:
                if visible:
                    heading = self.goal_angle_error()
                    cmd = self.goal_gain * heading + self.wall_avoidance()
                else:
                    heading = self.free_space_heading()
                    cmd = self.free_gain * heading + self.wall_avoidance()
                twist.angular.z = float(np.clip(cmd, -self.max_angular_cmd, self.max_angular_cmd))
                # Rallenta quando deve sterzare molto
                twist.linear.x = self.linear_speed if abs(heading) < np.deg2rad(30) else self.linear_speed * 0.4

        elif self.state == 'ROTATE':
            # Il verso resta "bloccato" per tutta la rotazione (evita di cambiare idea a metà)
            if min_front > threshold + 0.1:
                self.get_logger().info('Path clear. Switching to GO_FORWARD')
                self.state = 'GO_FORWARD'
            else:
                twist.angular.z = self.turn_direction * self.angular_speed
                self.get_logger().info(
                    f'ROTATE: turn_direction={self.turn_direction}, '
                    f'angular.z={twist.angular.z:+.2f} (positivo = sinistra)',
                    throttle_duration_sec=1.0)

        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = BumpAndGo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_vel_pub.publish(Twist())  # ferma il robot all'uscita
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()