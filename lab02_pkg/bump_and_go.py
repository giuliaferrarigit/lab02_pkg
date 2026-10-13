import rclpy
import math
import random
import tf_transformations
from rclpy.node import Node 
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

#il robot ha due comportamenti cioè go forwaard cioè va dritto 
#fino a che non trova un muro e poi ruota 
#finisce quando arriva al cubo verde

#LiDAR è importante per trovare ostacoli 
#pose tramite odom
class bump_and_go(Node):

    
    GO_FORWARD = 'GO_FORWARD'
    ROTATE = 'ROTATE'
    APPROACH = 'APPROACH'
    GOAL_REACHED = 'GOAL_REACHED'

    def __init__(self):
        super().__init__('bump_and_go') 
        self.declare_parameter ('max_linear_velocity', 0.2)
        #ipotesi cautelare
        self.declare_parameter ('max_angular_velocity', 1.5) #rad/s
        self.declare_parameter ('control_frequency', 10.0)
        self.declare_parameter ('threshold', 0.4)
        self.declare_parameter ('x_goal', 6.5)
        self.declare_parameter ('y_goal', 3.0)
        self.declare_parameter('goal_tolerance', 0.4)    
        # a 0.4 m dal goal consideriamo di averlo raggiunto
        self.declare_parameter('front_cone_half', 30.0)
        #è il cono a +30 e -30 che gli sto dicendo di considerare in deg    

        self.declare_parameter('min_extra_rotation', 30.0)   
        self.declare_parameter('max_extra_rotation', 120.0)  
        #scritto così lo posso modificare dall'esterno 
        #con sel.vel=0.3, per provare 0.15 bisogna aprire il file,
        #modificarlo e rilanciare

        self.front_angle = math.radians(self.get_parameter('front_cone_half').value)

        # Lettura dei valori
        self.v_max = self.get_parameter('max_linear_velocity').value
        self.w_max = self.get_parameter('max_angular_velocity').value
        self.frequency = self.get_parameter('control_frequency').value
        self.threshold = self.get_parameter('threshold').value
        self.goal_x = self.get_parameter('x_goal').value
        self.goal_y = self.get_parameter('y_goal').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.min_extra = self.get_parameter('min_extra_rotation').value
        self.max_extra = self.get_parameter('max_extra_rotation').value
        #  invio i comandi di velocità al robot
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        #  ricevo LiDAR e odometria
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        

        # distanza per uscire da ROTATE: piu' grande di threshold (isteresi),
        # altrimenti con il rumore del LiDAR lo stato sfarfalla
        self.clear_distance = self.threshold + 0.1
        
        # Timer: chiama control_loop 'frequency' volte al secondo
        self.create_timer(1.0 / self.frequency, self.control_loop)

        # Internal state variables for tracking robot motion and position
        self.scan_data = None
        self.x = None               # pose from odometry (None = not received yet)
        self.y = None
        self.yaw = None
        self.target_yaw = None
        self.state = self.GO_FORWARD
        self.turn_direction = 1  # 1 = left, -1 = right
        
        
    def odom_callback(self, msg):

        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        quaternion=msg.pose.pose.orientation 
        quat = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        _, _, yaw = tf_transformations.euler_from_quaternion(quat)
        self.yaw = yaw
        #yaw rotation around vertical axis 

    def scan_callback(self, msg):
        self.scan_data = msg


    def sector_values(self, angle_from, angle_to):
        s = self.scan_data
        values = []

        for i, r in enumerate(s.ranges):
            angle = s.angle_min + i * s.angle_increment
            angle = math.atan2(math.sin(angle), math.cos(angle))  # -> [-pi, pi]
            #ricorda che atan riceve un solo valore numerico che è y/x
            #atan2 riceve due argoomenti separati ed è atan2(y,x) 
            if angle_from <= angle <= angle_to:
                # drop invalid readings: inf (nothing seen), nan, 0.0
                if math.isfinite(r) and s.range_min <= r <= s.range_max:
#true when the number è real e false or nan quando è finito 
#se la distanza è un numero reale e dentro i limiti fisici di lidar                    
                    values.append(r)
        return values 
        

    def sector_min(self, angle_from, angle_to):
        values = self.sector_values(angle_from, angle_to)
        return min(values) if values else float('inf')
 
    def sector_mean(self, angle_from, angle_to):
        values = self.sector_values(angle_from, angle_to)
        return sum(values) / len(values) if values else float('inf')

    def set_state(self, new_state):
        if new_state != self.state:
            self.get_logger().info(f'State changed: {self.state} -> {new_state}')
            self.state = new_state
        
    def rotation (self):
        left = self.sector_mean(self.front_angle, math.pi / 2)    # 30 deg .. 90 deg
        right = self.sector_mean(-math.pi / 2, -self.front_angle)  # -90 deg .. -30 deg
#così si muove a destra o a sinistra di 60 o -60 deg
        self.turn_direction = 1.0 if left > right else -1.0
        self.target_yaw = None
        self.set_state(self.ROTATE)

    def set_rotation_target(self):
        # il fronte si e' appena liberato: decido di quanto girare ANCORA
        extra = math.radians(random.uniform(self.min_extra, self.max_extra))
        self.target_yaw = math.atan2(
            math.sin(self.yaw + self.turn_direction * extra),
            math.cos(self.yaw + self.turn_direction * extra))
 
    def target_reached(self):
        # vero quando ho raggiunto (o appena superato) target_yaw,
        # misurato nel verso in cui sto girando
        if self.target_yaw is None:
            return False
        diff = self.target_yaw - self.yaw
        diff = math.atan2(math.sin(diff), math.cos(diff))   # -> [-pi, pi]
        return self.turn_direction * diff <= 0
 
    def publish(self, v, w):
        # v = velocita' avanti [m/s], w = velocita' di rotazione [rad/s]
        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(w)
        self.cmd_pub.publish(cmd)
        
    def control_loop(self):

#calcolare l'ostacolo più vicino in un determinato a
#cerchio di un certo raggio

        if self.scan_data is None:
            return
#wait until we have received the first scan
        if self.state == self.GOAL_REACHED:
            self.publish(0.0, 0.0)
            return
        distance_goal=dist_goal = math.hypot(self.goal_x - self.x, self.goal_y - self.y) 
        #hypot. calcola la norma euclidea di un vettore, in questo caso il vettore che va dalla posizione attuale alla posizione goal
        if distance_goal < self.goal_tolerance:
            self.set_state(self.GOAL_REACHED)
            self.get_logger().info(f'Goal reached at ({self.x:.2f}, {self.y:.2f})')
            self.publish(0.0, 0.0)  # stop the robot
            return

        front = self.sector_min(-self.front_angle, self.front_angle)
       
        if self.state == self.GO_FORWARD:
            
            if front < self.threshold:
                self.rotation()
                self.publish(0.0, self.turn_direction * self.w_max)
            else:
                self.publish(self.v_max, 0.0)
                
            #no forward motion and rotate at the maximum angular speed
            #se positivo turns left se negative turns right  

        elif self.state == self.ROTATE:
            clear = front > self.clear_distance
 
            # esco solo se la strada e' libera E ho finito l'extra rotation
            if clear and self.target_reached():
                self.set_state(self.GO_FORWARD)
                self.publish(self.v_max, 0.0)
            else:
                if clear and self.target_yaw is None:
                    self.set_rotation_target()
                self.publish(0.0, self.turn_direction * self.w_max)
            
    

def main():
    rclpy.init()
    node=bump_and_go()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
