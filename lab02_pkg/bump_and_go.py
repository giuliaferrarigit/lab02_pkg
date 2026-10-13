import rclpy
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

    def __init__(self):
        super().__init__('bump_and_go') 
        self.declare_parameter ('max_linear_velocity', 0.2)
        #ipotesi cautelare
        self.declare_parameter ('max_angular_velocity', 1.5) #rad/s
        self.declare_parameter ('control_frequency', 10.0)
        self.declare_parameter ('threshold', 0.4)
        self.declare_parameter ('x_goal', 6.5)
        self.declare_parameter ('y_goal', 3.0)
        self.declare_parameter ('z_goal', 0.5)


        #scritto così lo posso modificare dall'esterno 
        #con sel.vel=0.3, per provare 0.15 bisogna aprire il file,
        #modificarlo e rilanciare


        # Lettura dei valori
        self.v_max = self.get_parameter('max_linear_velocity').value
        self.w_max = self.get_parameter('max_angular_velocity').value
        self.frequency = self.get_parameter('control_frequency').value
        self.threshold = self.get_parameter('threshold').value
        
        #  invio i comandi di velocità al robot
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        #  ricevo LiDAR e odometria
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Timer: chiama control_loop 'frequency' volte al secondo
        self.create_timer(1.0 / self.frequency, self.control_loop)

        # Internal state variables for tracking robot motion and position
        self.scan_data = None
        self.current_yaw = 0.0
        self.state = 'GO_FORWARD'
        self.turn_direction = 1  # 1 = left, -1 = right
        self.position = Point()
        
        # Define goal position that robot should navigate to
        self.goal_pos = Pose()
        self.goal_pos.position.x = 6.5
        self.goal_pos.position.y = 3.0



        quat = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
_, _, yaw = tf_transformations.euler_from_quaternion(quat)
        
def main():
    


if __name__ == '__main__':
    main()
