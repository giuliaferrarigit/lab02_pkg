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
        self.x_goal=self..get_parameter('x_goal').value
        self.y_goal=self.get_parameter('y_goal').value
        self.z_goal=self.get_parameter('z_goal').value
        #  invio i comandi di velocità al robot
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        #  ricevo LiDAR e odometria
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Timer: chiama control_loop 'frequency' volte al secondo
        self.create_timer(1.0 / self.frequency, self.control_loop)




        quat = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
_, _, yaw = tf_transformations.euler_from_quaternion(quat)
        
def main():
    


if __name__ == '__main__':
    main()
