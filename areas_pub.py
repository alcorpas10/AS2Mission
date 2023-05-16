from rclpy.node import Node
import threading

from time import sleep
import rclpy
import rclpy.executors
from rclpy import qos
from geometry_msgs.msg import Polygon, PolygonStamped, Point32
from std_msgs.msg import Header

class Areas(Node):
    def __init__(self):
        super().__init__("AreaPublisher")

        self.keep_running = True
        self.poly1_pub = self.create_publisher(PolygonStamped, '/planner/area1', qos.QoSProfile(reliability=qos.ReliabilityPolicy.RELIABLE, depth=10))
        self.poly2_pub = self.create_publisher(PolygonStamped, '/planner/area2', qos.QoSProfile(reliability=qos.ReliabilityPolicy.RELIABLE, depth=10))

        self.height = 2.0
        self.pub_polys = self.create_timer(5.0, self.pub_poly)

        self.__executor = rclpy.executors.SingleThreadedExecutor()
        self.__executor.add_node(self)

        self.t_spin_node = threading.Thread(target=self.spin_node)
        self.t_spin_node.start()

    
    def pub_poly(self):
        polygon1 = Polygon(points=[
            Point32(x=-3.25, y= 3.5 , z=self.height),
            Point32(x=-1.25, y= 2.25, z=self.height),
            Point32(x=-1.25, y=-1.25, z=self.height),
            Point32(x=-3.25, y=-1.25, z=self.height)
        ])
        polygon2 = Polygon(points=[
            Point32(x=-1.0 , y= 3.25, z=self.height),
            Point32(x= 1.25, y= 3.25, z=self.height),
            Point32(x= 1.25, y=-0.5 , z=self.height),
            Point32(x=-1.0 , y= 0.75, z=self.height)
        ])

        # polygon1 = Polygon(points=[
        #     Point32(x=-3.0 , y= 3.0, z=self.height),
        #     Point32(x=-1.5 , y= 2.0, z=self.height),
        #     Point32(x=-1.5 , y=-1.0, z=self.height),
        #     Point32(x=-3.0 , y=-1.0, z=self.height)
        # ])
        # polygon2 = Polygon(points=[
        #     Point32(x=-0.75, y= 3.0, z=self.height),
        #     Point32(x= 1.0 , y= 3.0, z=self.height),
        #     Point32(x= 1.0 , y= 0.0, z=self.height),
        #     Point32(x=-0.75, y= 1.0, z=self.height)
        # ])

        self.poly1_pub.publish(PolygonStamped(header=Header(frame_id='earth'),
                                             polygon=polygon1))
        self.poly2_pub.publish(PolygonStamped(header=Header(frame_id='earth'),
                                             polygon=polygon2))
        self.get_logger().info("Areas published")

    def spin_node(self):
        while self.keep_running and rclpy.ok():
            self.__executor.spin_once(timeout_sec=0)
            sleep(0.05)
    
    def shutdown(self):
        """Shutdown properly"""
        self.keep_running = False

if __name__ == '__main__':
    rclpy.init()

    pub = Areas()
