from rclpy.node import Node
import threading
from typing import List

from time import sleep
import rclpy
import rclpy.executors
from rclpy import qos
from mutac_msgs.srv import GeneratePlan
from mutac_msgs.msg import Alarm, Identifier
from std_msgs.msg import Int32

class Starter(Node):
    def __init__(self):
        super().__init__("PlanGetter")

        self.keep_running = True
        self.plan_cli = self.create_client(GeneratePlan, '/planner/generate_plan')

        self.land_pub = self.create_publisher(Int32, '/planner/signal/emergency_land', qos.QoSProfile(reliability=qos.ReliabilityPolicy.RELIABLE, depth=10))
        self.alarm_pub = self.create_publisher(Alarm, '/planner/signal/drone_alarm', qos.QoSProfile(reliability=qos.ReliabilityPolicy.RELIABLE, depth=10))

        self.__executor = rclpy.executors.SingleThreadedExecutor()
        self.__executor.add_node(self)

        self.t_spin_node = threading.Thread(target=self.spin_node)
        self.t_spin_node.start()

    
    def pub_plan(self, plan):
        while not self.plan_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        resp = self.plan_cli.call_async(plan)
        return resp

    def pub_land(self, drone):
        self.land_pub.publish(Int32(data=drone))

    def pub_repeat(self, drone, type):
        self.alarm_pub.publish(Alarm(identifier=Identifier(natural=drone), alarm=type))

    def spin_node(self):
        while self.keep_running and rclpy.ok():
            self.__executor.spin_once(timeout_sec=0)
            sleep(0.05)
    
    def shutdown(self):
        """Shutdown properly"""
        self.keep_running = False
        self.destroy_client(self.plan_cli)
