from as2_python_api.mission_interpreter.mission import Mission
from as2_python_api.mission_interpreter.mission_interpreter import MissionInterpreter

from std_msgs.msg import String

from rclpy import qos
import rclpy.executors

import rclpy
import sys

class MissionReciever:
    def __init__(self, drone_id, namespace):
        self.id = drone_id
        self.uav_name = namespace + str(self.id)
        self.mission = Mission(target=self.uav_name, verbose=False)
        self.interpreter = MissionInterpreter(mission=self.mission)
        self.landed = True

        mission_sub_name = str("/" + self.uav_name + "/mission")
        self.mission_sub = self.interpreter.drone.create_subscription(String, mission_sub_name, self.mission_callback, qos.QoSProfile(reliability=qos.ReliabilityPolicy.RELIABLE, depth=10))

    def mission_callback(self, msg : String):
        self.interpreter.drone.get_logger().info("New mission established")
        self.mission = Mission.parse_raw(msg.data)
        self.start_mission()

    def start_mission(self):
        self.interpreter.drone.get_logger().info("Start mission")
        self.interpreter.reset(self.mission)
        self.interpreter.drone.arm()
        self.interpreter.drone.offboard()
        self.interpreter.start_mission()

if __name__ == '__main__':
    rclpy.init()

    if(len(sys.argv) < 3):
        print("Usage: python3 mission_reciever.py id namespace")
        exit(-1)

    id = int(sys.argv[1])
    uav_namespace = str(sys.argv[2])

    MissionReciever(id, uav_namespace)