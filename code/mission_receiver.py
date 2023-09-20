from as2_python_api.mission_interpreter.mission import Mission
from as2_python_api.mission_interpreter.mission_interpreter import MissionInterpreter

from as2_msgs.msg import MissionUpdate

from rclpy import qos
import rclpy.executors

import rclpy
import sys

class MissionReceiver:
    def __init__(self, drone_id, namespace):
        self.id = drone_id
        self.uav_name = namespace + str(self.id)
        self.mission = Mission(target=self.uav_name, verbose=False)
        self.interpreter = MissionInterpreter(mission=self.mission)
        self.landed = True

        # mission_sub_name = str("/" + self.uav_name + "/mission")
        mission_sub_name = str("/mission_update")
        self.mission_sub = self.interpreter.drone.create_subscription(MissionUpdate, mission_sub_name, self.mission_callback, qos.QoSProfile(reliability=qos.ReliabilityPolicy.RELIABLE, depth=10))

    def mission_callback(self, msg : MissionUpdate):
        self.interpreter.drone.get_logger().info("Message received")
        if msg.drone_id == str(self.id):
            self.interpreter.drone.get_logger().info("This drone is the target")
            if msg.action == MissionUpdate.EXECUTE:
                self.execute_mission(Mission.parse_raw(msg.mission))
                self.interpreter.drone.get_logger().info("New mission established")

            if msg.action == MissionUpdate.STOP:
                self.stop_mission()
                self.interpreter.drone.get_logger().info("Mission stopped")

            # if msg.action == MissionUpdate.APPEND:
            #     self.append_mission(Mission.parse_raw(msg.mission))
            #     self.interpreter.drone.get_logger().info("Mission appended")
            
            if msg.action == MissionUpdate.INSERT:
                self.insert_mission(Mission.parse_raw(msg.mission))
                self.interpreter.drone.get_logger().info("Mission inserted")

            if msg.action == MissionUpdate.PAUSE:
                self.pause_mission()
                self.interpreter.drone.get_logger().info("Mission paused")

            if msg.action == MissionUpdate.RESUME:
                self.resume_mission()
                self.interpreter.drone.get_logger().info("Mission resumed")


    def execute_mission(self, mission : Mission):
        self.interpreter.reset(mission)
        self.interpreter.drone.arm()
        self.interpreter.drone.offboard()
        self.interpreter.start_mission()

    def stop_mission(self):
        self.interpreter.stop_mission()

    def append_mission(self, mission : Mission):
        self.interpreter.append_mission(mission)

    def insert_mission(self, mission : Mission):
        self.interpreter.insert_mission(mission)

    def pause_mission(self):
        self.interpreter.pause_mission()

    def resume_mission(self):
        self.interpreter.resume_mission()

if __name__ == '__main__':
    rclpy.init()

    if(len(sys.argv) < 3):
        print("Usage: python3 mission_reciever.py id namespace")
        exit(-1)

    id = int(sys.argv[1])
    uav_namespace = str(sys.argv[2])

    MissionReceiver(id, uav_namespace)