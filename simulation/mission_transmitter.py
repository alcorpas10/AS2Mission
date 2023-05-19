from as2_python_api.mission_interpreter.mission import Mission, MissionItem
from as2_msgs.msg import YawMode

from std_msgs.msg import Int32
from as2_msgs.msg import MissionUpdate
from mutac_msgs.msg import Plan, State

from rclpy import qos
import rclpy.executors

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher

import sys
import threading
from time import sleep

from typing import List, Dict

class MissionTransmitter(Node):
    def __init__(self, n_drones, namespace):
        super().__init__("MissionTransmitter")
        self.keep_running = True

        self.n_drones = n_drones
        self.namespace = namespace

        self.mission_id = 0

        self.drones_available = list(i for i in range(n_drones))

        self.mission_pubs : Dict[int, Publisher] = {}

        self.land_sub = self.create_subscription(Int32, "/planner/signal/emergency_land", self.land_callback, qos.QoSProfile(reliability=qos.ReliabilityPolicy.RELIABLE, depth=10))
        self.plan_sub = self.create_subscription(Plan, "/planner/planned_paths", self.path_callback, qos.QoSProfile(reliability=qos.ReliabilityPolicy.RELIABLE, depth=10))
        self.events_sub = self.create_subscription(State, "/planner/notification/drone_events", self.event_callback, qos.QoSProfile(reliability=qos.ReliabilityPolicy.RELIABLE, depth=10))

        for i in range(self.n_drones):
            mission_pub_name = str("/" + self.namespace + str(i) + "/mission")
            self.mission_pubs[i] = self.create_publisher(MissionUpdate, mission_pub_name, qos.QoSProfile(reliability=qos.ReliabilityPolicy.RELIABLE, depth=10))
        
        self.__executor = rclpy.executors.SingleThreadedExecutor()
        self.__executor.add_node(self)

        self.t_spin_node = threading.Thread(target=self.spin_node)
        self.t_spin_node.start()

    def land_callback(self, msg : Int32):
        print('LAND')
        target = self.namespace + str(msg.data)
        mission = Mission(target=target, verbose=False)
        mission.plan.append(MissionItem(behavior='go_to', args={
            '_x': 0.0, '_y': 0.0, '_z': 1.0,
            'speed': 0.5,
            'yaw_mode': YawMode.FIXED_YAW, 'yaw_angle': 0.00,
            'wait': True
        }))
        json_msg = mission.json()
        self.mission_pubs[msg.data].publish(MissionUpdate(drone_id=msg.data, mission_id=self.mission_id, type=MissionUpdate.EXECUTE, mission=json_msg))
        self.mission_id+=1

    def event_callback(self, msg : State):
        print('DRONE EVENT')
        drone_target = int(msg.identifier.natural)
        if msg.state == State.LOST:
            if drone_target in self.drones_available:
                self.drones_available.remove(drone_target)
            print(self.drones_available)
            if msg.type == State.LAND:
                target = self.namespace + str(drone_target)
                mission = Mission(target=target, verbose=False)
                mission.plan.append(MissionItem(behavior='land', args={
                    'speed': 0.5,
                    'wait': True
                }))
                json_msg = mission.json()
                self.mission_pubs[drone_target].publish(MissionUpdate(drone_id=drone_target, mission_id=self.mission_id, type=MissionUpdate.EXECUTE, mission=json_msg))
                for d in self.drones_available:
                    target = self.namespace + str(d)
                    mission = Mission(target=target, verbose=False)
                    json_msg = mission.json()
                    self.mission_pubs[d].publish(MissionUpdate(drone_id=d, mission_id=self.mission_id, type=MissionUpdate.EXECUTE, mission=json_msg))
                self.mission_id+=1
            if msg.type == State.HOMEBASE:
                target = self.namespace + str(drone_target)
                mission = Mission(target=target, verbose=False)
                mission.plan.append(MissionItem(behavior='go_to', args={
                    '_x': msg.position.x, '_y': msg.position.y, '_z': msg.position.z,
                    'speed': 0.5,
                    'yaw_mode': YawMode.FIXED_YAW, 'yaw_angle': 0.00,
                    'wait': True
                }))
                mission.plan.append(MissionItem(behavior='land', args={
                    'speed': 0.5,
                    'wait': True
                }))
                json_msg = mission.json()
                self.mission_pubs[drone_target].publish(MissionUpdate(drone_id=drone_target, mission_id=self.mission_id, type=MissionUpdate.EXECUTE, mission=json_msg))
                for d in self.drones_available:
                    target = self.namespace + str(d)
                    mission = Mission(target=target, verbose=False)
                    json_msg = mission.json()
                    self.mission_pubs[d].publish(MissionUpdate(drone_id=d, mission_id=self.mission_id, type=MissionUpdate.EXECUTE, mission=json_msg))
                self.mission_id+=1
        if msg.state == State.WP_REPEATED:
            target = self.namespace + str(drone_target)
            mission = Mission(target=target, verbose=False)
            mission.plan.append(MissionItem(behavior='go_to', args={
                '_x': msg.position.x, '_y': msg.position.y, '_z': msg.position.z,
                'speed': 0.5,
                'yaw_mode': YawMode.FIXED_YAW, 'yaw_angle': 0.00,
                'wait': True
            }))
            json_msg = mission.json()
            self.mission_pubs[drone_target].publish(MissionUpdate(drone_id=drone_target, mission_id=self.mission_id, type=MissionUpdate.INSERT, mission=json_msg))
        if msg.state == State.RECOVERED:
            self.drones_available.append(drone_target)
            print(self.drones_available)
        




    def path_callback(self, msg : Plan):
        self.get_logger().info("Got plan")
        missions : List[Mission] = list()
        for d in range(n_drones):
            target = self.namespace + str(d)
            missions.append(Mission(target=target, verbose=False))
            missions[d].plan.append(MissionItem(behavior='takeoff', args={
                'height': 1.0,
                'speed': 0.5,
                'wait': True
            }))

        for p in msg.paths:
            n = p.identifier.natural
            for point in p.points[1:]:
                missions[n].plan.append(MissionItem(behavior='go_to', args={
                    '_x': point.point.x, '_y': point.point.y, '_z': point.point.z,
                    'speed': 0.5,
                    'yaw_mode': YawMode.FIXED_YAW, 'yaw_angle': 0.00,
                    'wait': True
                }))
            missions[n].plan.append(MissionItem(behavior='land', args={
                'speed': 0.5,
                'wait': True
            }))
            json_msg = missions[n].json()
            self.mission_pubs[n].publish(MissionUpdate(drone_id=n, mission_id=self.mission_id, type=MissionUpdate.EXECUTE, mission=json_msg))
            # print(json_msg)


    def spin_node(self):
        while self.keep_running and rclpy.ok():
            self.__executor.spin_once(timeout_sec=0)
            sleep(0.05)

    def shutdown(self):
        self.keep_running = False
        self.destroy_client(self.plan_cli)

if __name__ == '__main__':
    rclpy.init()

    if(len(sys.argv) < 3):
        print("Usage: python3 mission_transmitter.py n_drones namespace")
        exit(-1)

    n_drones = int(sys.argv[1])
    uav_namespace = str(sys.argv[2])

    mission_transmitter = MissionTransmitter(n_drones, uav_namespace)