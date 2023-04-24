from rclpy.node import Node
from mutac_msgs.msg import Plan, LabeledPath, LabeledPoint, Label, Identifier
from geometry_msgs.msg import Point
from nav_msgs.msg import Path
from std_msgs.msg import Int32
import threading

from as2_python_api.drone_interface import DroneInterface

from time import sleep
import rclpy
import rclpy.executors
from mutac_msgs.srv import GeneratePlan, UpdatePlan

class Starter(Node):
    def __init__(self, uavs):
        super().__init__("PlanGetter")
        self.n_drones = len(uavs)
        self.path = list()
        self.covered_points = list(list() for _ in range(len(uavs)))
        self.left_points = list()
        self.pubs_left = list()
        self.pubs_covered = list()
        self.uavs = uavs
        self.ready = False
        self.mission = False
        self.keep_running = True

        self.drones_lost = list()
        self.n_drones_lost = 0

        self.plan_sub = self.create_subscription(Plan, "/mutac/planned_paths", self.path_callback_ , 10)
        self.plan_cli = self.create_client(GeneratePlan, '/mutac/generate_plan')
        self.plan_cli_update = self.create_client(UpdatePlan, '/mutac/update_plan')
        self.land_sub = self.create_subscription(Int32, "/emergency_land", self.land_callback, 10)

        for i in range(len(uavs)):
            nombre = "path" + str(i) + "_left"
            self.pubs_left.append(self.create_publisher(Path, nombre, 10))
        for i in range(len(uavs)):
            nombre = "path" + str(i) + "_covered"
            self.pubs_covered.append(self.create_publisher(Path, nombre, 10))

        self.__executor = rclpy.executors.SingleThreadedExecutor()
        self.__executor.add_node(self)

        self.t_spin_node = threading.Thread(target=self.spin_node)
        self.t_spin_node.start()

    def path_callback_(self, msg):
        print("Got plan")
        self.left_points = list()
        self.path = list()

        for path in range(self.n_drones):
            # print(path.identifier.natural, path.points[1].point.x, path.points[1].point.y, path.points[1].point.z)
            self.path.append(list())
            self.left_points.append(list())
        for path in msg.paths:
            path_id = path.identifier.natural
            for p in path.points:
                self.path[path_id].append([p.point.x, p.point.y, p.point.z, p.label])
            self.left_points[path_id] = self.path[path_id]
            self.covered_points[path_id].append([path.points[0].point.x, path.points[0].point.y, path.points[0].point.z, path.points[0].label])
        self.ready = True
        # print(self.path)
    
    def pub_plan(self, plan):
        while not self.plan_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        resp = self.plan_cli.call_async(plan)
        return resp
    
    def land_callback(self, msg : Int32):
        land_drone = msg.data
        if(land_drone < self.n_drones):
            self.get_logger().info("Prepare Land Drone " + str(land_drone))
            self.ready = False
            self.drones_lost.append(land_drone)
            self.uavs[land_drone].go_to.stop()
            sleep(1.0)
            self.get_logger().info("Run Land Drone " + str(land_drone))
            t = threading.Thread(target=lambda: self.uavs[land_drone].land(0.4), args=[])
            t.start()
        else:
            self.get_logger().info("Land Error")
        
    def ask_replan(self):
        paths = [LabeledPath() for _ in range(self.n_drones)]
        for n, p in enumerate(paths):
            p.identifier = Identifier(natural=n) if n not in self.drones_lost else Identifier(natural=-1)
            p.points = [LabeledPoint(
                label=punto[3],
                point=Point(
                    x=punto[0],
                    y=punto[1],
                    z=punto[2]
                )
            ) for punto in self.left_points[n]]

        # for d in self.drones_lost:
        #     self.left_points[d].clear()
        
        # print(paths)

        new_plan = UpdatePlan.Request(plan=Plan(paths=paths))

        while not self.plan_cli_update.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        resp = self.plan_cli_update.call_async(new_plan)
        return resp

    def spin_node(self):
        while self.keep_running and rclpy.ok():
            if(self.n_drones_lost < len(self.drones_lost)):
                self.ready = False
            self.__executor.spin_once(timeout_sec=0)
            sleep(0.05)
    
    def shutdown(self):
        """Shutdown properly"""
        self.keep_running = False
        self.destroy_subscription(self.plan_sub)
        self.destroy_subscription(self.land_sub)
        self.destroy_client(self.plan_cli)
