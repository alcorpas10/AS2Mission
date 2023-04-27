import rclpy
from rclpy.node import Node
from rclpy import qos

from mutac_msgs.msg import Alarm, State, Plan, Identifier
from mutac_msgs.srv import Replan

from geometry_msgs.msg import PoseStamped
from as2_msgs.msg import MissionUpdate
from std_msgs.msg import Empty

from monitor.drone import Drone


class ExecutionMonitor(Node):
    def __init__(self, id):
        """Initializes the execution monitor node"""
        super().__init__('execution_monitor_' + str(id),
                        allow_undeclared_parameters=True,
                        automatically_declare_parameters_from_overrides=True)
        self.id = id

        # Monitor constant parameters
        self.dist_trj = self.get_parameter('distance.trajectory').value
        self.dist_wp = self.get_parameter('distance.waypoint').value

        # Obtains the homebase position and creates the drone object
        homebase = self.get_parameter('homebase.drone'+str(self.id+1)).value
        self.drone = Drone(self.id, homebase)

        # Initializes the ros2 publishers and subscribers
        self.initializePublishers()
        self.initializeSubscribers()

        # Initializes the ros2 clients
        self.ask_replan_client = self.create_client(Replan, '/mutac/ask_replan')
        self.provide_wp_client = self.create_client(Replan, '/mutac/provide_wps')

        # Initializes the timer
        self.timer_period = 0.25 # The rate is 4 Hz
        self.timer = self.create_timer(self.timer_period, self.timerCallback)


    def initializePublishers(self):
        """Initializes the publishers"""
        self.event_pub = self.create_publisher(State, '/mutac/drone_events', qos.QoSProfile(reliability=qos.ReliabilityPolicy.RELIABLE, depth=100))
        self.covered_pub = self.create_publisher(Identifier, '/mutac/covered_points', qos.QoSProfile(reliability=qos.ReliabilityPolicy.RELIABLE, depth=100))

    def initializeSubscribers(self):
        """Initializes the subscribers. The topics are the ones used in Aerostack. To monitor
        real drones the 'cf' uav_name should be used instead of 'drone_sim'"""
        uav_name = "/drone_sim_"+str(self.id)
        # uav_name = "/cf"+str(self.id)

        # Aerostack topics
        self.pose_sub = self.create_subscription(PoseStamped, uav_name+'/self_localization/pose', self.drone.positionCallback, qos.qos_profile_sensor_data)

        # Mutac topics
        self.trj_sub = self.create_subscription(MissionUpdate, '/'+uav_name+'/mission', self.trajectoryCallback, qos.QoSProfile(reliability=qos.ReliabilityPolicy.RELIABLE, depth=10))
        self.replan_sub = self.create_subscription(Empty, '/mutac/request_wps', self.replanCallback, 100) # TODO check the published message
        self.alarm_sub = self.create_subscription(Alarm, '/mutac/drone_alarm', self.alarmCallback, 100) # TODO Doesn't exist in Aerostack by now

    def timerCallback(self):
        """At a certain rate it is checked the state of the drone along the mission.
        If an event is detected, it is published"""
        event_id = self.drone.checkDrone(self.dist_trj, self.dist_wp)

        if event_id == 0: # MISSION_FINISHED
            msg = State()
            msg.identifier.natural = self.id
            msg.state = event_id
            self.event_pub.publish(msg)
            self.drone.reset()
            # self.askReplan() # TODO to let the drone help once it has finished his mission

        elif event_id == 1: # LOST
            msg = State()
            msg.identifier.natural = self.id
            msg.state = event_id
            if self.drone.deviated:
                msg.type = State.HOMEBASE
                msg.position.x = self.drone.homebase[0]
                msg.position.y = self.drone.homebase[1]
                msg.position.z = self.drone.homebase[2]
            else:
                msg.type = State.LAND                
            self.event_pub.publish(msg)
            self.askReplan()
            self.drone.waypoints = []

        elif event_id == 2: # LANDED
            msg = State()
            msg.identifier.natural = self.id
            msg.state = event_id
            self.event_pub.publish(msg)

        elif event_id == 3: # WP ARRIVED
            msg = Identifier()
            msg.natural = self.id
            self.covered_pub.publish(msg)

        elif event_id == 4: # RECOVERED
            msg = State()
            msg.identifier.natural = self.id
            msg.state = State.RECOVERED
            self.event_pub.publish(msg)
            self.askReplan()

        elif event_id == 5: # WP REPEATED
            msg = State()
            msg.identifier.natural = self.id
            msg.state = State.WP_REPEATED
            pos = self.drone.waypoints[0]['point']
            msg.position.x = pos[0]
            msg.position.y = pos[1]
            msg.position.z = pos[2]
            self.event_pub.publish(msg)

    def trajectoryCallback(self, msg):
        """Callback for the trajectory topic. It is used to set the waypoints of the drone"""
        if msg.drone_id == self.id and msg.type == MissionUpdate.EXECUTE:
            self.drone.setWaypoints(msg.mission)
        else:
            self.get_logger().warn("Received a trajectory for another drone")

    def replanCallback(self, msg):
        """Callback for the replan topic. When accessed the planner 
        is asking for the left waypoints of the drone to replan the trajectory"""
        path = self.drone.generatePlanPath(False)

        if path != None: # If there are waypoints left
            srv = Replan.Request()
            srv.path = path

            while not self.provide_wp_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().warn('Service not available, waiting again...')
            
            # The waypoints are sent through the provide_wp service
            self.provide_wp_client.call_async(srv)

    def askReplan(self):
        """Asks for a replan to the planner. It is used when the drone is 
        lost or recovered"""
        srv = Replan.Request()
        srv.path = self.drone.generatePlanPath(True)
        self.get_logger().info("Asking replan: "+str(len(srv.path.points)))

        while not self.ask_replan_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Service not available, waiting again...')
        
        # The request is sent through the ask_replan service
        self.ask_replan_client.call_async(srv)

    def alarmCallback(self, msg):
        """Callback for the alarm topic. When error detection is done in other node and
        an error must be monitored, it is sent through this topic"""
        if msg.identifier.natural == self.id:
            if msg.alarm == Alarm.PHOTO_ERROR or msg.alarm == Alarm.VIBRATION_ERROR:
                self.drone.repeatWP()


class GetParam(Node):
    """Auxiliary class to get the drone id from the launch file and
    initialize the monitor node with it"""
    def __init__(self):
        super().__init__('GetParam',
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True)

def main(args=None):
    rclpy.init(args=args)

    id = GetParam().get_parameter('drone_id').value

    exec_mon = ExecutionMonitor(id)
    rclpy.spin(exec_mon)

if __name__ == '__main__':
    main()