import rclpy
from rclpy.node import Node
from rclpy import qos

from mutac_msgs.msg import Alarm, State
from sensor_msgs.msg import BatteryState, Imu

from monitor.drone import Drone


class SensorMonitor(Node):
    def __init__(self, id):
        """Initializes the sensor monitor node"""
        super().__init__('sensor_monitor_' + str(id),
                        allow_undeclared_parameters=True,
                        automatically_declare_parameters_from_overrides=True)
        self.id = id

        # Creates the drone object
        self.drone = Drone(self.id)

        # Initializes the ros2 publishers and subscribers
        self.event_pub = self.create_publisher(State, 'drone_events', qos.QoSProfile(reliability=qos.ReliabilityPolicy.RELIABLE, depth=100))
        self.initializeSubscribers()

        # Initializes the timer
        self.timer_period = 0.5 # TODO the rate was 3, now is 2
        self.timer = self.create_timer(self.timer_period, self.timerCallback)


    def initializeSubscribers(self):
        """Initializes the subscribers. The topics are the ones used in Aerostack. To monitor
        real drones the 'cf' uav_name should be used instead of 'drone_sim'"""
        uav_name = "/drone_sim_"+str(self.id)
        # uav_name = "/cf"+str(self.id)

        # Aerostack topics
        self.battery_sub = self.create_subscription(BatteryState, uav_name+'/sensor_measurements/battery', self.drone.batteryCallback, qos.qos_profile_sensor_data)
        self.imu_sub = self.create_subscription(Imu, uav_name+'/sensor_measurements/imu', self.drone.imuCallback, qos.qos_profile_sensor_data)
        self.alarm_sub = self.create_subscription(Alarm, 'drone_alarm', self.alarmCallback, 100) # TODO Doesn't exist in Aerostack by now

    def timerCallback(self):
        """At a certain rate it is checked the state of the drone along the mission.
        If an event is detected, it is published"""
        event_id = self.drone.checkDrone()

        if event_id == 1: # LOST
            msg = State()
            msg.identifier.natural = self.id
            msg.state = event_id
            if not self.drone.camera_ok:
                msg.type = State.HOMEBASE
            else:
                msg.type = State.LAND                
            self.event_pub.publish(msg)

        elif event_id == 5: # WP REPEATED
            msg = State()
            msg.identifier.natural = self.id
            msg.state = State.WP_REPEATED
            self.event_pub.publish(msg)

    def alarmCallback(self, msg):
        """Callback for the alarm topic. When error detection is done in other node and
        an error must be monitored, it is sent through this topic"""
        if msg.identifier.natural == self.id:
            if msg.alarm == Alarm.CAMERA_FAILURE:
                self.drone.camera_ok = False
            elif msg.alarm == Alarm.PHOTO_ERROR or msg.alarm == Alarm.VIBRATION_ERROR:
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

    sensor_mon = SensorMonitor(id)
    rclpy.spin(sensor_mon)

if __name__ == '__main__':
    main()