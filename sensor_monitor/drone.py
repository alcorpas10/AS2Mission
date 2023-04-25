
class Drone():
    def __init__(self, id):
        """Initializes the drone object"""
        self.id = id

        self.battery = 100
        self.camera_ok = True
        self.imu_err_count = 0
        self.repeat = False


    def checkDrone(self):
        """Checks the drone status and returns the event code"""
        # When the drone camera is broken
        if not self.camera_ok:
            print("Drone ", self.id, " has a broken camera")
            return 1
        
        # When the drone has very low battery
        if self.battery <= 5:
            print("Drone ", self.id, " has very low battery")
            return 1
        
        # When the IMU is too high several times
        if self.imu_err_count > 2:
            print("Drone ", self.id, " has IMU problems")
            self.imu_err_count = 0
            return 1

        # When the drone has to repeat the previous waypoint
        if self.repeat:
            print("Drone ", self.id, " needs to repeat the last waypoint")
            self.repeat = False
            return 5
        
        # Nothing happened
        return -1

    def repeatWP(self):
        """When there is a minor error during the drone execution"""
        self.repeat = True

    def batteryCallback(self, msg):
        """Callback that updates the drone battery"""
        self.battery = msg.percentage

    def imuCallback(self, msg):
        angular_vel = msg.angular_velocity
        linear_acc = msg.linear_acceleration
        if abs(angular_vel.x) > 1 or abs(angular_vel.y) > 1.5 or abs(angular_vel.z) > 0.25:
            self.imu_err_count += 1
        if abs(angular_vel.x) < -1 or abs(angular_vel.y) < -1.5 or abs(angular_vel.z) < -0.25:
            self.imu_err_count += 1
        if abs(linear_acc.x) > 1.5 or abs(linear_acc.y) > 1.5 or abs(linear_acc.z) > 15.0:
            self.imu_err_count += 1
        if abs(linear_acc.x) < -1 or abs(linear_acc.y) < -1 or abs(linear_acc.z) < 6.0:
            self.imu_err_count += 1

    def reset(self):
        """Resets the drone to its initial state"""
        self.battery = 100 # TODO check how the real battery works
        self.camera_ok = True
