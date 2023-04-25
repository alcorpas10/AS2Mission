import math
import time


class Drone():
    def __init__(self, id, homebase):
        """Initializes the drone object"""
        self.id = id

        self.homebase = tuple(homebase)
        self.in_homebase = False
        self.repeat = False

        self.last_distance = float("inf")
        self.last_wp = self.position
        self.time_last_msg = time.time() # Not used by now


    def checkDrone(self, dist_trj, dist_wp):
        """Checks the drone status and returns the event code"""
        # # When the drone reaches the homebase after getting lost
        # if self.state == State.LOST and self.in_homebase:
        #     print("Drone ", self.id, " was recovered")
        #     self.reset()
        #     return 4
        
        # # When the drone did not start a mission or got lost
        # if self.state == State.NOT_STARTED or self.state == State.LOST:
        #     return -1

        # wps = self.waypoints

        # waypoint_dist = self.distance(self.position, wps[0]['point'])

        # # When the drone has to repeat the previous waypoint
        # if self.repeat:
        #     print("Drone ", self.id, " needs to repeat the last waypoint")
        #     self.repeat = False
        #     # TODO check what happens when the alarm is activated several times in a row
        #     return 5

        # # When the drone is in the last waypoint
        # if len(wps) == 1 and waypoint_dist <= dist_wp:
        #     self.advanceWP()

        #     if self.state == State.GOING_HOME:
        #         self.state = State.LANDED
        #         return 2
        #     else:
        #         self.state = State.GOING_HOME
        #         return 0

        # # When the drone is in a waypoint
        # if waypoint_dist < dist_wp:
        #     self.advanceWP()
        #     return 3

        # # When the drone is in the trajectory as expected
        # if waypoint_dist < self.last_distance:
        #     self.last_distance = waypoint_dist
        #     self.pos_in_trj = self.position

        # # This point should never be reached
        return -1
    
    def setWaypoints(self, path):
        """Saves the waypoints of the path that the drone is going to follow"""
        path_id = path.identifier.natural
        if path_id == self.id:
            self.last_wp = self.position
            self.last_distance = float("inf")

    def advanceWP(self):
        """Advances the drone to the next waypoint"""
        self.last_distance = float("inf")
        self.last_wp = self.waypoints[0]['point']

    def repeatWP(self):
        """Repeats the last covered waypoint. It is used when there is a minor error
        during the drone inspection that does not require a replan"""
        self.last_distance = float("inf")
        self.waypoints.insert(0, {'label': self.last_label, 'point': self.last_wp})
        self.repeat = True
    
    def positionCallback(self, msg):
        """Callback that updates the drone position"""
        self.time_last_msg = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.in_homebase = True if self.distance(self.position, self.homebase) < 0.2 else False

    def reset(self):
        """Resets the drone to its initial state"""
        self.last_distance = float("inf")
        self.last_wp = self.position
