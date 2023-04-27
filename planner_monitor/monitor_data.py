from enum import Enum

from mutac_msgs.msg import LabeledPath, LabeledPoint


class State(Enum):
    """Enumerate that contains the possible states of a drone"""
    NOT_STARTED = 0
    ON_MISSION = 1
    MISSION_FINISHED = 2
    GOING_HOME = 3
    LANDED = 4
    LOST = 5

class Label(Enum):
    """Enumerate that contains the possible labels of a waypoint"""
    POSITIONING_LABEL = 0
    COVERING_LABEL = 1

class MonitorData:
    """Class that contains the basic data of a drone"""
    def __init__(self, id):
        """Initializes the drone object"""
        self.id = id

        self.state = State.NOT_STARTED

        self.position = (0.0, 0.0, 0.0)
        self.pos_in_trj = (0.0, 0.0, 0.0)

        self.waypoints = []

        self.last_label = Label.POSITIONING_LABEL
        

    def setState(self, state):
        """Setter for the state of the drone"""
        self.state = state
        if state == State.LOST:
            self.waypoints = []

    def setWaypoints(self, path):
        """Saves the waypoints of the path that the drone is going to follow 
        starting from the second point. The first one is the current position
        of the drone"""
        self.state = State.ON_MISSION # TODO change to make this work with the transmission
        self.waypoints = []
        for l_point in path.points[1:]:
            label = l_point.label.natural
            point = (l_point.point.x, l_point.point.y, l_point.point.z)
            self.waypoints.append({'label': label, 'point': point})

    def advanceWP(self):
        """Removes the first waypoint left of the drone."""
        self.last_label = self.waypoints[0]['label']
        self.waypoints = self.waypoints[1:]

    def generatePlanPath(self, asking):
        """Generates the path of the drone to be sent to the replanner.
        The path is composed by the current position of the drone and
        the left waypoints of the drone."""
        # If the drone is lost and it is not asking for the replanning it
        # does not need to send anything
        if self.state == State.LOST and not asking:
            return None
        path = LabeledPath()
        path.identifier.natural = self.id if self.state != State.LOST else -1

        point = LabeledPoint()
        pos = self.position if self.state != State.LOST else self.pos_in_trj
        point.point.x = pos[0]
        point.point.y = pos[1]
        point.point.z = pos[2]

        # TODO maybe check if waypoints are empty
        for waypoint in self.waypoints:
            l_point = LabeledPoint()
            l_point.point.x = waypoint['point'][0]
            l_point.point.y = waypoint['point'][1]
            l_point.point.z = waypoint['point'][2]
            l_point.label.natural = waypoint['label']
            path.points.append(l_point)

        if len(path.points) <= 0 or path.points[0].label.natural != self.last_label:
            point.label.natural = Label.POSITIONING_LABEL.value
        else:
            point.label.natural = path.points[0].label.natural
                
        path.points.insert(0, point)
        return path

    def positionCallback(self, msg):
        """Callback for the position of the drone"""
        self.position = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)

    def reset(self): # TODO check if there are more things to reset
        """Resets the drone to the initial state"""
        self.state = State.NOT_STARTED

        self.waypoints = []
        