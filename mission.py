#!/bin/python3

from time import sleep
import rclpy
from as2_python_api.drone_interface import DroneInterface


def drone_run(drone_interface: DroneInterface):
    """ Run the mission """

    speed = 2.0
    takeoff_height = 1.0
    height = 2.0

    sleep_time = 2.0

    dim = 2.0
    path = [
        [dim, dim, height],
        [dim, -dim, height],
        [-dim, dim, height],
        [-dim, -dim, height],
        [0.0, 0.0, takeoff_height],
    ]

    print("Start mission")

    ##### ARM OFFBOARD #####
    drone_interface.offboard()
    drone_interface.arm()

    ##### TAKE OFF #####
    print("Take Off")
    drone_interface.takeoff(takeoff_height, speed=1.0)
    print("Take Off done")
    sleep(sleep_time)

    ##### FOLLOW PATH #####
    sleep(sleep_time)
    print(f"Follow path with path facing: [{path}]")
    drone_interface.follow_path.follow_path_with_path_facing(path, speed)
    print("Follow path done")

    sleep(sleep_time)
    print(f"Follow path with keep yaw: [{path}]")
    drone_interface.follow_path.follow_path_with_keep_yaw(path, speed)
    print("Follow path done")

    sleep(sleep_time)
    print(f"Follow path with angle {-1.57}: [{path}]")
    drone_interface.follow_path.follow_path_with_yaw(path, speed, angle=-1.57)
    print("Follow path done")

    ##### GO TO #####
    for goal in path:
        print(f"Go to with path facing {goal}")
        drone_interface.go_to.go_to_point_path_facing(goal, speed=speed)
        print("Go to done")
    sleep(sleep_time)

    for goal in path:
        print(f"Go to with keep yaw {goal}")
        drone_interface.go_to.go_to_point(goal, speed=speed)
        print("Go to done")
    sleep(sleep_time)

    for goal in path:
        print(f"Go to with angle {-1.57}: {goal}")
        drone_interface.go_to.go_to_point_with_yaw(goal, speed=speed, angle=-1.57)
        print("Go to done")
    sleep(sleep_time)

    ##### LAND #####
    print("Landing")
    drone_interface.land(speed=0.5)
    print("Land done")

    drone_interface.disarm()


if __name__ == '__main__':
    rclpy.init()

    uav_name = "drone_sim_0"
    uav = DroneInterface(uav_name, verbose=False, use_sim_time=True)

    drone_run(uav)

    uav.shutdown()
    rclpy.shutdown()

    print("Clean exit")
    exit(0)
