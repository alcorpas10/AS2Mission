#!/bin/python3

from time import sleep
import rclpy
from as2_python_api.drone_interface import DroneInterface

import sys
import threading
from typing import List
from rclpy.node import Node
from mutac_msgs.msg import Plan, Generation, Sweep
from mutac_msgs.srv import GeneratePlan
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose, Point, Point32, Polygon

from starter import Starter

landed = []

def publish_plan_1(node : Starter):
    # polygon1 = Polygon(points=[
    #     Point32(x=-1.5 , y= 1.5, z=0.0),
    #     Point32(x= 0.0 , y= 1.5, z=0.0),
    #     Point32(x=-1.5 , y=-1.5, z=0.0)
    # ])
    # polygon2 = Polygon(points=[
    #     Point32(x= 0.75 , y= 1.5, z=0.0),
    #     Point32(x= 1.5 , y= 1.5, z=0.0),
    #     Point32(x= 1.5 , y= 0.0, z=0.0),
    #     Point32(x= 0.0 , y= 0.0, z=0.0)
    # ])
    polygon1 = Polygon(points=[
        Point32(x=-2.0 , y= 2.0, z=0.0),
    #    Point32(x=-0.5 , y= 2.0, z=0.0),
        Point32(x=-0.5 , y= 1.0, z=0.0),
        Point32(x=-0.5 , y=-2.0, z=0.0),
        Point32(x=-2.0 , y=-2.0, z=0.0)
    ])
    polygon2 = Polygon(points=[
        Point32(x= 0.0 , y= 2.0, z=0.0),
        Point32(x= 2.0 , y= 2.0, z=0.0),
        Point32(x= 2.0 , y=-1.0, z=0.0),
        Point32(x= 0.0 , y= 0.0, z=0.0)
    ])

    sweeps = [
        Sweep(
            polygon=polygon1,
            orientation=Point(x=0.0, y=-0.5, z=0.0)
        ),
        Sweep(
            polygon=polygon2,
            orientation=Point(x=0.0, y=-0.5, z=0.0)
        )
    ]

    generation = Generation(sweeps=sweeps)

    genPlan = GeneratePlan.Request(generation=generation)

    resp = node.pub_plan(genPlan)
    # rclpy.spin_until_future_complete(node, resp)

    return resp

def drone_run(drone_interface: DroneInterface, starter : Starter):
    """ Run the mission """

    speed = 0.5
    sleep_time = 2.0

    drone_id = int(drone_interface.drone_id[-1])

    path2follow = starter.path[drone_id][1:]

    print("Start mission")

    ##### FOLLOW PATH #####
    sleep(sleep_time)
    # print(f"Follow path with angle {0.0}: [{path2follow}]")
    # drone_interface.follow_path.follow_path_with_yaw(path2follow, speed, angle=0.0)
    if len(path2follow) < 1:
        return 0
    for goal in path2follow:
        # if len(starter.drones_lost) < 1:
        if starter.n_drones_lost == len(starter.drones_lost):
            print(f"Drone {drone_id} - Go to with angle {0.0}: {goal}")
            drone_interface.go_to.go_to_point_with_yaw((goal[0], goal[1], goal[2]), speed=speed, angle=0.0)
            print("Go to done")
            starter.covered_points[drone_id].append(goal)
            starter.left_points[drone_id].pop(0)
            pub_rviz(controller.pubs_covered, controller.covered_points)
            pub_rviz(controller.pubs_left, controller.left_points)
        else:
            return 0
    starter.ready = False
    starter.mission = False
    print("Follow path done")

def takeoff(uav: DroneInterface):
    """ Takeoff all drones """
    uav.arm()
    uav.offboard()
    uav.takeoff(1, 0.7)
    landed[int(uav.drone_id[-1])] = False

def land(drone_interface: DroneInterface):
    """ Land all drones """
    if (landed[int(drone_interface.drone_id[-1])] == False):
        drone_interface.land(0.4)
        landed[int(drone_interface.drone_id[-1])] = True

def run_func(uavs: List[DroneInterface], func, *args):
    """ Run a function in parallel for each drone """
    threads = []
    for uav in uavs:
        t = threading.Thread(target=func, args=(uav, *args))
        threads.append(t)
        t.start()
    print("Waiting for threads to finish...")
    for t in threads:
        t.join()
    print("all done")

# def move_uavs(uavs, starter):
#     """ Move all drones """
#     run_func(uavs, drone_run, starter)
#     return

def shutdown_all(uavs):
    """ Shutdown all drones """
    print("Exiting...")
    for uav in uavs:
        uav.shutdown()
    sys.exit(1)

def confirm(uavs: List[DroneInterface], msg: str = 'Continue') -> bool:
    """ Ask for confirmation """
    confirmation = input(f"{msg}? (y/n): ")
    if confirmation == "y":
        return True
    elif confirmation == "n":
        return False
    else:
        shutdown_all(uavs)

def pub_rviz(pubs, path):
    pub_path = []
    for i in range(len(path)):
        poses_list = []
        for p in path[i]:
            poses_list.append(PoseStamped(pose=Pose(position=Point(
                x=p[0],
                y=p[1],
                z=p[2]
            ))))

        path_msg = Path()
        path_msg.header.frame_id = 'earth'
        path_msg.poses = poses_list
        pub_path.append(path_msg)
        pubs[i].publish(pub_path[i])
        # print("/path" + str(i) + ": " + str(pub_path[i]))
    # rclpy.spin_once(node, timeout_sec=0.0)


if __name__ == '__main__':
    uavs = list()

    rclpy.init()

    if(len(sys.argv) < 2):
        print("Usage: python3 mission_test_swarm.py n_drones")
        exit(-1)

    n_drones = int(sys.argv[1])
    print("N_drones:", n_drones)

    landed = [True for _ in range(n_drones)]

    for n in range(n_drones):
        # uav_name = "drone_sim_" + str(n)
        uav_name = "cf" + str(n)
        uavs.append(DroneInterface(uav_name, verbose=False, use_sim_time=True))

    controller = Starter(uavs)

    print("Publish Plan")
    if confirm(uavs, "Publish Plan"):
        publish_plan_1(controller)

    sleep(2.0)

    while(not controller.ready):
        # rclpy.spin_once(plan_getter)
        pass

    # Publish paths for rviz
    pub_rviz(controller.pubs_left, controller.left_points)

    print("Takeoff")
    if confirm(uavs, "Takeoff"):
        run_func(uavs, takeoff)

    controller.mission = True
    print("Follow mission")
    if confirm(uavs, "Follow mission"):
        while(controller.mission):
            if(controller.ready):
                pub_rviz(controller.pubs_left, controller.left_points)
                # print("Follow mission")
                # if confirm(uavs, "Follow mission"):
                #     # move_uavs(uavs, controller)
                #     run_func(uavs, drone_run, controller)
                # else:
                #     controller.mission = False
                run_func(uavs, drone_run, controller)
                sleep(0.05)
            else:
                if(controller.n_drones_lost < len(controller.drones_lost)):
                    controller.ask_replan()
                    controller.n_drones_lost = len(controller.drones_lost)
            for d in controller.drones_lost:
                landed[d] = True
            # mission = (sum([len(l) != 0 for l in plan_getter.left_points]) != 0)
            # mission = False
            # for l in plan_getter.left_points:
                # print(l)
                # mission += (len(l) > 1)
            # print(mission)

    print("Land")
    if confirm(uavs, "Land"):
        run_func(uavs, land)

    controller.shutdown()
    
    shutdown_all(uavs)

    rclpy.shutdown()

    print("Clean exit")
    exit(0)
