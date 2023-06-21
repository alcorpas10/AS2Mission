#!/bin/python3

from time import sleep
import rclpy

import sys
from mutac_msgs.msg import Generation, Sweep
from mutac_msgs.srv import GeneratePlan
from geometry_msgs.msg import Point, Point32, Polygon

sys.path.insert(1, 'code')

from starter import Starter

def publish_plan_1(node : Starter):
    polygon1 = Polygon(points=[
        Point32(x=-2.0 , y= 2.0, z=0.0),
        Point32(x=-0.5 , y= 1.0, z=0.0),
        Point32(x=-0.5 , y=-2.0, z=0.0),
        Point32(x=-2.0 , y=-2.0, z=0.0)
    ])
    polygon2 = Polygon(points=[
        Point32(x= 0.25, y= 2.0, z=0.0),
        Point32(x= 2.0 , y= 2.0, z=0.0),
        Point32(x= 2.0 , y=-1.0, z=0.0),
        Point32(x= 0.25, y= 0.0, z=0.0)
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

    return resp

def confirm(msg: str = 'Continue') -> bool:
    """ Ask for confirmation """
    confirmation = input(f"{msg}? (y/n): ")
    if confirmation == "y":
        return True
    elif confirmation == "n":
        return False
    else:
        sys.exit(1)

if __name__ == '__main__':
    uavs = list()

    rclpy.init()

    auto_confirm = False
    if len(sys.argv) > 1:
        if str(sys.argv[1]) == "-y":
            auto_confirm = True

    controller = Starter()

    # controller.get_logger().info("Publish Plan")
    # if not auto_confirm:
    #     if confirm("Publish Plan"):
    #         controller.get_logger().info("Publishing...")
    #         publish_plan_1(controller)
    # else:
    #     controller.get_logger().info("Publishing...")
    #     publish_plan_1(controller)

    sleep(20)

    print('LAND DRONE 0')
    controller.pub_land(0)

    sleep(3)

    print('LAND DRONE 2')
    controller.pub_land(2)

    sleep(2.0)

    if not auto_confirm:
        if confirm("End"):
            pass

    controller.shutdown()

    rclpy.shutdown()

    print("Clean exit")
    exit(0)
