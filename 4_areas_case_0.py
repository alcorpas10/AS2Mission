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
        Point32(x= -9.0, y= 5.0, z=0.0),
        Point32(x= -5.0, y= 5.0, z=0.0),
        Point32(x= -5.0, y= 1.0, z=0.0),
        Point32(x= -7.0, y= 0.0, z=0.0),
        Point32(x=-10.0, y= 0.0, z=0.0),
        Point32(x=-10.0, y= 4.0, z=0.0)
    ])
    polygon2 = Polygon(points=[
        Point32(x=  5.0, y= 5.0, z=0.0),
        Point32(x= 10.0, y= 5.0, z=0.0),
        Point32(x= 10.0, y= 1.0, z=0.0),
        Point32(x=  7.0, y= 1.0, z=0.0),
        Point32(x=  5.0, y= 3.0, z=0.0)
    ])
    polygon3 = Polygon(points=[
        Point32(x= -8.0, y=-1.0, z=0.0),
        Point32(x= -3.0, y=-1.0, z=0.0),
        Point32(x= -3.0, y=-5.0, z=0.0),
        Point32(x=-10.0, y=-5.0, z=0.0)
    ])
    polygon4 = Polygon(points=[
        Point32(x=  2.0, y=-2.0, z=0.0),
        Point32(x=  8.0, y=-2.0, z=0.0),
        Point32(x= 10.0, y=-5.0, z=0.0),
        Point32(x=  4.0, y=-5.0, z=0.0)
    ])

    sweeps = [
        Sweep(
            polygon=polygon1,
            orientation=Point(x=0.0, y=-0.3, z=0.0)
        ),
        Sweep(
            polygon=polygon2,
            orientation=Point(x=0.0, y=-0.3, z=0.0)
        ),
        Sweep(
            polygon=polygon3,
            orientation=Point(x=0.0, y=-0.3, z=0.0)
        ),
        Sweep(
            polygon=polygon4,
            orientation=Point(x=0.0, y=-0.3, z=0.0)
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
    else:
        return False

if __name__ == '__main__':
    uavs = list()

    rclpy.init()

    auto_confirm = False
    if len(sys.argv) > 1:
        if str(sys.argv[1]) == "-y":
            auto_confirm = True

    controller = Starter()

    controller.get_logger().info("Publish Plan")
    if not auto_confirm:
        if confirm("Publish Plan"):
            controller.get_logger().info("Publishing...")
            publish_plan_1(controller)
    else:
        controller.get_logger().info("Publishing...")
        publish_plan_1(controller)

    sleep(2.0)

    if not auto_confirm:
        if confirm("End"):
            pass

    controller.shutdown()

    rclpy.shutdown()

    print("Clean exit")
    exit(0)
