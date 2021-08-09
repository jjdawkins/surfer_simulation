#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os
import sys
import rclpy
from gazebo_msgs.srv import SpawnEntity

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('spawn_model_node')
    cli = node.create_client(SpawnEntity, '/spawn_entity')
    node.declare_parameter('name')
    node.declare_parameter('init_pos')
    my_name = node.get_parameter('name')
    init_pos = node.get_parameter('init_pos')
    print(str(my_name.value))
    content = ""
    if sys.argv[1] is not None:
        with open(sys.argv[1], 'r') as content_file:
            content = content_file.read()

    req = SpawnEntity.Request()
    req.name = 'surfer' #str(my_name.value)
    req.xml = content
    req.initial_pose.position.x = float(init_pos.value[0])
    req.initial_pose.position.y = float(init_pos.value[1])
    req.robot_namespace = 'surfer'#str(my_name.value)
    req.reference_frame = "world"

    while not cli.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service not available, waiting again...')

    future = cli.call_async(req)
    rclpy.spin_until_future_complete(node, future)

    if future.result() is not None:
        node.get_logger().info(
            'Result ' + str(future.result().success) + " " + future.result().status_message)
    else:
        node.get_logger().info('Service call failed %r' % (future.exception(),))

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
