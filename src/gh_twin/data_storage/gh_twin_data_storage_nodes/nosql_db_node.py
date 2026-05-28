#!/usr/bin/env python3

import rclpy
from rclpy.node import Node


class NoSqlDbNode(Node):
    def __init__(self):
        super().__init__('nosql_db_node')
        self.get_logger().info('NoSQL DB node placeholder. This node will later handle schemaless storage and document collections.')

    def main_loop(self):
        rclpy.spin(self)


def main(args=None):
    rclpy.init(args=args)
    node = NoSqlDbNode()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
