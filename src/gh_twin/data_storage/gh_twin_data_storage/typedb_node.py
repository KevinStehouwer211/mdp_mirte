import rclpy
from rclpy.node import Node

from gh_twin_data_storage.msg import Pest, Plant, Sensor


class TypedbStorageNode(Node):
    def __init__(self):
        super().__init__('typedb_storage_node')
        self.plants = []
        self.pests = []
        self.sensors = []

        self.create_subscription(Plant, 'plant_data', self.plant_callback, 10)
        self.create_subscription(Pest, 'pest_data', self.pest_callback, 10)
        self.create_subscription(Sensor, 'sensor_data', self.sensor_callback, 10)

        self.get_logger().info('Typedb storage node is ready to receive plant, pest, and sensor messages.')

    def plant_callback(self, msg: Plant):
        record = {
            'id': msg.id,
            'location': {'x': msg.location.x, 'y': msg.location.y},
            'color': msg.color,
            'height': msg.height,
            'is_flower': msg.is_flower,
        }
        self.plants.append(record)
        self.get_logger().info(
            f'Received plant id={msg.id}, loc=({msg.location.x:.2f},{msg.location.y:.2f}), '
            f'color={msg.color}, height={msg.height:.2f}, flower={msg.is_flower}'
        )

    def pest_callback(self, msg: Pest):
        record = {
            'id': msg.id,
            'location': {'x': msg.location.x, 'y': msg.location.y},
            'sprayed': msg.sprayed,
        }
        self.pests.append(record)
        self.get_logger().info(
            f'Received pest id={msg.id}, loc=({msg.location.x:.2f},{msg.location.y:.2f}), sprayed={msg.sprayed}'
        )

    def sensor_callback(self, msg: Sensor):
        readings = [
            {'type': r.type, 'value': r.value, 'unit': r.unit}
            for r in msg.readings
        ]
        record = {
            'id': msg.id,
            'location': {'x': msg.location.x, 'y': msg.location.y},
            'readings': readings,
        }
        self.sensors.append(record)
        self.get_logger().info(
            f'Received sensor id={msg.id}, loc=({msg.location.x:.2f},{msg.location.y:.2f}), '
            f'{len(readings)} readings'
        )


def main(args=None):
    rclpy.init(args=args)
    node = TypedbStorageNode()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
