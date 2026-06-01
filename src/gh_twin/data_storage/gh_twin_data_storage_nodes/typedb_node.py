#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from gh_twin_data_storage.msg import Pest, Plant, Sensor
from typedb.driver import TypeDB, SessionType, TransactionType
import yaml


class TypeDBStorageNode(Node):
    def __init__(self):
        super().__init__('typedb_storage_node')

        self.declare_parameter('typedb_address', "localhost:1729")
        self.declare_parameter('database_name', "greenhouse")
        self.declare_parameter('map_id', 'greenhouse-map-001')
        self.declare_parameter('robot_id', 'robot1')
        self.declare_parameter('current_wp_id', '')
        self.declare_parameter('waypoints_file', '')

        self.typedb_address = self.get_parameter("typedb_address").value
        self.database_name = self.get_parameter("database_name").value
        self.driver = None
        
        self.map_id = self.get_parameter("map_id").value
        self.robot_id = self.get_parameter("robot_id").value
        self.waypoints_file = self.get_parameter("waypoints_file").value
        self.plants = []
        self.pests = []
        self.sensors = []
        self.scan_counter = 0

        self.create_subscription(Plant, 'plant_data', self.plant_callback, 10)
        self.create_subscription(Pest, 'pest_data', self.pest_callback, 10)
        self.create_subscription(Sensor, 'sensor_data', self.sensor_callback, 10)

        self.get_logger().info('Typedb storage node is ready to receive plant, pest, and sensor messages.')

    def connect_to_typedb(self):
        self.get_logger().info(f"Connecting to TypeDB at {self.typedb_address}")

        driver = TypeDB.core_driver(self.typedb_address)

        if not driver.databases.contains(self.database_name):
            raise RuntimeError(f"Database '{self.database_name}' does not exist.")

        self.driver = driver
        self.get_logger().info(f"Connected to database: {self.database_name}")

    def _read_attr(self, concept_map, var_name):
        return concept_map.get(var_name).as_attribute().get_value()

    def _read_query(self, query):
        if self.driver is None:
            raise RuntimeError("Call connect_to_typedb() first.")

        with self.driver.session(self.database_name, SessionType.DATA) as session:
            with session.transaction(TransactionType.READ) as transaction:
                return list(transaction.query.get(query))
            
    def _write_query(self, query):
        if self.driver is None:
            raise RuntimeError("Call connect_to_typedb() first.")

        with self.driver.session(self.database_name, SessionType.DATA) as session:
            with session.transaction(TransactionType.WRITE) as transaction:
                results = list(transaction.query.insert(query))
                transaction.commit()
                return results

    def close(self):
        if self.driver:
            self.driver.close()
            self.get_logger().info("TypeDB connection closed.")

    def _typeql_string(self, value):
        return str(value).replace("\\", "\\\\").replace('"', '\\"')

    def load_waypoints_from_yml(self):
        if not self.waypoints_file:
            self.get_logger().error(f"No waypoints file available.")
            return
        
        with open(self.waypoints_file, 'r') as file:
            waypoints = yaml.safe_load(file)
            for wp in waypoints:
                query = f'''
                insert
                  $wp isa waypoint,
                    has id "{self._typeql_string(wp['id'])}",
                    has index {int(wp['index'])},
                    has x {float(wp['x'])},
                    has y {float(wp['y'])},
                    has yaw {float(wp['yaw'])},
                    has pose-source "{self._typeql_string(wp['pose_source'])}";
                '''
                self._write_query(query)

    def _get_current_wp_id(self):
        query = f'''
        match
          $robot isa robot, has id "{self._typeql_string(self.robot_id)}";
          $wp isa waypoint, has id $wp_id;
          $loc (located: $robot, location: $wp) isa current-location;
        get $wp_id;
        '''
        results = self._read_query(query)
        if results:
            return self._read_attr(results[0], "wp_id")

        raise RuntimeError(f"No current-location found for robot '{self.robot_id}'.")

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

        plant_id = f"plant{msg.id}"
        try: 
            waypoint_id = self._get_current_wp_id()
            query = f'''
            match
              $wp isa waypoint, has id "{self._typeql_string(waypoint_id)}";
            insert
              $plant isa plant,
                has id "{self._typeql_string(plant_id)}",
                has scan-status "not-scanned";
              $obs (observer: $wp, observed-plant: $plant) isa observation-location;
              $loc (located: $plant, at-waypoint: $wp) isa object-location;
            '''
            self._write_query(query)
            self.get_logger().info(f"Inserted plant {plant_id} at waypoint {waypoint_id} in TypeDB")
        except Exception as exception:
            self.get_logger().error(f"Failed to insert plant {plant_id} into TypeDB: {exception}")

    def pest_callback(self, msg: Pest):
        record = {
            'id': msg.id,
            'location': {'x': msg.location.x, 'y': msg.location.y},
            'sprayed': msg.sprayed,
        }
        self.pests.append(record)
        self.get_logger().info(f'Received pest id={msg.id}, loc=({msg.location.x:.2f},{msg.location.y:.2f}), sprayed={msg.sprayed}')

        pest_id = f"pest{msg.id}"
        try: 
            waypoint_id = self._get_current_wp_id()
            spray_status = "sprayed" if msg.sprayed else "not-sprayed"
            query = f'''
            match
              $wp isa waypoint, has id "{self._typeql_string(waypoint_id)}";
            insert
              $pest isa pest-detection,
                has id "{self._typeql_string(pest_id)}",
                has detection-status "detected",
                has spray-status "{spray_status}";
              $loc (located: $pest, at-waypoint: $wp) isa object-location;
            '''
            self._write_query(query)
            self.get_logger().info(f"Inserted pest {pest_id} into TypeDB at waypoint {waypoint_id}")
        except Exception as exception:
            self.get_logger().error(f"Failed to insert pest {pest_id} into TypeDB: {exception}")

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

        try:
            waypoint_id = self._get_current_wp_id()
        except Exception as exception:
            self.get_logger().error(f"Failed to determine waypoint for sensor {msg.id}: {exception}")
            return
        
        self.scan_counter += 1
        scan_id = f"sensor{msg.id}-scan-{self.scan_counter}"
        try:
            query = f'''
            match
              $wp isa waypoint, has id "{self._typeql_string(waypoint_id)}";
            insert
              $scan isa scan-event,
                has id "{self._typeql_string(scan_id)}",
                has image-uri "ros://sensor/{msg.id}";
              $scanLoc (scan: $scan, location: $wp) isa waypoint-scan;
            '''
            self._write_query(query)
            self.get_logger().info(f"Inserted scan event {scan_id} at waypoint {waypoint_id}")
        except Exception as exception:
            self.get_logger().error(f"Failed to insert scan event {scan_id}: {exception}")
            return

        for i, reading in enumerate(msg.readings):
            reading_id = f"{scan_id}-reading-{i}"
            query = f'''
            match
              $scan isa scan-event, has id "{self._typeql_string(scan_id)}";
            insert
              $reading isa sensor-reading,
                has id "{self._typeql_string(reading_id)}",
                has reading-type "{self._typeql_string(reading.type)}",
                has reading-value {float(reading.value)},
                has reading-unit "{self._typeql_string(reading.unit)}";
              $readingLink (scan: $scan, reading: $reading) isa reading-link;
            '''
            try:
                self._write_query(query)
                self.get_logger().info(f"Inserted reading {reading_id} for scan {scan_id}")
            except Exception as exception:
                self.get_logger().error(f"Failed to insert reading {reading_id}: {exception}")


def main(args=None):
    rclpy.init(args=args)
    node = TypeDBStorageNode()
    node.connect_to_typedb()
    node.load_waypoints_from_yaml("waypoints.yaml")

    try:
        rclpy.spin(node)
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
