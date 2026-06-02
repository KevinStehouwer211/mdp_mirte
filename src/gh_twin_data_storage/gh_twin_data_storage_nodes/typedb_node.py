#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from gh_twin_interfaces.msg import Pest, Flower, Sensor
from typedb.driver import TypeDB, SessionType, TransactionType
from std_msgs.msg import String
from lupin_greenhouse_msgs.srv import GetTagReading
from lupin_greenhouse_msgs.msg import TagReading
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

        self.create_subscription(Flower, 'flower_data', self.flower_callback, 10)
        self.create_subscription(Pest, 'pest_data', self.pest_callback, 10)
        self.create_subscription(Sensor, 'sensor_data', self.sensor_callback, 10)

        self.sensor_tag_sub = self.create_subscription(
            String, 
            '/vision/scanned_tag', # TODO: Update this topic name to match your actual vision output topic
            self.vision_tag_callback, 
            10
        )

        self.bridge_client = self.create_client(GetTagReading, '/greenhouse_bridge/get_tag_reading')
        self.hmi_publisher = self.create_publisher(TagReading, '/greenhouse/telemetry', 10)

        self.get_logger().info('Typedb storage node is ready to receive flower, pest, and sensor messages.')

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

    def _query_waypoints(self):
        query = '''
        match
          $wp isa waypoint, has id $wp_id, has x $x, has y $y;
        get $wp_id, $x, $y;
        '''
        results = self._read_query(query)
        waypoints = []
        for row in results:
            waypoints.append({
                'id': self._read_attr(row, 'wp_id'),
                'x': float(self._read_attr(row, 'x')),
                'y': float(self._read_attr(row, 'y')),
            })
        return waypoints

    def remove_waypoint(self, waypoint_id: str):
        """Remove a waypoint and all its associated TypeDB relations."""
        if not waypoint_id:
            raise ValueError('Waypoint id must be provided.')

        check_query = f'''
        match
          $wp isa waypoint, has id "{self._typeql_string(waypoint_id)}";
        get $wp;
        '''
        results = self._read_query(check_query)
        if not results:
            raise RuntimeError(f"Waypoint '{waypoint_id}' not found in TypeDB.")

        delete_query = f'''
        match
          $wp isa waypoint, has id "{self._typeql_string(waypoint_id)}";
        delete $wp;
        '''
        self._write_query(delete_query)
        self.get_logger().info(f"Removed waypoint '{waypoint_id}' from TypeDB")

    def remove_all_waypoints(self):
        """Remove all waypoints and their associated relations from TypeDB."""
        delete_query = '''
        match
          $wp isa waypoint;
        delete $wp;
        '''
        self._write_query(delete_query)
        self.get_logger().info("Removed all waypoints from TypeDB")

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
                    has bin-id "{self._typeql_string(wp['bin_id'])}",
                    has x {float(wp['x'])},
                    has y {float(wp['y'])},
                    has yaw {float(wp['yaw'])},
                    has pose-source "{self._typeql_string(wp['pose_source'])}";
                '''
                self._write_query(query)

    def _find_nearest_waypoint(self, location):
        waypoints = self._query_waypoints()
        if not waypoints:
            raise RuntimeError('No waypoints found in TypeDB.')

        best = min(
            waypoints,
            key=lambda wp: (wp['x'] - location['x']) ** 2 + (wp['y'] - location['y']) ** 2,
        )
        return best

    def _get_waypoint_for_location(self, location):
        if location is None or 'x' not in location or 'y' not in location:
            raise RuntimeError('Location must include x and y values.')

        try:
            nearest = self._find_nearest_waypoint(location)
            dx = location['x'] - nearest['x']
            dy = location['y'] - nearest['y']
            distance = (dx * dx + dy * dy) ** 0.5
            self.get_logger().info(
                f"Mapped location ({location['x']:.2f},{location['y']:.2f}) "
                f"to waypoint {nearest['id']} (distance={distance:.3f})"
            )
            return nearest['id']
        except Exception as exception:
            self.get_logger().warning(
                f"Unable to map location ({location.get('x')},{location.get('y')}) to a waypoint: {exception}. "
                f"Falling back to current robot waypoint."
            )
            return self._get_current_wp_id()

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

    def flower_callback(self, msg: Flower):
        record = {
            'id': msg.id,
            'location': {'x': msg.location.x, 'y': msg.location.y},
            'color': msg.color,
            'bloomed': msg.bloomed,
            'robot_pose': msg.robot_pose,
        }
        self.plants.append(record)
        self.get_logger().info(
            f'Received flower id={msg.id}, loc=({msg.location.x:.2f},{msg.location.y:.2f}), '
            f'color={msg.color}, bloomed={msg.bloomed}, pose=({msg.robot_pose.position.x:.2f},{msg.robot_pose.position.y:.2f})'
        )

        flower_id = f"flower{msg.id}"
        try:
            waypoint_id = self._get_waypoint_for_location(record['location'])
            query = f'''
            match
              $wp isa waypoint, has id "{self._typeql_string(waypoint_id)}";
            insert
              $flower isa plant,
                has id "{self._typeql_string(flower_id)}",
                has scan-status "not-scanned";
              $obs (observer: $wp, observed-plant: $flower) isa observation-location;
              $loc (located: $flower, at-waypoint: $wp) isa object-location;
            '''
            self._write_query(query)
            self.get_logger().info(f"Inserted flower {flower_id} at waypoint {waypoint_id} in TypeDB")
        except Exception as exception:
            self.get_logger().error(f"Failed to insert flower {flower_id} into TypeDB: {exception}")

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
            waypoint_id = self._get_waypoint_for_location(record['location'])
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
            'sensor_type': msg.sensor_type,
            'readings': readings,
        }
        self.sensors.append(record)
        self.get_logger().info(
            f'Received sensor id={msg.id}, loc=({msg.location.x:.2f},{msg.location.y:.2f}), '
            f'type={msg.sensor_type}, {len(readings)} readings'
        )

        try:
            waypoint_id = self._get_waypoint_for_location(record['location'])
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

    def vision_tag_callback(self, msg: String):
        """
        Triggered automatically when the vision node broadcasts a newly scanned tag.
        """
        tag_id = msg.data
        self.request_tag_reading(tag_id)

    def request_tag_reading(self, tag_id: str):
        """
        Call this method whenever a tag is scanned (e.g., triggered by your vision/scheduler pipeline).
        It sends an asynchronous request to the greenhouse bridge service.
        """
        # Wait for the service to be available so we don't crash on an unready system
        if not self.bridge_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('Service /greenhouse_bridge/get_tag_reading not available!')
            return

        # Construct the request payload
        request = GetTagReading.Request()
        request.tag_id = str(tag_id)

        # Invoke the service asynchronously and attach a callback for when the response arrives
        self.get_logger().info(f"Sending service request to bridge for Tag ID: {tag_id}")
        future = self.bridge_client.call_async(request)
        future.add_done_callback(self.bridge_service_callback)

    def bridge_service_callback(self, future):
        """
        Callback triggered automatically when the greenhouse bridge responds.
        """
        try:
            response = future.result()
            self.get_logger().info("Received successful service response from greenhouse bridge.")
            
            # --- TypeDB Processing Section ---
            # Add stuff here?
            
            # --- HMI Relaying Section ---
            # Extract the TagReading message payload from the response and publish it
            # so that your GUI / HMI node can consume it.
            hmi_msg = response.reading
            
            self.hmi_publisher.publish(hmi_msg)
            self.get_logger().info(f"Relayed telemetry for Tag {hmi_msg.tag_id} over /greenhouse/telemetry topic.")

        except Exception as e:
            self.get_logger().error(f"Service call failed: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = TypeDBStorageNode()
    node.connect_to_typedb()
    node.load_waypoints_from_yml()

    try:
        rclpy.spin(node)
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
