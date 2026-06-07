#!/usr/bin/env python3

import argparse
import re
import subprocess
from pathlib import Path

import rclpy
from rclpy.node import Node

from gh_twin_msgs.msg import Pest, Flower, Sensor
from typedb.driver import TypeDB, SessionType, TransactionType
import os


class PddlPlannerNode(Node):    
    def __init__(
        self,
        domain_file=None,
        problem_file=None,
        planner_cmd=None,
        typedb_address=None,
        database_name=None,
        node_name='pddl_planner_node',
        use_global_arguments=True,
    ):
        super().__init__(node_name, use_global_arguments=use_global_arguments)

        self.declare_parameter('typedb_address', "localhost:1729")
        self.declare_parameter('database_name', "greenhouse")
        self.declare_parameter('map_id', 'greenhouse-map-001')
        self.declare_parameter('robot_id', 'robot1')
        self.declare_parameter('arm_id', 'arm1')

        self.typedb_address = typedb_address or self.get_parameter("typedb_address").value
        self.database_name = database_name or self.get_parameter("database_name").value
        self.driver = None
        
        self.map_id = self.get_parameter("map_id").value
        self.robot_id = self.get_parameter("robot_id").value
        self.arm_id = self.get_parameter("arm_id").value

        self.domain_file = Path(domain_file) if domain_file else None
        self.problem_file = Path(problem_file) if problem_file else None
        self.planner_cmd = planner_cmd
        self.domain_text = ''
        self.problem_text = ''
        self.base_problem_text = ''
        self.location_names = []

        if self.domain_file and self.problem_file:
            self.load_pddl_files(self.domain_file, self.problem_file)

        self.get_logger().info('PDDL planner node initialized.')

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
            raise RuntimeError("Not connected to TypeDB.")

        with self.driver.session(self.database_name, SessionType.DATA) as session:
            with session.transaction(TransactionType.READ) as transaction:
                return list(transaction.query.get(query))
            
    def _write_query(self, query):
        if self.driver is None:
            raise RuntimeError("Not connected to TypeDB.")

        with self.driver.session(self.database_name, SessionType.DATA) as session:
            with session.transaction(TransactionType.WRITE) as transaction:
                results = list(transaction.query.insert(query))
                transaction.commit()
                return results

    def close(self):
        if self.driver:
            self.driver.close()
            self.get_logger().info("TypeDB connection closed.")

    def query_robot_arm_pair(self):
        query = """
        match
          $robot isa robot, has id $robot_id;
          $arm isa arm, has id $arm_id, has arm-state $arm_state;
          $mount (robot-body: $robot, arm-part: $arm) isa mounted-arm;
        get $robot_id, $arm_id, $arm_state;
        """

        results = self._read_query(query)

        if not results:
            raise RuntimeError("No robot-arm pair found. Check TypeDBdata file.")

        row = results[0]
        return {
            "robot_id": self._read_attr(row, "robot_id"),
            "arm_id": self._read_attr(row, "arm_id"),
            "arm_state": self._read_attr(row, "arm_state"),
        }

    def query_current_robot_waypoint(self, robot_id):
        query = f"""
        match
          $robot isa robot, has id "{robot_id}";
          $wp isa waypoint, has id $wp_id;
          $loc (located: $robot, location: $wp) isa current-location;
        get $wp_id;
        """

        results = self._read_query(query)

        if not results:
            raise RuntimeError(f"No current-location found for robot '{robot_id}'. Check TypeDB data file.")

        return self._read_attr(results[0], "wp_id")

    def query_waypoints(self):
        query = """
        match
          $wp isa waypoint, has id $wp_id;
        get $wp_id;
        """

        return sorted({self._read_attr(r, "wp_id") for r in self._read_query(query)})

    def query_connections(self):
        query = """
        match
          $from isa waypoint, has id $from_id;
          $to isa waypoint, has id $to_id;
          $edge (from: $from, to: $to) isa path-connection;
        get $from_id, $to_id;
        """

        connections = {(self._read_attr(r, "from_id"), self._read_attr(r, "to_id")) for r in self._read_query(query)}

        waypoint_bin_association = self.query_waypoint_bin_association()

        for waypoint_id in waypoint_bin_association.values():
            for from_id in waypoint_id:
                for to_id in waypoint_id:
                    if from_id != to_id:
                        connections.add((from_id, to_id))

        return sorted(connections)
    
    def query_waypoint_bin_association(self):
        query = """
        match 
            $wp isa waypoint, has id $wp_id, has bin-id $bin_id;
        get $wp_id, $bin_id;
        """

        waypoint_bin_association = {}
        for r in self._read_query(query):
            wp_id = self._read_attr(r, "wp_id")
            bin_id = self._read_attr(r, "bin_id")
            
            if bin_id not in waypoint_bin_association:
                waypoint_bin_association[bin_id] = set()
            waypoint_bin_association[bin_id].add(wp_id)

        return {
            bin_id: sorted(waypoint_ids)
            for bin_id, waypoint_ids in waypoint_bin_association.items()
            if len(waypoint_ids) > 1
        }

    def query_plants_with_waypoints(self):
        query = """
        match
          $plant isa plant, has id $plant_id, has scan-status $scan_status;
          $wp isa waypoint, has id $wp_id;
          $obs (observer: $wp, observed-plant: $plant) isa observation-location;
        get $plant_id, $wp_id, $scan_status;
        """

        plants = []
        for r in self._read_query(query):
            plants.append({
                "plant_id": self._read_attr(r, "plant_id"),
                "wp_id": self._read_attr(r, "wp_id"),
                "scan_status": self._read_attr(r, "scan_status"),
            })

        return sorted(plants, key=lambda p: p["plant_id"])

    def query_pests_with_waypoints(self):
        query = """
        match
          $pest isa pest-detection, has id $pest_id, has spray-status $spray_status;
          $wp isa waypoint, has id $wp_id;
          $loc (located: $pest, at-waypoint: $wp) isa object-location;
        get $pest_id, $wp_id, $spray_status;
        """

        pests = []
        for r in self._read_query(query):
            pests.append({
                "pest_id": self._read_attr(r, "pest_id"),
                "wp_id": self._read_attr(r, "wp_id"),
                "spray_status": self._read_attr(r, "spray_status"),
            })

        return sorted(pests, key=lambda p: p["pest_id"])
    
    """
    def query_sensors_with_waypoints(self):
        query =
        match
          $sensor isa sensor, has id $sensor_id;
          $wp isa waypoint, has id $wp_id;
          $obs (observer: $wp, observed-sensor: $sensor) isa observation-location;
          $reading (measured-by: $sensor) isa sensor-reading, has type $type, has value $value, has unit $unit;
        
        return sorted({
            (self._read_attr(r, "wp_id"), self._read_attr(r, "sensor_id"), self._read_attr(r, "type"))
            for r in self._read_query(query)
        })
    """

    def load_pddl_files(self, domain_file: Path, problem_file: Path):
        self.domain_file = Path(domain_file)
        self.problem_file = Path(problem_file)

        self.domain_text = self.domain_file.read_text(encoding='utf-8')
        self.problem_text = self.problem_file.read_text(encoding='utf-8')
        self.base_problem_text = self.problem_text

        self.get_logger().info(f'Loaded domain file: {self.domain_file}')
        self.get_logger().info(f'Loaded problem file: {self.problem_file}')

    def replan(self, output_problem_file=None, blocked_edges=None, visited_waypoints=None, scanned_waypoints=None, infeasible_waypoints=None):
        problem_file = output_problem_file or self.problem_file
        self.generate_pddl_problem(problem_file, 
            blocked_edges=blocked_edges,
            visited_waypoints=visited_waypoints,
            scanned_waypoints=scanned_waypoints,
            infeasible_waypoints=infeasible_waypoints,
        )
        self.get_logger().info('Problem file updated for replanning')
        return self.run_planner(domain_file=self.domain_file, problem_file=problem_file)

    def _to_readable_pddl_name(self, identifier) -> str:
        name = str(identifier).strip().lower().replace(' ', '_').replace('.', '_')
        if name[0].isdigit():
            name = "id_" + name
        return name

    def _format_pddl_objects(self, robot_id, arm_id, waypoints, plants, pests):
        objects = [
            f"    {self._to_readable_pddl_name(robot_id)} - robot",
            f"    {self._to_readable_pddl_name(arm_id)} - arm",
            "    " + " ".join(self._to_readable_pddl_name(wp) for wp in waypoints) + " - waypoint",
        ]

        plant_names = [f'{self._to_readable_pddl_name(plant["plant_id"])}' for plant in plants]
        pest_names = [f'{self._to_readable_pddl_name(pest["pest_id"])}' for pest in pests]
        #sensor_names = [f'sensor-{self._to_readable_pddl_name(item["id"])}' for item in sensors]
        #location_names = [self._location_name(item["location"]) for item in plants + pests]
        #self.location_names = sorted(set(location_names))

        if plant_names:
            objects.append("    " + " ".join(plant_names) + ' - plant')
        if pest_names:
            objects.append("    " + " ".join(pest_names) + ' - pest')
        #if sensor_names:
        #    objects.append("    " + " ".join(sensor_names) + ' - sensor')
        #if self.location_names:
        #    objects.append("    " + ' '.join(self.location_names) + ' - location')

        if not objects:
            return '(:objects)\n'
        
        return '\n    '.join(objects)

    def _format_pddl_init(self, robot_id, arm_id, arm_state, current_wp, waypoints, connections, plants, pests, visited_waypoints=None, scanned_waypoints=None):
        visited_waypoints = set(visited_waypoints or set())
        scanned_waypoints = set(scanned_waypoints or set())

        init = [
            f"    (robot-at {self._to_readable_pddl_name(robot_id)} {self._to_readable_pddl_name(current_wp)})"
        ]
        visited_waypoints.add(self._to_readable_pddl_name(current_wp))
        scanned_waypoints.add(self._to_readable_pddl_name(current_wp))

        for waypoint in sorted(visited_waypoints):
            init.append(f"    (wp-visited {waypoint})")

        for waypoint in sorted(scanned_waypoints):
            init.append(f"    (wp-scanned {waypoint})")

        if arm_state == "base":
            init.append(f"    (arm-base {self._to_readable_pddl_name(self.arm_id)})")
        elif arm_state == "scan":
            init.append(f"    (arm-scan {self._to_readable_pddl_name(self.arm_id)})")
        elif arm_state == "spray":
            init.append(f"    (arm-spray {self._to_readable_pddl_name(self.arm_id)})")
        else:
            raise RuntimeError(f"Unsupported arm-state for planning: {arm_state}")

        for from_wp, to_wp in connections:
            init.append(f"    (connected-wp {self._to_readable_pddl_name(from_wp)} {self._to_readable_pddl_name(to_wp)})")

        for plant in plants:
            plant_name = self._to_readable_pddl_name(plant["plant_id"])
            wp_name = self._to_readable_pddl_name(plant["wp_id"])
            init.append(f"    (plant-at {plant_name} {wp_name})")
            #lines.append(f'; color={plant["color"]} height={plant["height"]:.2f} flower={plant["is_flower"]}')

        for pest in pests:
            pest_name = self._to_readable_pddl_name(pest["pest_id"])
            wp_name = self._to_readable_pddl_name(pest["wp_id"])
            init.append(f"    (pest-at {pest_name} {wp_name})")
            if pest["spray_status"] == "sprayed":
                init.append(f"    (pest-sprayed {pest_name})")
        """
        for sensor in sensors:
            sensor_name = f'sensor-{sensor["id"]}'
            wp_name = self._location_name(sensor["wp_id"])
            init.append(f"    (sensor {sensor_name})")
            init.append(f"    (sensor-at {sensor_name} {wp_name})")
            for reading in sensor['readings']:
                init.append(f"    ; sensor reading {reading['type']}={reading['value']}{reading['unit']}")
        """
        if not init:
            return '(:init)\n'
        return '\n    '.join(init)
    
    def _format_pddl_goal(self, waypoints, plants, pests, visited_waypoints=None, scanned_waypoints=None, infeasible_waypoints=None):
        visited_waypoints = set(visited_waypoints or set())
        scanned_waypoints = set(scanned_waypoints or set())
        infeasible_waypoints = set(infeasible_waypoints or set())
        goal = []

        for pest in pests:
            pest_wp = self._to_readable_pddl_name(pest["wp_id"])
            if pest["spray_status"] == "not-sprayed" and pest_wp not in infeasible_waypoints:
                goal.append(f"      (pest-sprayed {self._to_readable_pddl_name(pest['pest_id'])})")
        
        for waypoint_id in waypoints:
            waypoint_name = self._to_readable_pddl_name(waypoint_id)
            if waypoint_name in infeasible_waypoints:
                continue
            if waypoint_name not in visited_waypoints:
                goal.append(f"      (wp-visited {waypoint_name})")
            if waypoint_name not in scanned_waypoints:
                goal.append(f"      (wp-scanned {waypoint_name})")

        if not goal:
            goal.append("      ; no pending scan or spray goals")
        return '\n    '.join(goal)


    def generate_pddl_problem(self, output_file="problem_generated.pddl", blocked_edges=None, visited_waypoints=None, scanned_waypoints=None, infeasible_waypoints=None):
        self.get_logger().info(f"Generating PDDL problem: {output_file}")
        blocked_edges = set(blocked_edges or set())
        visited_waypoints = set(visited_waypoints or set())
        scanned_waypoints = set(scanned_waypoints or set())
        infeasible_waypoints = set(infeasible_waypoints or set())

        robot_arm = self.query_robot_arm_pair()
        robot_id = robot_arm["robot_id"]
        arm_id = robot_arm["arm_id"]
        arm_state = robot_arm["arm_state"]

        current_wp = self.query_current_robot_waypoint(robot_id)
        current_wp_name = self._to_readable_pddl_name(current_wp)
        visited_waypoints.add(current_wp_name)
        scanned_waypoints.add(current_wp_name)

        waypoints = self.query_waypoints()
        connections = self.query_connections()
        connections = [
            (from_wp, to_wp)
            for from_wp, to_wp in connections
            if (self._to_readable_pddl_name(from_wp), self._to_readable_pddl_name(to_wp)) not in blocked_edges
        ]
        plants = self.query_plants_with_waypoints()
        pests = self.query_pests_with_waypoints()
        #sensors = self.query_sensors_with_waypoints()

        objects = self._format_pddl_objects(robot_id, arm_id, waypoints, plants, pests)
        init = self._format_pddl_init(robot_id, arm_id, arm_state, current_wp, waypoints, connections, plants, pests, visited_waypoints=visited_waypoints, scanned_waypoints=scanned_waypoints)
        goal = self._format_pddl_goal(
            waypoints,
            plants,
            pests,
            visited_waypoints=visited_waypoints,
            scanned_waypoints=scanned_waypoints,
            infeasible_waypoints=infeasible_waypoints,
        )

        if not waypoints:
            raise RuntimeError("No waypoints found. Save waypoints or load demo data first.")

        content = f"""(define (problem greenhouse-generated-problem)
  (:domain greenhouse-domain)

  (:objects
{objects}
  )

  (:init
{init}
  )

  (:goal
    (and
{goal}
    )
  )
)
"""

        with open(output_file, "w") as f:
            f.write(content)

        self.get_logger().info(f"Wrote {output_file}")
        self.get_logger().info(f"Waypoints: {len(waypoints)}, connections: {len(connections)}, plants: {len(plants)}, pests: {len(pests)}")

    def run_planner(self, domain_file="DomainGreenhouse.pddl", problem_file="problem_generated.pddl"):
        self.get_logger().info("Running POPF planner...")

        popf_bin = os.environ.get("POPF_BIN", "/opt/ros/humble/lib/popf/popf")

        if not os.path.isfile(popf_bin):
            self.get_logger().error(f"POPF not found at: {popf_bin}. Change the path or install POPF planner.")
            return None

        env = os.environ.copy()
        env["LD_LIBRARY_PATH"] = "/opt/ros/humble/lib:" + env.get("LD_LIBRARY_PATH", "")

        command = [popf_bin, domain_file, problem_file]

        try:
            result = subprocess.run(
                command,
                capture_output=True,
                text=True,
                env=env,
                timeout=60,
            )
        except subprocess.TimeoutExpired:
            self.get_logger().error("POPF timed out.")
            return None
        except Exception as e:
            self.get_logger().error(f"Failed to run POPF: {e}")
            return None

        if result.stdout:
            self.get_logger().info(f"POPF planner output:\n{result.stdout}")

        if result.stderr:
            self.get_logger().warn(f"POPF planner stderr:\n{result.stderr}")

        if result.returncode != 0:
            self.get_logger().error(f"POPF failed with return code {result.returncode}")
            return None

        return result.stdout

def main(args=None):
    parser = argparse.ArgumentParser(description='PDDL planner node for gh_twin.')
    parser.add_argument('--domain-file', type=str, required=False, help='Path to existing domain.pddl file')
    parser.add_argument('--problem-file', type=str, required=False, help='Path to existing problem.pddl file')
    parser.add_argument('--planner-cmd', type=str, required=False, help='Optional PDDL planner command to execute during replan')
    parsed, unknown = parser.parse_known_args(args=args)

    rclpy.init(args=args)
    planner = PddlPlannerNode(
        domain_file=parsed.domain_file,
        problem_file=parsed.problem_file,
        planner_cmd=parsed.planner_cmd,
    )

    try:
        planner.connect_to_typedb()
        planner.generate_pddl_problem("problem_generated.pddl")
        planner.run_planner(domain_file="DomainGreenhouse.pddl", problem_file="problem_generated.pddl")
        rclpy.spin(planner)
    finally:
        planner.close()
        planner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
