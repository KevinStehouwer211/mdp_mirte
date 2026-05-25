import argparse
import re
import subprocess
from pathlib import Path

import rclpy
from rclpy.node import Node

from gh_twin_data_storage.msg import Pest, Plant, Sensor


def _sanitize_name(prefix: str, identifier: int) -> str:
    return f"{prefix}-{identifier}".replace(' ', '_').replace('.', '_')


def _normalize_id(identifier) -> str:
    return str(identifier).strip().replace(' ', '_').replace('.', '_')


class PddlManagerNode(Node):
    def __init__(self, world_file=None, problem_file=None, planner_cmd=None):
        super().__init__('pddl_manager_node')

        self.world_file = Path(world_file) if world_file else None
        self.problem_file = Path(problem_file) if problem_file else None
        self.planner_cmd = planner_cmd
        self.world_text = ''
        self.problem_text = ''
        self.base_problem_text = ''
        self.plants = []
        self.pests = []
        self.sensors = []
        self.location_names = []

        self.create_subscription(Plant, 'plant_data', self.plant_callback, 10)
        self.create_subscription(Pest, 'pest_data', self.pest_callback, 10)
        self.create_subscription(Sensor, 'sensor_data', self.sensor_callback, 10)

        if self.world_file and self.problem_file:
            self.load_pddl_files(self.world_file, self.problem_file)

        self.get_logger().info('PDDL manager node initialized.')

    def load_pddl_files(self, world_file: Path, problem_file: Path):
        self.world_file = Path(world_file)
        self.problem_file = Path(problem_file)

        self.world_text = self.world_file.read_text(encoding='utf-8')
        self.problem_text = self.problem_file.read_text(encoding='utf-8')
        self.base_problem_text = self.problem_text

        self.get_logger().info(f'Loaded world file: {self.world_file}')
        self.get_logger().info(f'Loaded problem file: {self.problem_file}')

    def plant_callback(self, msg: Plant):
        record = {
            'id': _normalize_id(msg.id),
            'location': {'x': msg.location.x, 'y': msg.location.y},
            'color': msg.color,
            'height': float(msg.height),
            'is_flower': bool(msg.is_flower),
        }
        self.plants.append(record)
        self.get_logger().info(f'Received plant id={record["id"]} from typedb')

    def pest_callback(self, msg: Pest):
        record = {
            'id': _normalize_id(msg.id),
            'location': {'x': msg.location.x, 'y': msg.location.y},
            'sprayed': bool(msg.sprayed),
        }
        self.pests.append(record)
        self.get_logger().info(f'Received pest id={record["id"]} from typedb')

    def sensor_callback(self, msg: Sensor):
        readings = [
            {'type': r.type, 'value': float(r.value), 'unit': r.unit}
            for r in msg.readings
        ]
        record = {
            'id': _normalize_id(msg.id),
            'location': {'x': msg.location.x, 'y': msg.location.y},
            'readings': readings,
        }
        self.sensors.append(record)
        self.get_logger().info(f'Received sensor id={record["id"]} from typedb')

    def _extract_section(self, text: str, section_name: str):
        pattern = re.compile(rf"\(:{section_name}\b", re.IGNORECASE)
        match = pattern.search(text)
        if not match:
            return None, text

        start = match.start()
        depth = 0
        end = None
        for index in range(start, len(text)):
            char = text[index]
            if char == '(':
                depth += 1
            elif char == ')':
                depth -= 1
                if depth == 0:
                    end = index + 1
                    break

        if end is None:
            return None, text

        section = text[start:end]
        remaining = text[:start] + text[end:]
        return section, remaining

    def _replace_or_insert_section(self, text: str, section_name: str, new_section: str):
        section, remaining = self._extract_section(text, section_name)
        if section is not None:
            return text.replace(section, new_section, 1)

        # Insert after domain line if objects section missing
        domain_match = re.search(r"\(:domain\s+[^\s)]+\)", text, re.IGNORECASE)
        if section_name == 'objects' and domain_match:
            insert_at = domain_match.end()
            return text[:insert_at] + '\n  ' + new_section + text[insert_at:]

        # Insert before goal section if init section missing
        goal_match = re.search(r"\(:goal\b", text, re.IGNORECASE)
        if section_name == 'init' and goal_match:
            insert_at = goal_match.start()
            return text[:insert_at] + '  ' + new_section + '\n' + text[insert_at:]

        return text + '\n' + new_section

    def _format_pddl_objects(self):
        plant_names = [f'plant-{_normalize_id(item["id"])}' for item in self.plants]
        pest_names = [f'pest-{_normalize_id(item["id"])}' for item in self.pests]
        sensor_names = [f'sensor-{_normalize_id(item["id"])}' for item in self.sensors]
        location_names = [self._location_name(item["location"]) for item in self.plants + self.pests + self.sensors]
        self.location_names = sorted(set(location_names))

        sections = []
        if plant_names:
            sections.append(' '.join(plant_names) + ' - plant')
        if pest_names:
            sections.append(' '.join(pest_names) + ' - pest')
        if sensor_names:
            sections.append(' '.join(sensor_names) + ' - sensor')
        if self.location_names:
            sections.append(' '.join(self.location_names) + ' - location')

        if not sections:
            return '(:objects)\n'

        return '(:objects\n    ' + '\n    '.join(sections) + '\n)\n'

    def _location_name(self, location: dict) -> str:
        x = f'{location["x"]:.2f}'.replace('.', '_')
        y = f'{location["y"]:.2f}'.replace('.', '_')
        return f'loc-{x}-{y}'

    def _format_pddl_init(self):
        lines = []
        for plant in self.plants:
            object_name = f'plant-{plant["id"]}'
            location_name = self._location_name(plant['location'])
            lines.append(f'(plant {object_name})')
            lines.append(f'(at {object_name} {location_name})')
            lines.append(f'; color={plant["color"]} height={plant["height"]:.2f} flower={plant["is_flower"]}')

        for pest in self.pests:
            object_name = f'pest-{pest["id"]}'
            location_name = self._location_name(pest['location'])
            lines.append(f'(pest {object_name})')
            lines.append(f'(at {object_name} {location_name})')
            lines.append(f'; sprayed={pest["sprayed"]}')

        for sensor in self.sensors:
            object_name = f'sensor-{sensor["id"]}'
            location_name = self._location_name(sensor['location'])
            lines.append(f'(sensor {object_name})')
            lines.append(f'(at {object_name} {location_name})')
            for reading in sensor['readings']:
                lines.append(f'; sensor reading {reading["type"]}={reading["value"]}{reading["unit"]}')

        if not lines:
            return '(:init)\n'
        return '(:init\n    ' + '\n    '.join(lines) + '\n)\n'

    def update_problem_from_typedb(self):
        if not self.problem_file:
            self.get_logger().error('Problem file is not initialized. Cannot update problem from typedb.')
            return

        objects_section = self._format_pddl_objects()
        init_section = self._format_pddl_init()

        updated_text = self._replace_or_insert_section(self.base_problem_text, 'objects', objects_section)
        updated_text = self._replace_or_insert_section(updated_text, 'init', init_section)
        self.problem_text = updated_text
        self.get_logger().info('Updated problem text from typedb entries.')

    def save_problem_file(self, output_path=None):
        if not self.problem_text:
            self.get_logger().error('No problem text available to save.')
            return
        path = Path(output_path) if output_path else self.problem_file
        if path is None:
            self.get_logger().error('No target problem file path is set.')
            return
        path.write_text(self.problem_text, encoding='utf-8')
        self.get_logger().info(f'Saved updated problem file to {path}')

    def replan(self, output_problem_file=None, planner_command=None):
        self.update_problem_from_typedb()
        if output_problem_file:
            self.save_problem_file(output_problem_file)
        else:
            self.save_problem_file()

        planner = planner_command or self.planner_cmd
        if planner:
            self.get_logger().info(f'Running planner: {planner}')
            result = subprocess.run(planner, shell=True, capture_output=True, text=True)
            self.get_logger().info(f'Planner stdout:\n{result.stdout}')
            self.get_logger().info(f'Planner stderr:\n{result.stderr}')
            if result.returncode != 0:
                self.get_logger().error(f'Planner returned code {result.returncode}')
            return result.returncode

        self.get_logger().info('Replan completed by regenerating problem file. No external planner configured.')
        return 0


def main(args=None):
    parser = argparse.ArgumentParser(description='PDDL manager node for gh_twin.')
    parser.add_argument('--world-file', type=str, required=False, help='Path to existing world.pddl file')
    parser.add_argument('--problem-file', type=str, required=False, help='Path to existing problem.pddl file')
    parser.add_argument('--planner-cmd', type=str, required=False, help='Optional PDDL planner command to execute during replan')
    parsed, unknown = parser.parse_known_args(args=args)

    rclpy.init(args=args)
    node = PddlManagerNode(
        world_file=parsed.world_file,
        problem_file=parsed.problem_file,
        planner_cmd=parsed.planner_cmd,
    )

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
