from pathlib import Path

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    package_dir = Path(get_package_share_directory("flower_perception"))

    model_path = package_dir / "models" / "best.pt"
    rviz_config = package_dir / "rviz" / "FP_rviz.rviz"

    # No rosbag here: nodes run against the live robot, so use the real clock.
    return LaunchDescription(
        [
            # RViz with config
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", str(rviz_config)],
            ),

            # RQT
            Node(
                package="rqt_gui",
                executable="rqt_gui",
                name="rqt",
                output="screen",
            ),

            # Flower detector + tracker (assigns a stable track id per flower)
            Node(
                package="flower_perception",
                executable="flower_detector_tracker",
                name="flower_detector_tracker",
                output="screen",
                parameters=[
                    {
                        "model_path": str(model_path),
                        "image_topic": "/gripper_camera/image_raw/compressed",
                        "detections_topic": "/flower_perception",
                        "debug_image_topic": "/flower_perception/debug_image",
                        "confidence_threshold": 0.43,
                    }
                ],
            ),

            # Two-view triangulation: call /flower_triangulation/snapshot twice
            # (move the arm between calls) to get x,y,z per flower in odom.
            Node(
                package="flower_perception",
                executable="flower_triangulation",
                name="flower_triangulation",
                output="screen",
                parameters=[
                    {
                        "detections_topic": "/flower_perception",
                        "camera_frame": "wrist_camera",
                        "base_frame": "odom",
                        "min_view_separation": 0.05,   # m; store a view every 5 cm of camera travel
                        "min_views": 2,                # min rays before a flower is published
                    }
                ],
            ),

            # Project the triangulated 3-D detections back onto the debug image
            # as wireframe 3-D boxes, to visually check the triangulation.
            Node(
                package="flower_perception",
                executable="flower_bbox_projection",
                name="flower_bbox_projection",
                output="screen",
                parameters=[
                    {
                        "image_topic": "/gripper_camera/image_raw/compressed",
                        "detections_2d_topic": "/flower_perception",
                        "detections_3d_topic": "/flower_triangulation/detections",
                        "output_image_topic": "/flower_perception/debug_image/bbox_projected",
                        "camera_frame": "wrist_camera",
                        "base_frame": "odom",
                    }
                ],
            ),
        ]
    )
