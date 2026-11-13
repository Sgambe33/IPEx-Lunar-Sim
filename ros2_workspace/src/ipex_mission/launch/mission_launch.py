"""
ros2 launch ipex_mission mission.launch.py [strategy:=threshold|predictive] [localization:=tags|ground_truth] [rviz:=true]

Starts the following nodes:
  robot_localization  EKF: wheel odometry + apriltags
  clahe               histogram equalization of camera images         (localization:=tags)
  apriltag_ros        one apriltag_node per camera for tag detections (localization:=tags)
  tag_localizer       rover pose from tag detections                  (localization:=tags)
  mission             mission core node, handles logic and sends commands
  RViz2               optional
"""
import json
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, EqualsSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory("ipex_mission")
    cfg = os.path.join(share, "config", "mission.yaml")
    rviz_cfg = os.path.join(share, "rviz", "mission.rviz")

    with open(cfg, encoding="utf-8") as f:
        yaml_cfg = yaml.safe_load(f)

    with open(os.path.join(share, "config", "site_survey.json"), encoding="utf-8") as f:
        tags = json.load(f)["fiducials"]["tags"]
        
    common = yaml_cfg["/**"]["ros__parameters"]
    mission = yaml_cfg["mission"]["ros__parameters"]
    cameras = common["cameras"]
    clahe = yaml_cfg["clahe"]["ros__parameters"]
    image_topic = "image_clahe" if clahe["enabled"] else "image_raw"   # what apriltag_ros detects on

    args = [
        DeclareLaunchArgument("strategy", default_value=mission["strategy"]),
        DeclareLaunchArgument("max_mission_time", default_value=str(mission["max_mission_time"])),
        DeclareLaunchArgument("localization", default_value="tags", description="tags or ground_truth"),
        DeclareLaunchArgument("rviz", default_value="false", description="also open RViz2"),
    ]
    localization_is_tags = EqualsSubstitution(LaunchConfiguration("localization"), "tags")
    use_tags = IfCondition(localization_is_tags)
    use_ground_truth = UnlessCondition(localization_is_tags)

    nodes = []

    nodes.append(Node(
        package="ipex_mission", executable="mission", name="mission", output="screen",
        parameters=[{
            **common,
            **mission,
            "strategy": ParameterValue(LaunchConfiguration("strategy"), value_type=str),
            "max_mission_time": ParameterValue(LaunchConfiguration("max_mission_time"), value_type=float),
        }],
    ))

    for condition, extra in ((use_tags, []), (use_ground_truth, [yaml_cfg["ekf_ground_truth"]["ros__parameters"]])):
        nodes.append(Node(
            package="robot_localization", executable="ekf_node", name="ekf", output="screen",
            parameters=[cfg] + extra, remappings=[("odometry/filtered", "/localization/pose")], condition=condition,
        ))

    apriltag = dict(yaml_cfg["apriltag"]["ros__parameters"])

    for cam in cameras:
        cam_name = cam.replace("/", "_")
        nodes.append(Node(
            package="apriltag_ros", executable="apriltag_node", name=f"apriltag_{cam_name}", output="screen",
            parameters=[cfg, apriltag, {
                "tag.ids": [int(t["id"]) for t in tags],
                "tag.sizes": [float(t["size"]) for t in tags],
                "tag.frames": [f"{cam_name}_tag{int(t['id'])}" for t in tags],
            }],
            remappings=[("image_rect", f"/camera/{cam}/{image_topic}"), ("camera_info", f"/camera/{cam}/camera_info"), ("detections", f"/camera/{cam}/detections")],
            condition=use_tags,
        ))

    nodes.append(Node(
        package="ipex_mission", executable="tag_localizer", name="tag_localizer", output="screen",
        parameters=[cfg], condition=use_tags,
    ))

    if clahe["enabled"]:
        nodes.append(Node(
            package="ipex_mission", executable="clahe", name="clahe", output="screen",
            parameters=[cfg], condition=use_tags,
        ))

    nodes.append(Node(
        package="rviz2", executable="rviz2", name="rviz2", arguments=["-d", rviz_cfg], output="screen",
        parameters=[cfg], condition=IfCondition(LaunchConfiguration("rviz")),
    ))
    
    return LaunchDescription(args + nodes)