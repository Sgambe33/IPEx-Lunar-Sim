"""
ROS node handling apriltag detections. Using one apriltag_ros for each camera.
Solves camera position by using PnP on detected and known tags.

Subscribes  /camera/<cam>/detections  apriltags detected
            /camera/<cam>/camera_info  camera info published by unity

Publishes   /localization/tag_pose  estimated rover position from tags 
"""
import math
import numpy as np
import rclpy
import tf2_ros
from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import Point, Pose, PoseWithCovarianceStamped, Quaternion
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Header
from .fiducials import TagMap, base_pose, camera_pose, pose_ambiguity, quat_to_rot, survey_order
from .site import config_path, load_json


class TagLocalizer(Node):
    def __init__(self) -> None:
        super().__init__("tag_localizer", automatically_declare_parameters_from_overrides=True)

        names = self.list_parameters([], 0).names
        p = {q.name: q.value for q in self.get_parameters(names)}

        if "max_range_m" not in p:
            raise RuntimeError("No tag parameters: start the node with config/mission.yaml (ros2 launch ipex_mission mission.launch.py)")
        
        self.p: dict = p
        self.tag_map: TagMap = TagMap(load_json(config_path("site_survey.json"))["fiducials"])
        self.info: dict[str, CameraInfo] = {}
        self.tf_buffer: tf2_ros.Buffer = tf2_ros.Buffer()
        self.tf_listener: tf2_ros.TransformListener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        for cam in p["cameras"]:
            self.create_subscription(CameraInfo, f"/camera/{cam}/camera_info", lambda msg, c=cam: self.info.__setitem__(c, msg), qos_profile_sensor_data)
            self.create_subscription(AprilTagDetectionArray, f"/camera/{cam}/detections", lambda msg, c=cam: self.on_detections(c, msg), 10)
        
        self.pub: Publisher = self.create_publisher(PoseWithCovarianceStamped, "/localization/tag_pose", 10)
        self.get_logger().info(f"Tag localizer: {len(self.tag_map.tags)} exported tags, cameras {p['cameras']}")

    def on_detections(self, cam, msg):
        """Callback for incoming detections."""
        if cam not in self.info:
            return
        dets = {d.id: survey_order(d.corners) for d in msg.detections if d.id in self.tag_map.tags}
        if not dets:
            return
        camera_info = self.info[cam]
        distortion = camera_info.d if len(camera_info.d) else None
        results = camera_pose(self.tag_map, dets, camera_info.k, distortion)
        if results is None:
            return
        R_mc, t_mc, rms = results
        
        try:
            tf = self.tf_buffer.lookup_transform(self.p["base_frame"], msg.header.frame_id, Time())
        except tf2_ros.TransformException:
            self.get_logger().warn(f"no TF {self.p['base_frame']} <- {msg.header.frame_id}", throttle_duration_sec=10.0)
            return
        
        q = tf.transform.rotation
        t = tf.transform.translation
        x, y, z, yaw = base_pose(R_mc, t_mc, quat_to_rot(q.x, q.y, q.z, q.w), np.array([t.x, t.y, t.z]))

        min_distance = min(float(np.linalg.norm(self.tag_map.center(i) - t_mc)) for i in dets)
        if min_distance > self.p["max_range_m"] or rms > self.p["max_rms_px"]:
            return
        if len(dets) == 1 and min_distance > self.p["single_tag_max_range_m"]:
            return
        if pose_ambiguity(self.tag_map, dets, camera_info.k, distortion) > self.p["max_ambiguity"]:
            return

        # build estimated pose msg with some uncertainty
        sigma_xy = self.p["sigma0_m"] + self.p["sigma_k"] * min_distance ** 2
        sigma_yaw = math.radians(self.p["sigma_yaw0_deg"]) + sigma_xy / max(min_distance, self.p["sigma_min_dist_m"])
        out = PoseWithCovarianceStamped(header=Header(stamp=msg.header.stamp, frame_id="map"))
        out.pose.pose = Pose(position=Point(x=float(x), y=float(y), z=float(z)), orientation=Quaternion(z=math.sin(yaw / 2), w=math.cos(yaw / 2)))
        cov = [0.0] * 36
        cov[0] = cov[7] = sigma_xy ** 2
        cov[14] = (2 * sigma_xy) ** 2
        cov[21] = cov[28] = cov[35] = sigma_yaw ** 2
        out.pose.covariance = cov
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = TagLocalizer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
