"""
ROS2 node that handles mission logic.

Subscribes  /localization/pose   nav_msgs/Odometry from robot_localization (for robot_localization)
            /rover/battery, /rover/excavator                               (the rover's battery and drum loads/stats)
            /isru/hopper                                                   (the hopper's contents in kg)
            /obstacles                                                     (contains known obstacles like infrastructure and boulders)

Publishes   /cmd_vel                                    (for issuing driving commands)
            /excavation/{arm,drum}_{front,rear}_cmd     (for controlling both arms and drums)
            /mission/markers                            (contains all generated markers like dig area...)
            /mission/path                               (contains the path the rover is currently following)
"""
import math
import rclpy
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, Twist, Vector3
from ipex_interfaces.msg import BatteryState, ExcavatorState, HopperState
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosidl_runtime_py import message_to_ordereddict
from std_msgs.msg import ColorRGBA, Float32, Header
from visualization_msgs.msg import Marker, MarkerArray
from .geometry import yaw_from_quat
from .mission_core import Mission
from .site import apply_site_survey, config_path, load_json


class MissionNode(Node):
    STATE_TOPICS = {"/rover/battery": BatteryState, "/rover/excavator": ExcavatorState, "/isru/hopper": HopperState}

    def __init__(self) -> None:
        super().__init__("mission", automatically_declare_parameters_from_overrides=True)

        names = self.list_parameters([], 0).names
        params = {p.name: p.value for p in self.get_parameters(names) if p.name != "use_sim_time"}

        if "strategy" not in params:
            raise RuntimeError("No mission parameters: start the node with config/mission.yaml (ros2 launch ipex_mission mission_launch.py)")

        self.params: dict = apply_site_survey(params, load_json(config_path("site_survey.json")))
        self.pose_valid: bool = False  # pose is invalid until first tag detection
        self.dem_info: dict = load_json(config_path("terrain_heights.json"))
        self.mission: Mission = self.new_mission()

        latched_profile = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Odometry, "/localization/pose", self.on_pose, QoSProfile(depth=20))

        self.state_parts: dict[str, dict] = {}  # stores each state topic last msg received
        for topic, msg_type in self.STATE_TOPICS.items():
            self.create_subscription(msg_type, topic, lambda msg, t=topic: self.on_state(t, msg), QoSProfile(depth=10))
        self.create_subscription(MarkerArray, "/obstacles", self.on_obstacles, latched_profile)

        self.pub_cmd: Publisher = self.create_publisher(Twist, "/cmd_vel", QoSProfile(depth=10))
        self.pub_impl: dict[str, Publisher] = {k: self.create_publisher(Float32, f"/excavation/{k}_cmd", QoSProfile(depth=10)) for k in ("arm_front", "arm_rear", "drum_front", "drum_rear")}
        self.pub_markers: Publisher = self.create_publisher(MarkerArray, "/mission/markers", latched_profile)
        self.pub_path: Publisher = self.create_publisher(Path, "/mission/path", QoSProfile(depth=1))

        self.create_timer(0.1, self.tick)             # every 0.1s check mission status and act accordingly
        self.create_timer(2.0, self.publish_markers)  # every 2s publish markers
        self.get_logger().info(f"Mission '{params['strategy']}'")

    def new_mission(self):
        """One Mission per Unity Play session."""
        return Mission(self.params, self.dem_info, clock=self.sim_now, log=self.get_logger().info, publish_path=self.publish_path)

    def sim_now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_pose(self, msg):
        if not self.pose_valid:
            if msg.pose.covariance[0] > self.params["pose_ready_std_m"] ** 2:
                return
            self.pose_valid = True
        q = msg.pose.pose.orientation
        self.mission.set_pose(msg.pose.pose.position.x, msg.pose.pose.position.y, yaw_from_quat(q.x, q.y, q.z, q.w))

    def on_state(self, topic, msg):
        self.state_parts[topic] = dict(message_to_ordereddict(msg))
        if len(self.state_parts) == len(self.STATE_TOPICS):
            state = {}
            for part in self.state_parts.values():
                state.update(part)
            self.mission.set_state(state)

    def on_obstacles(self, msg):
        rocks = []
        lander = None
        infra = []
        for m in msg.markers:
            item = (m.pose.position.x, m.pose.position.y, 0.5 * max(m.scale.x, m.scale.y))
            if m.ns == "lander":
                lander = item
            elif m.ns == "hopper":
                infra.append(item)
            else:
                rocks.append(item)
        self.mission.set_obstacles(rocks, lander, infra)

    def tick(self):
        m = self.mission
        if m.finished and m.end_reason.startswith("simulation restarted"):
            self.get_logger().info("Unity restarted Play: starting a new run")
            self.mission = self.new_mission()
            self.pose_valid = False
        self.mission.step()
        m = self.mission
        self.pub_cmd.publish(Twist(linear=Vector3(x=m.cmd_v), angular=Vector3(z=m.cmd_w)))
        for k, pub in self.pub_impl.items():
            pub.publish(Float32(data=float(m.impl[k])))

    def publish_path(self, pts):
        """Publishes a path made of pts."""
        msg = Path(header=Header(frame_id="map", stamp=self.get_clock().now().to_msg()))
        for x, y in pts:
            msg.poses.append(PoseStamped(header=msg.header, pose=Pose(position=Point(x=float(x), y=float(y)))))
        self.pub_path.publish(msg)

    def marker(self, ns, id, mtype, stamp):
        """Create an instance of Marker given the parameters."""
        return Marker(header=Header(frame_id="map", stamp=stamp), ns=ns, id=id, type=mtype, action=Marker.ADD)

    def publish_markers(self):
        """Publishes a MarkerArray containing all mission markers: dig area, lanes..."""
        site = self.mission.site
        arr = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        id = 0
        if site.area is not None:
            m = self.marker("dig_area", id, Marker.LINE_STRIP, stamp)
            id += 1
            m.scale.x = 0.05
            for cx, cy in site.area.corners() + site.area.corners()[:1]:
                m.points.append(Point(x=float(cx), y=float(cy), z=0.03))
            m.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.9)
            arr.markers.append(m)
        for lane in site.lanes:
            m = self.marker("lanes", id, Marker.LINE_STRIP, stamp)
            id += 1
            m.scale.x = 0.08
            for s in (0.0, lane.length):
                px, py = lane.point(s)
                m.points.append(Point(x=float(px), y=float(py), z=0.02))
            used = min(1.0, (lane.passes + lane.next_s / max(lane.length, 1e-3)) / max(1, site.max_passes))
            if lane.blocked:
                m.color = ColorRGBA(r=0.6, g=0.1, b=0.1, a=0.8)
            else:
                m.color = ColorRGBA(r=0.3 + 0.7 * used, g=0.5, b=1.0 - used, a=0.8)
            arr.markers.append(m)
        h = site.hopper
        if h is not None:
            m = self.marker("hopper", id, Marker.CUBE, stamp)
            id += 1
            m.pose.position = Point(x=float(h.x), y=float(h.y), z=float(h.height) / 2)
            m.pose.orientation = Quaternion(z=math.sin(h.yaw / 2), w=math.cos(h.yaw / 2))
            m.scale = Vector3(x=float(h.length + 2 * h.wall), y=float(h.width + 2 * h.wall), z=float(h.height))
            m.color = ColorRGBA(r=1.0, g=0.6, b=0.1, a=0.9)
            arr.markers.append(m)
            t = self.marker("hopper", id, Marker.TEXT_VIEW_FACING, stamp)
            id += 1
            t.pose.position = Point(x=float(h.x), y=float(h.y), z=0.6)
            t.scale.z = 0.3
            t.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            t.text = f"hopper {self.mission.state.get('delivered_kg', 0.0):.0f} kg"
            arr.markers.append(t)

        d = self.marker("antenna", id, Marker.CYLINDER, stamp)
        d.pose.position = Point(x=float(site.antenna[0]), y=float(site.antenna[1]), z=0.02)
        d.scale = Vector3(x=0.4, y=0.4, z=0.04)
        d.color = ColorRGBA(r=0.2, g=0.8, b=1.0, a=0.8)
        arr.markers.append(d)
        self.pub_markers.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        m = node.mission
        if not m.finished and m.t_start is not None:
            m.finish("interrupted")
            node.tick()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
