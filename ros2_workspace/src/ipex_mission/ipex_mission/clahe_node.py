"""
ROS2 node that handles histogram equalization on images before handing them to apriltag detection node.
Uses cv_bridge package to easily convert ROS2 msg.Image to cv2 format.

Subscribes  /camera/<cam>/image_raw     mono8 images sent by Unity node
Publishes   /camera/<cam>/image_clahe   histogram equalized image with original header
"""
import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class ClaheNode(Node):
    def __init__(self) -> None:
        super().__init__("clahe", automatically_declare_parameters_from_overrides=True)

        names = self.list_parameters([], 0).names
        p = {q.name: q.value for q in self.get_parameters(names)}

        if "clip" not in p:
            raise RuntimeError("No CLAHE parameters: start the node with config/mission.yaml (ros2 launch ipex_mission mission_launch.py)")
        
        self.clahe: cv2.CLAHE = cv2.createCLAHE(clipLimit=p["clip"], tileGridSize=(p["tiles"], p["tiles"]))
        self.bridge: CvBridge = CvBridge()
        self.pubs: dict[str, Publisher] = {}

        for cam in p["cameras"]:
            self.pubs[cam] = self.create_publisher(Image, f"/camera/{cam}/image_clahe", qos_profile_sensor_data)
            self.create_subscription(Image, f"/camera/{cam}/image_raw", lambda msg, c=cam: self.on_image(c, msg), qos_profile_sensor_data)

    def on_image(self, cam, msg: Image):
        """When an image is received, its histogram is equalized and the image republished on a new topic."""
        equalized = self.clahe.apply(self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono8"))
        self.pubs[cam].publish(self.bridge.cv2_to_imgmsg(equalized, encoding="mono8", header=msg.header))


def main(args=None):
    rclpy.init(args=args)
    node = ClaheNode()
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
