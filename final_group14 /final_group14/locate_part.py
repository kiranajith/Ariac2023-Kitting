import rclpy
from rclpy.node import Node
import PyKDL
from ariac_msgs.msg import AdvancedLogicalCameraImage
from geometry_msgs.msg import Pose
from rclpy.qos import QoSProfile, ReliabilityPolicy

custom_qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

class Part:
    def __init__(self, part_type, color, world_pose):
        self.part_type = part_type
        self.color = color
        self.world_pose = world_pose

    def __str__(self):
        return (f"part poses: \n- part:\n  , Color: {self.color}\n,  type: {self.part_type}\n   pose:\n    \n "
                f"   position:\n    x:{self.world_pose.position.x}\n    y:{self.world_pose.position.y}\n    z:{self.world_pose.position.z} "
                f"   \nOrientation:\n x:{self.world_pose.orientation.x}\n    y:{self.world_pose.orientation.y}\n    z:{self.world_pose.orientation.z}\n    w:{self.world_pose.orientation.w}\n")

class MyNode(Node):
    """A ROS2 node that subscribes to the /ariac/sensors/left_bins_camera/image and /ariac/sensors/right_bins_camera/image topic and processes retrieving 


    Args:
        Node 
    """
    def __init__(self, node_name):
        """ initilaises the node with the given name 

        Args:
            node_name (_type_): name of the node 
        """
        super().__init__(node_name)

        # Subscribers
        self.left_bins_camera_sub = self.create_subscription(AdvancedLogicalCameraImage, '/ariac/sensors/left_bins_camera/image', self._left_bins_camera_callback, qos_profile=custom_qos_profile)
        self.right_bins_camera_sub = self.create_subscription(AdvancedLogicalCameraImage, '/ariac/sensors/right_bins_camera/image', self._right_bins_camera_callback, qos_profile=custom_qos_profile)
        
        self.flag1 = False
        self.flag2 = False

    def _multiply_pose(self, pose1: Pose, pose2: Pose) -> Pose:
        """ function to perfom use the multiplication method to get KDL frame 

        Args:
            pose1 (Pose): inittial frame
            pose2 (Pose): target frame

        Returns:
            Pose: KDL frame
        """
        orientation1 = pose1.orientation
        frame1 = PyKDL.Frame(
            PyKDL.Rotation.Quaternion(orientation1.x, orientation1.y, orientation1.z, orientation1.w),
            PyKDL.Vector(pose1.position.x, pose1.position.y, pose1.position.z))

        orientation2 = pose2.orientation
        frame2 = PyKDL.Frame(
            PyKDL.Rotation.Quaternion(orientation2.x, orientation2.y, orientation2.z, orientation2.w),
            PyKDL.Vector(pose2.position.x, pose2.position.y, pose2.position.z))

        frame3 = frame1 * frame2

        pose = Pose()
        pose.position.x = frame3.p.x()
        pose.position.y = frame3.p.y()
        pose.position.z = frame3.p.z()

        q = frame3.M.GetQuaternion()
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]
        return pose

    def _left_bins_camera_callback(self, message):
        """_ function to perform callback to the subscriber

        Args:
            message (_type_): messages which are are subscribed from the/ariac/sensors/left_bins_camera/image topic
        """
        if not self.flag1:
            self.get_logger().info('Output from left_bins_camera')
            for part_info in message.part_poses:
                world_pose = self._multiply_pose(message.sensor_pose, part_info.pose)
                part = Part(part_info.part.type, part_info.part.color, world_pose)
                # self.get_logger().info(f"Detected part in left_bins_camera:\n{part}")
                print(part)
            self.flag1 = True

    # def _right_bins_camera_callback(self, message):
    #     """_ function to perform callback to the subscriber

    #     Args:
    #         message (_type_): messages which are are subscribed from the/ariac/sensors/left_bins_camera/image topic

    #     """
    #     if not self.flag2:
    #         self.get_logger().info('Output from right_bins_camera')
    #         for part_info in message.part_poses:
    #             world_pose = self._multiply_pose(message.sensor_pose, part_info.pose)
    #             part = Part(part_info.part.type, part_info.part.color, world_pose)
    #             # self.get_logger().info(f"Detected part in right_bins_camera:\n{part}")
    #             print(part)
    #         self.flag2 = True

def main(args=None):
    """ creates an instance of the node class 

    Args:
        args (_type_, optional): None
    """

    rclpy.init(args=args)
    node = MyNode('part_detector')
    rclpy.spin(node)
    node.destroy_node()

