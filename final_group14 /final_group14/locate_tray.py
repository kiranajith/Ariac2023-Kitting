
import rclpy
from rclpy.node import Node
import PyKDL
from ariac_msgs.msg import AdvancedLogicalCameraImage
from geometry_msgs.msg import Pose
from rclpy.qos import QoSProfile, ReliabilityPolicy
from custom_interfaces.msg import TrayPose

custom_qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

class Tray:
    def __init__(self, tray_id, world_pose, sensor_pose):
        self.tray_id = tray_id
        self.world_pose = world_pose
        self.sensor_pose = sensor_pose



    def __str__(self):
        world_pose_x = self.world_pose.position.x
        world_pose_y = self.world_pose.position.y
        world_pose_z = self.world_pose.position.z

        sensor_pose_x = self.sensor_pose.position.x
        sensor_pose_y = self.sensor_pose.position.y
        sensor_pose_z = self.sensor_pose.position.z
        
        world_orientation_x = self.world_pose.orientation.x
        world_orientation_y = self.world_pose.orientation.y
        world_orientation_z = self.world_pose.orientation.z

        sensor_orientation_x = self.sensor_pose.orientation.x
        sensor_orientation_y = self.sensor_pose.orientation.y
        sensor_orientation_z = self.sensor_pose.orientation.z


        return (f"part_poses: []\ntray_poses: \n- id: {self.tray_id}\n  pose:\n   position: \n"
                f"    x: {world_pose_x}\n    y: {world_pose_y}\n    z: {world_pose_z}\n"   
                f"   orientation:\n   x: {world_orientation_x}\n   y: {world_orientation_y}\n   z: {world_orientation_z}"
                f"\nSensor pose: \n position\n   x: {sensor_pose_x} \n   y: {sensor_pose_y} \n   z: {sensor_pose_z}\n"
                f"   orientation:\n   x: {sensor_orientation_x}\n   y: {sensor_orientation_y}\n   z: {sensor_orientation_z}" )

class MyNode(Node):
    """ a node that subscribers to the topics /ariac/sensors/table1_camera/image and /ariac/sensors/table2_camera/ and process retrieving of tray poses

    Args:
        Node
    """
    def __init__(self, node_name):
        """  initilaises the node with the given name 

        Args:
            node_name (_type_): name of the node
        """
        super().__init__(node_name)

        self.received_table1 = False
        self.received_table2 = False

        self.subscriber1 = self.create_subscription(AdvancedLogicalCameraImage, '/ariac/sensors/table1_camera/image', self._subscriber1_callback, qos_profile=custom_qos_profile)
        self.subscriber2 = self.create_subscription(AdvancedLogicalCameraImage, '/ariac/sensors/table2_camera/image', self._subscriber2_callback, qos_profile=custom_qos_profile)
        self.publisher = self.create_publisher(TrayPose, 'tray_poses_topic', 10)
              
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
   
    
    def _subscriber1_callback(self, message):
        """ function to perform callback to the subscriber

        Args:
            message (_type_): messagees which are subscibed from the /ariac/sensors/table1_camera/image topic
        """
        if not self.received_table1:

            for tray_pose in message.tray_poses:
                world_pose = self._multiply_pose(message.sensor_pose, tray_pose.pose)
                detected_tray = Tray(tray_pose.id, world_pose, message.sensor_pose)
                print(detected_tray)
                world_pose = self._multiply_pose(message.sensor_pose, tray_pose.pose)
                tray_pose_msg = TrayPose()
                tray_pose_msg.world_pose = world_pose
                tray_pose_msg.sensor_pose = message.sensor_pose
                tray_pose_msg.id = tray_pose.id
                self.publisher.publish(tray_pose_msg)

            self.received_table1 = True

    def _subscriber2_callback(self, message):
        """ function to perform callback to the subscriber

        Args:
            message (_type_): messages which are are subscribed from the/ariac/sensors/table2_camera/image topic
        """
        if not self.received_table2:
           
            for tray_pose in message.tray_poses:
                world_pose = self._multiply_pose(message.sensor_pose, tray_pose.pose)
                detected_tray = Tray(tray_pose.id, world_pose, message.sensor_pose)
                print(detected_tray)
                world_pose = self._multiply_pose(message.sensor_pose, tray_pose.pose)
                tray_pose_msg = TrayPose()
                tray_pose_msg.world_pose = world_pose
                tray_pose_msg.sensor_pose = message.sensor_pose
                tray_pose_msg.id = tray_pose.id
                self.publisher.publish(tray_pose_msg)

            self.received_table2 = True


def main(args=None):
    """ creates an instance of the node class 

    Args:
        args (_type_, optional): None
    """
    rclpy.init(args=args)
    node = MyNode('get_pose')
    rclpy.spin(node)
    node.destroy_node()








