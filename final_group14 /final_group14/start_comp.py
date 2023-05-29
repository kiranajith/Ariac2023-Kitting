import PyKDL
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_srvs.srv import Trigger
from ariac_msgs.msg import CompetitionState
from geometry_msgs.msg import Pose
from ariac_msgs.msg import Part
from ariac_msgs.msg import Order 
from rclpy.qos import QoSProfile, ReliabilityPolicy

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from ariac_msgs.srv import ChangeGripper, VacuumGripperControl, MoveAGV
from competitor_interfaces.msg import Robots as RobotsMsg
from ariac_msgs.msg import AdvancedLogicalCameraImage

from tf_transformations import quaternion_from_euler
from competitor_interfaces.srv import (
    EnterToolChanger, ExitToolChanger, PickupTray, MoveTrayToAGV, PlaceTrayOnAGV,
    RetractFromAGV, PickupPart, MovePartToAGV, PlacePartInTray
)
from custom_interfaces.msg import SavedOrder,PartQuantity,TrayPose
custom_qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)


# ---Class Declarations---

class orderInfo:
    """
    A class to store and display the details of an ARIAC order.
    
    """

    def __init__(self, id, type, priority, kitting_task) -> None:
        """
        Initialize class variables.

        Args:
            id (int): Order ID - a unique alphanumeric identifier.
            type (str): Order type.
            priority (int): Order priority.
            kitting_task (KittingTask): Information about the kitting task associated with the order.
        """
        self.id  = id
        self.type = type
        self.priority = priority
        self.kitting_task = kitting_task
    

        self.part_color_selected = None
        self.part_type_seleted = None
        self.agv_number = None
        self.tray_id = None
        self.destination = "Warehouse"

        
    def __str__(self) -> str:
        """function to override the print statement for an object 

        Returns:
            str: a string which prints the class attributes
        """
        kitting_task_str = f' agv_number: {self.kitting_task.agv_number}\n  tray_id: {self.kitting_task.tray_id}\n  destination: {self.kitting_task.destination}\n  parts:\n'
        self.agv_number= self.kitting_task.agv_number
        self.tray_id = self.kitting_task.tray_id
        for part in self.kitting_task.parts:
            if part.part.color == 0:
               self.part_color_selected= 'red'
            elif part.part.color == 1:
                self.part_color_selected = 'green'
            elif part.part.color == 2:
                self.part_color_selected = 'blue'
            elif part.part.color == 3:
                self.part_color_selected = 'orange'
            elif part.part.color == 4:
                self.part_color_selected = 'purple'

            if part.part.type == 10:
                self.part_type_seleted = 'battery'
            elif part.part.type == 11:
                self.part_type_seleted = 'pump'
            elif part.part.type == 12:
                self.part_type_seleted = 'sensor'
            elif part.part.type == 13:
                self.part_type_seleted = 'regulator'
            else:
                self.part = None
            # kitting_task_str += f' - part:\n   color: {part.part.color}\n   type: {part.part.type}\n   quadrant: {part.quadrant}\n'
            kitting_task_str += f' - part:\n   color: {self.part_color_selected}\n   type: {self.part_type_seleted}\n   quadrant: {part.quadrant}\n'


        return f'PROCESSED ORDER\n------------------\nid : {self.id}\ntype : {self.type}\npriority : {self.priority}\nkitting_task :\n {kitting_task_str}'



class Tray:
    """
        A class to store and display the details of the trays detected by the table cameras

    """
    def __init__(self, tray_id, world_pose, sensor_pose):
        """funcito to initiaise the class members

        Args:
            tray_id (int): tray id 
            world_pose ( stores the pose of the tray in the world coordinate frame
            sensor_pose : stores the pose of the tray in the camera coordinate frame 
        """
        self.tray_id = tray_id
        self.world_pose = world_pose
        self.sensor_pose = sensor_pose

    def __str__(self):
        """
        function to override the print function for the instances created for this class

        Returns:
            str: prints the values stored in the class attributes 
        """
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

class Part:
    """   
    A class to store and display the details of the parts detected by the bin cameras

    """
    def __init__(self, part_type, color, world_pose):
        """
        function to initialise the class attributes
        """
        self.part_type = part_type
        self.color = color
        self.world_pose = world_pose

    def __str__(self):
        """
        function to override the print function for the instances created for this class


        Returns:
            str: prints the values stored in the class attributes 
        """
        return (f"part poses: \n- part:\n  , Color: {self.color}\n,  type: {self.part_type}\n   pose:\n    \n "
                f"   position:\n    x:{self.world_pose.position.x}\n    y:{self.world_pose.position.y}\n    z:{self.world_pose.position.z} "
                f"   \nOrientation:\n x:{self.world_pose.orientation.x}\n    y:{self.world_pose.orientation.y}\n    z:{self.world_pose.orientation.z}\n    w:{self.world_pose.orientation.w}\n")


class RobotCommanderInterface(Node):
    """
    
    """
    def __init__(self,node_name):
        """ to initialise member attributes

        Args:
            node_name (Node): declaring the node 
        """
        super().__init__(node_name)

       #---- class attributes ----#

        
        self.order_received = False
        self.received_table1 = False
        self.received_table2 = False

        self.kit_completed = False
        self.competition_started = False
        self.competition_state = None

        self.TRAY_ID = None
        self.AGV_NUMBER = None
        self.target_tray_pose = None
        self.QUADRANT = []
        self.bin = []
        self.PART_TYPES =[]
        self.PART_COLORS =[]

        self.flag1 = False
        self.flag2 = False
        
        self.parts_list = []

        self.tray_pose_dict = {}  # Create an empty dictionary to store the tray poses

 
        self.orders = {}
        self.part_ids = {
            "battery": 10,
            "pump": 11,
            "sensor": 12,
            "regulator": 13,
        }

        self.color_ids = {
            "red": 0,
            "green": 1,
            "blue": 2,
            "orange": 3,
            "purple": 4,
        }
        self.color = None
        self.part = None
    

       #---- Declaring parameters ---#

        sim_time = Parameter(
            "use_sim_time",
            rclpy.Parameter.Type.BOOL,
            True
        )
        self.set_parameters([sim_time])
        self.get_logger().info('node created')
        
        self.declare_parameter('order_id', '1')
        self.order_id = self.get_parameter('order_id').value


        #---- Declaring callback groups ----#


        timer_group = MutuallyExclusiveCallbackGroup()
        service_group = MutuallyExclusiveCallbackGroup()

        #---- Declaring subscribers ----#

        self.subscriber = self.create_subscription(Order, '/ariac/orders', self._order_callback, qos_profile=10)

        self.subscriber1 = self.create_subscription(AdvancedLogicalCameraImage, '/ariac/sensors/table1_camera/image', self._subscriber1_callback, qos_profile=custom_qos_profile)
        self.subscriber2 = self.create_subscription(AdvancedLogicalCameraImage, '/ariac/sensors/table2_camera/image', self._subscriber2_callback, qos_profile=custom_qos_profile)

        self.left_bins_camera_sub = self.create_subscription(AdvancedLogicalCameraImage, '/ariac/sensors/left_bins_camera/image', self._left_bins_camera_callback, qos_profile=custom_qos_profile)
        self.right_bins_camera_sub = self.create_subscription(AdvancedLogicalCameraImage, '/ariac/sensors/right_bins_camera/image', self._right_bins_camera_callback, qos_profile=custom_qos_profile)
 


        self.create_subscription(CompetitionState, '/ariac/competition_state',
                                 self._competition_state_cb, 1)
        self.robot_action_timer = self.create_timer(1, self._robot_action_timer_callback,
                                                     callback_group=timer_group)

        self.start_competition_client = self.create_client(Trigger, '/ariac/start_competition',callback_group=service_group)

        # if self.competition_state == CompetitionState.READY and not self.competition_started:
        self.start_competition()


        #--- service client declaration ---#

        self.move_floor_robot_home_client = self.create_client(
            Trigger, '/competitor/floor_robot/go_home',callback_group=service_group) # client of the 'go_home' service used to move the robot to its home configguration

        self.goto_tool_changer_client = self.create_client(
            EnterToolChanger, '/competitor/floor_robot/enter_tool_changer',callback_group=service_group) # client of the 'enter_tool_changer' service used to direct the robot to the tool changer 

        self.exit_tool_changer_client = self.create_client(
            ExitToolChanger, '/competitor/floor_robot/exit_tool_changer',callback_group=service_group) # client of the 'exit_tool_changer' service used to direct the robot to exit the tool changer 

        self.pickup_tray_client = self.create_client(
            PickupTray, '/competitor/floor_robot/pickup_tray',callback_group=service_group) # client of the 'pickup_changer' service used to direct the robot to pickup the tray 

        self.move_tray_to_agv_client = self.create_client(
            MoveTrayToAGV, '/competitor/floor_robot/move_tray_to_agv',callback_group=service_group) # client of the 'move_tray_to_agv' service used to direct the robot to move to the designated  AGV

        self.place_tray_on_agv_client = self.create_client(
            PlaceTrayOnAGV, '/competitor/floor_robot/place_tray_on_agv',callback_group=service_group)# client of the 'place_tray_on _AGV' service used to direct the robot to place the tray on the AGV 

        self.retract_from_agv_client = self.create_client(
            RetractFromAGV, '/competitor/floor_robot/retract_from_agv',callback_group=service_group) # client of the 'reatrcact_from_AGV' service used to direct the robot to retract from the AGV 

        self.pickup_part_client = self.create_client(
            PickupPart, '/competitor/floor_robot/pickup_part',callback_group=service_group) # client of the 'pickup_part' service used to direct the robot to pickup a part 

        self.move_part_to_agv_client = self.create_client(
            MovePartToAGV, '/competitor/floor_robot/move_part_to_agv',callback_group=service_group) # client of the 'move_part_to_agv' service used to direct the robot to move the picked up part to the AGV 

        self.place_part_in_tray_client = self.create_client(
            PlacePartInTray, '/competitor/floor_robot/place_part_in_tray',callback_group=service_group) # client of the 'move_part_to_agv' service used to direct the robot to move the picked up part to the AGV 

        self.lock_tray_client = self.create_client(
            Trigger, '/ariac/agv1_lock_tray',callback_group=service_group) # client of the 'agvX_lock_tray' service used to lock the AGV 

        self.move_agv_client = self.create_client(
            MoveAGV, '/ariac/move_agv4',callback_group=service_group) # client of the 'move_aagvX' service used to move the agv to the warehouse
        self.change_gripper_client = self.create_client(
            ChangeGripper, '/ariac/floor_robot_change_gripper',callback_group=service_group)

        self.enable_gripper_client = self.create_client(
            VacuumGripperControl, '/ariac/floor_robot_enable_gripper',callback_group=service_group) # client of the 'floor_robot_enable_gripper' service used to enable/disable the gripper
        self.get_logger().info('Service clients Declared')


    def _order_callback(self, message):
        """
        Process an incoming message from the '/ariac/orders' topic.

        """
        self.get_logger().info('Order Received')
        order_id = message.id
        order_type = message.type
        order_priority = message.priority
        order_kitting_task = message.kitting_task

        order_info = orderInfo(order_id, order_type, order_priority, order_kitting_task)

        # Store the order in a dictionary
        self.orders[order_id] = order_info

        # Check if the received order matches the specified order_id from the parameter
        if order_id == self.order_id:
            
            self.get_logger().info(f'Processing order with ID: {self.order_id}')
            print(self.orders[self.order_id])
            print(order_info)
            self.AGV_NUMBER = order_info.agv_number
            self.TRAY_ID = order_info.tray_id
            for part in order_kitting_task.parts:
                self.QUADRANT.append(part.quadrant)

            self.order_received = True
        

        # else:
        #     self.get_logger().info(f'Storing order with ID: {order_id}')
        

    
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
        """
        Process an incoming message from the '/ariac/AdvancedLogicalCameraImage' topic.

        """
        # print(f'tray calllback, tray_ID = {self.TRAY_ID}')
        if not self.received_table1:
            for tray_pose in message.tray_poses:
                world_pose = self._multiply_pose(message.sensor_pose, tray_pose.pose)
                detected_tray = Tray(tray_pose.id, world_pose, message.sensor_pose)
                self.tray_pose_dict[tray_pose.id] = world_pose  
                print(detected_tray)
                print(f'tray_pose.id={tray_pose.id}')
                if tray_pose.id == self.TRAY_ID:
                    self.target_tray_pose = world_pose
                self.received_table1 = True

    def _subscriber2_callback(self, message):
        if not self.received_table2:
            for tray_pose in message.tray_poses:
                world_pose = self._multiply_pose(message.sensor_pose, tray_pose.pose)
                detected_tray = Tray(tray_pose.id, world_pose, message.sensor_pose)
                self.tray_pose_dict[tray_pose.id] = world_pose 
                if tray_pose.id == self.TRAY_ID:
                    self.target_tray_pose = world_pose
                self.received_table2 = True



    def _left_bins_camera_callback(self, message):
        if not self.flag1:
            self.get_logger().info('Output from left_bins_camera')
            for part_info in message.part_poses:
                world_pose = self._multiply_pose(message.sensor_pose, part_info.pose)
                part = Part(part_info.part.type, part_info.part.color, world_pose)
                print(f"Adding part from left_bins_camera: {part}")

                # print(part)
                self.parts_list.append(part)
                self.PART_TYPES.append(part_info.part.type)
                self.PART_COLORS.append(part_info.part.color)
                self.bin.append("left_bins")
                self.flag1 = True

    def _right_bins_camera_callback(self, message):
        if not self.flag2:
            self.get_logger().info('Output from right_bins_camera')
            for part_info in message.part_poses:
                world_pose = self._multiply_pose(message.sensor_pose, part_info.pose)
                part = Part(part_info.part.type, part_info.part.color, world_pose)

                # print(part)
                self.parts_list.append(part)
                self.PART_TYPES.append(part_info.part.type)
                self.PART_COLORS.append(part_info.part.color)
                self.bin.append("right_bins")
                self.flag2 = True



    def _competition_state_cb(self, msg: CompetitionState):
        # print('comp callback function')
        self.competition_state = msg.competition_state

    def _robot_action_timer_callback(self):
        '''
        Callback for the timer that triggers the robot actions

        '''

        # self.start_competition()

        if self.competition_state == CompetitionState.READY and not self.competition_started:
            self.start_competition()

        if not self.order_received:
            return

        # exit the callback if the kit is completed
        if self.kit_completed:
            return

        if not self.received_table1 :
            return
        if not self.received_table2 :
            return
        
        if not self.flag1 :
            return
        
        if  not self.flag2 :
            return
        
        #-----SET VARIABLES------


        tray_id = self.TRAY_ID
        agv_number = f"agv{self.AGV_NUMBER}"
        tray_table = "kts1"


        # print(f'Target tray pose:{self.target_tray_pose}')
        # print(f'---------------------Parts info:{self.parts_list}--------------------')

  
        part_poses = []
        part_types = []
        part_colors = []
        part_bins = [] 
   
        for part in self.parts_list:
            part_pose = part.world_pose  # This is a Pose object
            part_types.append(part.part_type)
            part_colors.append(part.color)

            part_poses.append(part_pose)
 

        self.move_robot_home("floor_robot")

        # change gripper type
        self.goto_tool_changer("floor_robot", tray_table, "trays")
        self.change_gripper(2)


        # exit gripper type
        self.exit_tool_changer('floor_robot',tray_table,'trays')

        # pick up tray
        self.enable_gripper(True)
        self.pickup_tray("floor_robot",self.TRAY_ID,self.target_tray_pose,tray_table)

        self.move_tray_to_agv("floor_robot", self.target_tray_pose, agv_number)
        self.place_tray_on_agv("floor_robot", self.TRAY_ID, agv_number)
        self.enable_gripper(False)
        self.retract_from_agv("floor_robot", agv_number)


        self.goto_tool_changer("floor_robot", tray_table, "parts")
        self.change_gripper(1)
        self.exit_tool_changer('floor_robot',tray_table,'parts')


        for i in range(len(part_poses)):
            self.enable_gripper(True)
            self.pickup_part("floor_robot", part_types[i], part_colors[i], part_poses[i],self.bin[i])
            self.move_part_to_agv("floor_robot", part_poses[i], agv_number, self.QUADRANT[i])
            self.place_part_in_tray("floor_robot", agv_number, self.QUADRANT[i])
            self.enable_gripper(False)
            self.retract_from_agv("floor_robot", agv_number)

        # move robot home
        self.move_robot_home("floor_robot")

        # move agv to warehouse
        self.lock_agv(agv_number)
        self.move_agv_to_warehouse(agv_number)


        self.kit_completed = True


    def start_competition(self):
        '''
        Start the competition
        '''
        self.get_logger().info('Waiting for competition state READY')

        request = Trigger.Request()
        future = self.start_competition_client.call_async(request)

        # Wait until the service call is completed
        rclpy.spin_until_future_complete(self, future)

        if future.result().success:
            self.get_logger().info('Started competition.')
            self.competition_started = True
            # self.subscriber = self.create_subscription(Order, '/ariac/orders', self.order_callback, qos_profile=10)

        else:
            self.get_logger().warn('Unable to start competition')

    def goto_tool_changer(self, robot, station, gripper_type):
        '''
        Move the end effector inside the gripper slot.

        Args:
            station (str): Gripper station name
            gripper_type (str): Gripper type

        Raises:
            KeyboardInterrupt: Exception raised when the user presses Ctrl+C
        '''

        self.get_logger().info('Move inside gripper slot service called')

        request = EnterToolChanger.Request()

        if robot == "floor_robot":
            request.robot = RobotsMsg.FLOOR_ROBOT
        else:
            raise ValueError('Invalid robot name')

        request.station = station
        request.gripper_type = gripper_type

        future = self.goto_tool_changer_client.call_async(request)

        try:
            rclpy.spin_until_future_complete(self, future)
        except KeyboardInterrupt as kb_error:
            raise KeyboardInterrupt from kb_error

        if future.result() is not None:
            response = future.result()
            if response:
                self.get_logger().info('Robot is at the tool changer')
        else:
            self.get_logger().error(f'Service call failed {future.exception()}')
            self.get_logger().error('Unable to move the robot to the tool changer')

    def exit_tool_changer(self,robot,station,gripper_type):
        """exit fromt the tool changer

        Args:
            robot (_type_): name of robot
            station (_type_): tray station
            gripper_type (_type_): gripper type

        """
        
        self.get_logger().info('Exit gripper slot service called')

        request = ExitToolChanger.Request()

        if robot == "floor_robot":
            request.robot = RobotsMsg.FLOOR_ROBOT
        else:
            raise ValueError('Invalid robot name')

        request.station = station
        request.gripper_type = gripper_type

        future = self.exit_tool_changer_client.call_async(request)

        try:
            rclpy.spin_until_future_complete(self, future)
        except KeyboardInterrupt as kb_error:
            raise KeyboardInterrupt from kb_error

        if future.result() is not None:
            response = future.result()
            if response:
                self.get_logger().info('Robot is at the tool changer')
        else:
            self.get_logger().error(f'Service call failed {future.exception()}')
            self.get_logger().error('Unable to move the robot to the tool changer')

    def change_gripper(self, gripper_type):
        """chnages the gripper type

        Args:
            gripper_type (_type_): gripper type

        """
        self.get_logger().info('Change gripper service called')

        request = ChangeGripper.Request()
        # if robot == "floor_robot":
        #     request.robot = RobotsMsg.FLOOR_ROBOT
        # else:
        #     raise ValueError('Invalid robot name')

        request.gripper_type = gripper_type

        future = self.change_gripper_client.call_async(request)

        try:
            rclpy.spin_until_future_complete(self, future)
        except KeyboardInterrupt as kb_error:
            raise KeyboardInterrupt from kb_error

        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Gripper changed successfully: {response.message}')
            else:
                self.get_logger().error(f'Gripper change failed: {response.message}')
        else:
            self.get_logger().error(f'Service call failed {future.exception()}')
            self.get_logger().error('Unable to change gripper')


#     # ... (previous code)

    def pickup_tray(self, robot, tray_id, tray_pose, tray_station):
        """to pick up tray 

        Args:
            robot (_type_): name of robot
            tray_id (_type_): tray id
            tray_pose (_type_): pose of tray
            tray_station (_type_): tray station

        Raises:
            ValueError: _description_
            KeyboardInterrupt: _description_
        """
        self.get_logger().info('Pickup tray service called')

        request = PickupTray.Request()

        if robot == "floor_robot":
            request.robot = RobotsMsg.FLOOR_ROBOT
        else:
            raise ValueError('Invalid robot name')

        request.tray_id = tray_id
        request.tray_pose = tray_pose
        request.tray_station = tray_station

        future = self.pickup_tray_client.call_async(request)

        try:
            rclpy.spin_until_future_complete(self, future)
        except KeyboardInterrupt as kb_error:
            raise KeyboardInterrupt from kb_error

        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Tray picked up successfully: ')
            else:
                self.get_logger().error(f'Tray pickup failed: ')
        else:
            self.get_logger().error(f'Service call failed {future.exception()}')
            self.get_logger().error('Unable to pick up tray')

#     # ... (previous code in the _robot_action_timer_callback function)

#         # pick up tray
#         # self.pickup_tray("floor_robot", detected_tray.tray_id, detected_tray.world_pose, tray_table)

#     # ... (remaining code)

    def enable_gripper(self, enable):
        """
        Enable or disable the robot's gripper.

        Args:
            enable (bool): True to enable the gripper, False to disable it.
        """
        while not self.enable_gripper_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for the gripper control service...')

        request = VacuumGripperControl.Request()
        request.enable = enable

        future = self.enable_gripper_client.call_async(request)

        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info('Gripper operation successful')
            else:
                self.get_logger().error('Gripper operation failed')
        else:
            self.get_logger().error(f'Service call failed {future.exception()}')


    def move_robot_home(self, robot_name):
        """
        Move the specified robot to its home position.

        Args:
            robot_name (str):  name of the robot 
        """
        request = Trigger.Request()

        if robot_name == 'floor_robot':
            if not self.move_floor_robot_home_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().error('Robot commander node not running')
                return

            future = self.move_floor_robot_home_client.call_async(request)
        else:
            self.get_logger().error(f'Robot name: ({robot_name}) is not valid')
            return

        # Wait until the service call is completed
        rclpy.spin_until_future_complete(self, future)

        if future.result().success:
            self.get_logger().info(f'Moved {robot_name} to home position')
            # self.create_subscription(Order, '/ariac/orders', self.order_callback, qos_profile=10)


        else:
            self.get_logger().warn(future.result().message)


    def move_tray_to_agv(self, robot, tray_pose, agv):
        """
        Move the tray to the AGV.

        Args:
            robot (str):  name of the robot.
            tray_pose (Pose):  pose of the tray.
            agv (str):    AGV number
        """
        self.get_logger().info('Move tray to AGV service called')

        # Wait for the service to become available
        while not self.move_tray_to_agv_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for the move tray to AGV service...')

        # Create a request object
        request = MoveTrayToAGV.Request()

        if robot == "floor_robot":
            request.robot = RobotsMsg.FLOOR_ROBOT
        else:
            raise ValueError('Invalid robot name')

        request.tray_pose = tray_pose
        request.agv = agv

        # Call the service asynchronously
        future = self.move_tray_to_agv_client.call_async(request)

        # Wait for the service call to complete
        try:
            rclpy.spin_until_future_complete(self, future)
        except KeyboardInterrupt as kb_error:
            raise KeyboardInterrupt from kb_error

        # Check the service call result
        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Tray moved to AGV successfully: ')
            else:
                self.get_logger().error(f'Moving tray to AGV failed: {response.message}')
        else:
            self.get_logger().error(f'Service call failed {future.exception()}')
            self.get_logger().error('Unable to move tray to AGV')

    def place_tray_on_agv(self, robot, tray_id, agv):
        """
        Place the tray on the AGV.

        Args:
            robot (str): name of the robot.
            tray_id (str): tray id.
            agv (str):  AGV number.
        """
        self.get_logger().info('Place tray on AGV service called')

        request = PlaceTrayOnAGV.Request()

        if robot == "floor_robot":
            request.robot = RobotsMsg.FLOOR_ROBOT
        else:
            raise ValueError('Invalid robot name')

        request.tray_id = tray_id
        request.agv = agv

        future = self.place_tray_on_agv_client.call_async(request)

        try:
            rclpy.spin_until_future_complete(self, future)
        except KeyboardInterrupt as kb_error:
            raise KeyboardInterrupt from kb_error

        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info('Tray placed on AGV successfully')
            else:
                self.get_logger().error('Tray placement on AGV failed')
        else:
            self.get_logger().error(f'Service call failed {future.exception()}')
            self.get_logger().error('Unable to place tray on AGV')



    # Define the function to call the service
    def retract_from_agv(self, robot, agv):
        """
        Retract the robot from the AGV.

        Args:
            robot (str): The name of the robot.
            agv (str): The AGV number.
        """
        self.get_logger().info('Retract from AGV service called')

        request = RetractFromAGV.Request()

        if robot == "floor_robot":
            request.robot = RobotsMsg.FLOOR_ROBOT
        else:
            raise ValueError('Invalid robot name')

        request.agv = agv

        future = self.retract_from_agv_client.call_async(request)

        try:
            rclpy.spin_until_future_complete(self, future)
        except KeyboardInterrupt as kb_error:
            raise KeyboardInterrupt from kb_error

        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info('Retracted from AGV successfully')
            else:
                self.get_logger().error('Retraction from AGV failed')
        else:
            self.get_logger().error(f'Service call failed {future.exception()}')
            self.get_logger().error('Unable to retract from AGV')

    def pickup_part(self, robot, part_type, part_color, part_pose, bin_side):
        """
        Pick up a part.

        Args:
            robot (str): The name of the robot.
            part_type (str): The type of the part.
            part_color (str): The color of the part.
            part_pose (Pose): The pose of the part.
            bin_side (str): The side of the bin.
        """
        self.get_logger().info('Pickup part service called')

        request = PickupPart.Request()

        if robot == "floor_robot":
            request.robot = RobotsMsg.FLOOR_ROBOT
        else:
            raise ValueError('Invalid robot name')

        request.part_type = part_type
        request.part_color = part_color
        request.part_pose = part_pose
        request.bin_side = bin_side

        future = self.pickup_part_client.call_async(request)

        try:
            rclpy.spin_until_future_complete(self, future)
        except KeyboardInterrupt as kb_error:
            raise KeyboardInterrupt from kb_error

        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info('Part picked up successfully')
            else:
                self.get_logger().error('Part pickup failed')
        else:
            self.get_logger().error(f'Service call failed {future.exception()}')
            self.get_logger().error('Unable to pick up part')



    def move_part_to_agv(self, robot, part_pose, agv, quadrant):
        """
        Move a part to the AGV.

        Args:
            robot (str): The name of the robot.
            part_pose (Pose): The pose of the part.
            agv (str): The AGV number
            quadrant (int): The quadrant to move the part to.
        """
        self.get_logger().info('Move part to AGV service called')

        request = MovePartToAGV.Request()

        if robot == "floor_robot":
            request.robot = RobotsMsg.FLOOR_ROBOT
        else:
            raise ValueError('Invalid robot name')

        request.part_pose = part_pose
        request.agv = agv
        request.quadrant = quadrant

        future = self.move_part_to_agv_client.call_async(request)

        try:
            rclpy.spin_until_future_complete(self, future)
        except KeyboardInterrupt as kb_error:
            raise KeyboardInterrupt from kb_error

        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info('Part moved to AGV successfully')
            else:
                self.get_logger().error('Part move to AGV failed')
        else:
            self.get_logger().error(f'Service call failed {future.exception()}')
            self.get_logger().error('Unable to move part to AGV')
    
    # def set_floor_robot_gripper_state(self, enable):
    #     self.enable_gripper(enable)


    def place_part_in_tray(self, robot, agv, quadrant):
        """
        Place a part in the tray.

        Args:
            robot (str): The name of the robot.
            agv (str): The AGV number
            quadrant (int): The quadrant to place the part in.
        """

        request = PlacePartInTray.Request()

        if robot == "floor_robot":
         request.robot = RobotsMsg.FLOOR_ROBOT
        else:
            raise ValueError('Invalid robot name')

        request.agv = agv
        request.quadrant = quadrant

        future = self.place_part_in_tray_client.call_async(request)

        try:
            rclpy.spin_until_future_complete(self, future)
        except KeyboardInterrupt:
            self.get_logger().warn('Interrupted while waiting for the service. Exiting...')
            return

        if future.result() is not None:
            response = future.result()
        if response:
            self.get_logger().warn('Deactivate gripper')
            self.enable_gripper(False)
        else:
            self.get_logger().warn(f'Service call failed {future.exception()}')
        
    def lock_agv(self, agv):
        """
        Lock the AGV.

        Args:
            agv (str): The AGV number.
        """
        request = Trigger.Request()

        if agv == 'agv1':
            client = self.create_client(Trigger, '/ariac/agv1_lock_tray')
        elif agv == 'agv2':
            client = self.create_client(Trigger, '/ariac/agv2_lock_tray')
        elif agv == 'agv3':
            client = self.create_client(Trigger, '/ariac/agv3_lock_tray')
        elif agv == 'agv4':
            client = self.create_client(Trigger, '/ariac/agv4_lock_tray')
        else:
            self.get_logger().error(f'Invalid AGV name: {agv}')
            return

        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f'Service /ariac/{agv}_lock_tray is not available')
            return

        try:
            response = client.call_async(request)
            while rclpy.ok():
                rclpy.spin_once(self)
                if response.done():
                    result = response.result()
                    if result.success:
                        self.get_logger().info(f'Tray locked on {agv} successfully')
                    else:
                        self.get_logger().error(f'Failed to lock tray on {agv}')
                    break
        except Exception as e:
            self.get_logger().error(f'Error locking tray on {agv}: {e}')




    def move_agv_to_warehouse(self, agv_number):
        """
        Move the AGV to the warehouse.

        Args:
            agv_number (int): The AGV number
        """
        # Create the request object
        request = MoveAGV.Request()
        request.location = 3

        # Update the service name according to the AGV number
        service_name = f'/ariac/move_agv{agv_number}'

        # Wait for the service to be available
        while not self.move_agv_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f"Service {service_name} not available, waiting...")

        # Call the service
        response = self.move_agv_client.call_async(request)
        rclpy.spin_until_future_complete(self, response)

        if response.result().success:
            self.get_logger().info(f'AGV {agv_number} moved to destination: ')
        else:
            self.get_logger().info(f'Failed to move AGV {agv_number}: ')


def main(args=None):
    """ creates an instance of the node class 

    Args:
        args (_type_, optional):  None.
    """
    rclpy.init(args=args)
    node = RobotCommanderInterface('commander')
    rclpy.spin(node)
    node.destroy_node()

        