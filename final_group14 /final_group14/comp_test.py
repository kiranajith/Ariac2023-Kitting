# import rclpy
# from rclpy.node import Node
# from rclpy.parameter import Parameter
# from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
# from std_srvs.srv import Trigger
# from ariac_msgs.msg import CompetitionState
# from competitor_interfaces.msg import Robots as RobotsMsg
# from competitor_interfaces.srv import (
#     EnterToolChanger
# )


# class RobotCommanderInterface(Node):
   

#     def __init__(self):
#         super().__init__('competition_interface')

#         sim_time = Parameter(
#             "use_sim_time",
#             rclpy.Parameter.Type.BOOL,
#             True
#         )

#         self.set_parameters([sim_time])

#         timer_group = MutuallyExclusiveCallbackGroup()
#         service_group = MutuallyExclusiveCallbackGroup()

#         # Flag to indicate if the kit has been completed
#         self._kit_completed = False
#         self._competition_started = False
#         self._competition_state = None

#         # subscriber
#         self.create_subscription(CompetitionState, '/ariac/competition_state',
#                                  self._competition_state_cb, 1)

#         # timer
#         self._robot_action_timer = self.create_timer(1, self._robot_action_timer_callback,
#                                                      callback_group=timer_group)

#         # Service client for starting the competition
#         self._start_competition_client = self.create_client(Trigger, '/ariac/start_competition')

#         # Service client for moving the floor robot to the home position
#         self._move_floor_robot_home_client = self.create_client(
#             Trigger, '/competitor/floor_robot/go_home',
#             callback_group=service_group)
        
#         # Service client for entering the gripper slot
#         self._goto_tool_changer_client = self.create_client(
#             EnterToolChanger, '/competitor/floor_robot/enter_tool_changer',
#             callback_group=service_group)

        

#     def _competition_state_cb(self, msg: CompetitionState):
       
#         self._competition_state = msg.competition_state

#     def _robot_action_timer_callback(self):
       

#         if self._competition_state == CompetitionState.READY and not self._competition_started:
#             self.start_competition()

#         # exit the callback if the kit is completed
#         if self._kit_completed:
#             return

#         # move robot home
#         self.move_robot_home("floor_robot")

#         # change gripper type
#         self.goto_tool_changer("floor_robot", "kts1", "trays")

#         # to ignore function calls in this callback
#         self._kit_completed = True

#     def start_competition(self):
      
#         self.get_logger().info('Waiting for competition state READY')

#         request = Trigger.Request()
#         future = self._start_competition_client.call_async(request)

#         # Wait until the service call is completed
#         rclpy.spin_until_future_complete(self, future)

#         if future.result().success:
#             self.get_logger().info('Started competition.')
#             self._competition_started = True
#         else:
#             self.get_logger().warn('Unable to start competition')

#     def goto_tool_changer(self, robot, station, gripper_type):
     

#         self.get_logger().info('Move inside gripper slot service called')

#         request = EnterToolChanger.Request()

#         if robot == "floor_robot":
#             request.robot = RobotsMsg.FLOOR_ROBOT
#         else:
#             raise ValueError('Invalid robot name')

#         request.station = station
#         request.gripper_type = gripper_type

#         future = self._goto_tool_changer_client.call_async(request)

#         try:
#             rclpy.spin_until_future_complete(self, future)
#         except KeyboardInterrupt as kb_error:
#             raise KeyboardInterrupt from kb_error

#         if future.result() is not None:
#             response = future.result()
#             if response:
#                 self.get_logger().info('Robot is at the tool changer')
#         else:
#             self.get_logger().error(f'Service call failed {future.exception()}')
#             self.get_logger().error('Unable to move the robot to the tool changer')

#     def move_robot_home(self, robot_name):
        
#         request = Trigger.Request()

#         if robot_name == 'floor_robot':
#             if not self._move_floor_robot_home_client.wait_for_service(timeout_sec=1.0):
#                 self.get_logger().error('Robot commander node not running')
#                 return

#             future = self._move_floor_robot_home_client.call_async(request)
#         else:
#             self.get_logger().error(f'Robot name: ({robot_name}) is not valid')
#             return

#         # Wait until the service call is completed
#         rclpy.spin_until_future_complete(self, future)

#         if future.result().success:
#             self.get_logger().info(f'Moved {robot_name} to home position')
#         else:
#             self.get_logger().warn(future.result().message)


# def main(args=None):
    
#     rclpy.init(args=args)
#     node = RobotCommanderInterface()
#     rclpy.spin(node)
#     node.destroy_node()

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_srvs.srv import Trigger
from ariac_msgs.msg import CompetitionState
from geometry_msgs.msg import Pose
from ariac_msgs.msg import Part

from ariac_msgs.srv import ChangeGripper, VacuumGripperControl, MoveAGV
from competitor_interfaces.msg import Robots as RobotsMsg
from tf_transformations import quaternion_from_euler
from competitor_interfaces.srv import (
    EnterToolChanger, ExitToolChanger, PickupTray, MoveTrayToAGV, PlaceTrayOnAGV,
    RetractFromAGV, PickupPart, MovePartToAGV, PlacePartInTray
)

class RobotCommanderInterface(Node):
    def __init__(self,node_name):
        super().__init__(node_name)

        sim_time = Parameter(
            "use_sim_time",
            rclpy.Parameter.Type.BOOL,
            True
        )

        self.set_parameters([sim_time])

        timer_group = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()

        self._kit_completed = False
        self._competition_started = False
        self._competition_state = None

        self.create_subscription(CompetitionState, '/ariac/competition_state',
                                 self._competition_state_cb, 1)

        self._robot_action_timer = self.create_timer(1, self._robot_action_timer_callback,
                                                     callback_group=timer_group)

        self._start_competition_client = self.create_client(Trigger, '/ariac/start_competition')

        self._move_floor_robot_home_client = self.create_client(
            Trigger, '/competitor/floor_robot/go_home')

        self._goto_tool_changer_client = self.create_client(
            EnterToolChanger, '/competitor/floor_robot/enter_tool_changer')

        self._exit_tool_changer_client = self.create_client(
            ExitToolChanger, '/competitor/floor_robot/exit_tool_changer')

        self._pickup_tray_client = self.create_client(
            PickupTray, '/competitor/floor_robot/pickup_tray')

        self._move_tray_to_agv_client = self.create_client(
            MoveTrayToAGV, '/competitor/floor_robot/move_tray_to_agv')

        self._place_tray_on_agv_client = self.create_client(
            PlaceTrayOnAGV, '/competitor/floor_robot/place_tray_on_agv')

        self._retract_from_agv_client = self.create_client(
            RetractFromAGV, '/competitor/floor_robot/retract_from_agv')

        self._pickup_part_client = self.create_client(
            PickupPart, '/competitor/floor_robot/pickup_part')

        self._move_part_to_agv_client = self.create_client(
            MovePartToAGV, '/competitor/floor_robot/move_part_to_agv')

        self._place_part_in_tray_client = self.create_client(
            PlacePartInTray, '/competitor/floor_robot/place_part_in_tray')

        self._lock_tray_client = self.create_client(
            Trigger, '/ariac/agv1_lock_tray')

        self._move_agv_client = self.create_client(
            MoveAGV, '/ariac/move_agv4')

        self._change_gripper_client = self.create_client(
            ChangeGripper, '/ariac/floor_robot_change_gripper')

        self._enable_gripper_client = self.create_client(
            VacuumGripperControl, '/ariac/floor_robot_enable_gripper')

    def _competition_state_cb(self, msg: CompetitionState):
        self._competition_state = msg.competition_state

    def _robot_action_timer_callback(self):
        '''
        Callback for the timer that triggers the robot actions
        '''

        if self._competition_state == CompetitionState.READY and not self._competition_started:
            self.start_competition()

        # exit the callback if the kit is completed
        if self._kit_completed:
            return

        # move robot home
        # self.move_robot_home("floor_robot")

        # change gripper type
        # self.goto_tool_changer("floor_robot", "kts1", "trays")
        
        #change gripper
        # self.change_gripper(2)


        #exit gripper type
        # self.exit_tool_changer('floor_robot','kts1','trays')

        #pick up tray
        # self.pickup_tray("floor_robot",tray_id,tray_pose,tray_table)

        # to ignore function calls in this callback
        # self._kit_completed = True









    ##################################

        # Pose of tray in the world frame
        quaternion = quaternion_from_euler(0, 0, 3.141591)
        tray_pose = Pose()
        tray_pose.position.x = -0.870000
        tray_pose.position.y = -5.840000
        tray_pose.position.z = 0.734990
        tray_pose.orientation.x = quaternion[0]
        tray_pose.orientation.y = quaternion[1]
        tray_pose.orientation.z = quaternion[2]
        tray_pose.orientation.w = quaternion[3]

        # Pose of a purple pump in the world frame
        quaternion = quaternion_from_euler(0, 0, 3.141591)
        purple_pump_pose = Pose()
        purple_pump_pose.position.x = -2.079999
        purple_pump_pose.position.y = 2.805001
        purple_pump_pose.position.z = 0.723487
        purple_pump_pose.orientation.x = quaternion[0]
        purple_pump_pose.orientation.y = quaternion[1]
        purple_pump_pose.orientation.z = quaternion[2]
        purple_pump_pose.orientation.w = quaternion[3]

        pump_type = Part.PUMP
        pump_color = Part.PURPLE
        pump_quadrant = 2
        pump_bin = "right_bins"

        # Pose of a blue battery in the world frame
        quaternion = quaternion_from_euler(0, 0, 3.141591)
        blue_battery_pose = Pose()
        blue_battery_pose.position.x = -2.080001
        blue_battery_pose.position.y = -2.445000
        blue_battery_pose.position.z = 0.723434
        blue_battery_pose.orientation.x = quaternion[0]
        blue_battery_pose.orientation.y = quaternion[1]
        blue_battery_pose.orientation.z = quaternion[2]
        blue_battery_pose.orientation.w = quaternion[3]

        battery_type = Part.BATTERY
        battery_color = Part.BLUE
        battery_quadrant = 4
        battery_bin = "left_bins"

        # ID of the tray
        tray_id = 3
        # table where the tray is located
        tray_table = "kts1"

        # AGV
        agv = "agv4"


        # # move robot home
        # self.move_robot_home("floor_robot")

        # # change gripper type
        # self.goto_tool_changer("floor_robot", tray_table, "trays")

        # self.change_gripper(2)

        # self.exit_tool_changer('floor_robot',tray_table,'trays')

        # self.retract_from_tool_changer("floor_robot", tray_table, "trays")
        # # pick and place tray
        # self.enable_gripper(True)
        # self.pickup_tray("floor_robot", tray_id, tray_pose, tray_table)
        # self.move_tray_to_agv("floor_robot", tray_pose, agv)
        # self.place_tray_on_agv("floor_robot", tray_id, agv)
        # self.enable_gripper(False)

        # self.retract_from_agv("floor_robot", agv)

        # # # change gripper to pick up parts
        # self.goto_tool_changer("floor_robot", tray_table, "parts")
        # self.change_gripper(1)

        # self.exit_tool_changer('floor_robot',tray_table,'parts')
        
        # # self.retract_from_tool_changer("floor_robot", tray_table, "parts")

        # # # # pick and place purple pump
        # self.enable_gripper(True)

        # # self.pickup_part("floor_robot", pump_type, pump_color, purple_pump_pose,pump_bin)

        # # self.move_part_to_agv("floor_robot", purple_pump_pose, agv, pump_quadrant)
        # # self.place_part_in_tray("floor_robot", agv, pump_quadrant)
        # # self.enable_gripper(False)


        # # self.retract_from_agv("floor_robot", agv)

        # # move robot home
        # # self.move_robot_home("floor_robot")

        # # pick and place blue battery
        print(f'''
                part_type:{battery_type}
                battery_color:{battery_color}
                battery pose:{blue_battery_pose}
                batter_bin :{battery_bin}
                
                ''')
        self.pickup_part("floor_robot", battery_type, battery_color, blue_battery_pose,battery_bin)
        # self.move_part_to_agv("floor_robot", blue_battery_pose, agv, battery_quadrant)
        # self.place_part_in_tray("floor_robot", agv, battery_quadrant)
        # self.enable_gripper(False)
        # self.retract_from_agv("floor_robot", agv)

        # # # move robot home
        # self.move_robot_home("floor_robot")

        # # move agv to warehouse
        # self.lock_agv(agv)
        # self.move_agv_to_warehouse(agv)

        # self._kit_completed = True




    def start_competition(self):
        '''
        Start the competition
        '''
        self.get_logger().info('Waiting for competition state READY')

        request = Trigger.Request()
        future = self._start_competition_client.call_async(request)

        # Wait until the service call is completed
        rclpy.spin_until_future_complete(self, future)

        if future.result().success:
            self.get_logger().info('Started competition.')
            self._competition_started = True
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

        future = self._goto_tool_changer_client.call_async(request)

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
        
        self.get_logger().info('Exit gripper slot service called')

        request = ExitToolChanger.Request()

        if robot == "floor_robot":
            request.robot = RobotsMsg.FLOOR_ROBOT
        else:
            raise ValueError('Invalid robot name')

        request.station = station
        request.gripper_type = gripper_type

        future = self._exit_tool_changer_client.call_async(request)

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
        self.get_logger().info('Change gripper service called')

        request = ChangeGripper.Request()
        # if robot == "floor_robot":
        #     request.robot = RobotsMsg.FLOOR_ROBOT
        # else:
        #     raise ValueError('Invalid robot name')

        request.gripper_type = gripper_type

        future = self._change_gripper_client.call_async(request)

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


    # ... (previous code)

    def pickup_tray(self, robot, tray_id, tray_pose, tray_station):
        self.get_logger().info('Pickup tray service called')

        request = PickupTray.Request()

        if robot == "floor_robot":
            request.robot = RobotsMsg.FLOOR_ROBOT
        else:
            raise ValueError('Invalid robot name')

        request.tray_id = tray_id
        request.tray_pose = tray_pose
        request.tray_station = tray_station

        future = self._pickup_tray_client.call_async(request)

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

    # ... (previous code in the _robot_action_timer_callback function)

        # pick up tray
        # self.pickup_tray("floor_robot", detected_tray.tray_id, detected_tray.world_pose, tray_table)

    # ... (remaining code)

    def enable_gripper(self, enable):
        while not self._enable_gripper_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for the gripper control service...')

        request = VacuumGripperControl.Request()
        request.enable = enable

        future = self._enable_gripper_client.call_async(request)

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
        '''Move one of the robots to its home position.

        Arguments:
            robot_name -- Name of the robot to move home
        '''
        request = Trigger.Request()

        if robot_name == 'floor_robot':
            if not self._move_floor_robot_home_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().error('Robot commander node not running')
                return

            future = self._move_floor_robot_home_client.call_async(request)
        else:
            self.get_logger().error(f'Robot name: ({robot_name}) is not valid')
            return

        # Wait until the service call is completed
        rclpy.spin_until_future_complete(self, future)

        if future.result().success:
            self.get_logger().info(f'Moved {robot_name} to home position')
        else:
            self.get_logger().warn(future.result().message)


    def move_tray_to_agv(self, robot, tray_pose, agv):
        self.get_logger().info('Move tray to AGV service called')

        # Wait for the service to become available
        while not self._move_tray_to_agv_client.wait_for_service(timeout_sec=1.0):
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
        future = self._move_tray_to_agv_client.call_async(request)

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
        self.get_logger().info('Place tray on AGV service called')

        request = PlaceTrayOnAGV.Request()

        if robot == "floor_robot":
            request.robot = RobotsMsg.FLOOR_ROBOT
        else:
            raise ValueError('Invalid robot name')

        request.tray_id = tray_id
        request.agv = agv

        future = self._place_tray_on_agv_client.call_async(request)

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
        self.get_logger().info('Retract from AGV service called')

        request = RetractFromAGV.Request()

        if robot == "floor_robot":
            request.robot = RobotsMsg.FLOOR_ROBOT
        else:
            raise ValueError('Invalid robot name')

        request.agv = agv

        future = self._retract_from_agv_client.call_async(request)

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

        future = self._pickup_part_client.call_async(request)

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
        self.get_logger().info('Move part to AGV service called')

        request = MovePartToAGV.Request()

        if robot == "floor_robot":
            request.robot = RobotsMsg.FLOOR_ROBOT
        else:
            raise ValueError('Invalid robot name')

        request.part_pose = part_pose
        request.agv = agv
        request.quadrant = quadrant

        future = self._move_part_to_agv_client.call_async(request)

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


        request = PlacePartInTray.Request()

        if robot == "floor_robot":
         request.robot = RobotsMsg.FLOOR_ROBOT
        else:
            raise ValueError('Invalid robot name')

        request.agv = agv
        request.quadrant = quadrant

        future = self._place_part_in_tray_client.call_async(request)

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
        # Create the request object
        request = MoveAGV.Request()
        request.location = 3

        # Update the service name according to the AGV number
        service_name = f'/ariac/move_agv{agv_number}'

        # Wait for the service to be available
        while not self._move_agv_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f"Service {service_name} not available, waiting...")

        # Call the service
        response = self._move_agv_client.call_async(request)
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
