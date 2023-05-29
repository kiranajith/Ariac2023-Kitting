import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_srvs.srv import Trigger
from ariac_msgs.msg import CompetitionState
from geometry_msgs.msg import Pose
from ariac_msgs.msg import Part
from ariac_msgs.msg import Order 

from ariac_msgs.srv import ChangeGripper, VacuumGripperControl, MoveAGV
from competitor_interfaces.msg import Robots as RobotsMsg
from tf_transformations import quaternion_from_euler
from competitor_interfaces.srv import (
    EnterToolChanger, ExitToolChanger, PickupTray, MoveTrayToAGV, PlaceTrayOnAGV,
    RetractFromAGV, PickupPart, MovePartToAGV, PlacePartInTray
)
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
        
    def __str__(self) -> str:


        kitting_task_str = f' agv_number: {self.kitting_task.agv_number}\n  tray_id: {self.kitting_task.tray_id}\n  destination: {self.kitting_task.destination}\n  parts:\n'
        self.agv_number= self.kitting_task.agv_number
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
            kitting_task_str += f' - part:\n   color: {part.part.color}\n   type: {part.part.type}\n   quadrant: {part.quadrant}\n'
            new_str = f'''
            =================
            Processing Order {self.id}
            =================
            Type: Kitting
            Priority: 0

            Kitting Task
            ===============
            AGV: {self.kitting_task.agv_number}
            Tray ID: {self.kitting_task.tray_id}
            Destination: warehouse
            ===============
            Products:
            ------
            Part: [sensor,red]
            Quadrant: 3
            ------
            Part: [regulator,orange]
            Quadrant: {part.quadrant}
            '''
        return f'id : {self.id}\ntype : {self.type}\npriority : {self.priority}\nkitting_task :\n {kitting_task_str}'
        # return new_str
class MyNode(Node):
    """
    A ROS2 node that subscribes to the '/ariac/orders' topic and processes incoming orders using the orderInfo class.

    """

    def __init__(self, node_name):
        """
        Initialize the node with the given name.
        """
        super().__init__(node_name)
        sim_time = Parameter(
            "use_sim_time",
            rclpy.Parameter.Type.BOOL,
            True
        )
        
        self.set_parameters([sim_time])
        self.get_logger().info('node created')




        
        self.declare_parameter('order_id', '1')
        self.order_id = self.get_parameter('order_id').value
        # Subscriber
        self.subscriber = self.create_subscription(Order, '/ariac/orders', self._subscriber_callback, qos_profile=10)

        # Dictionary to store orders
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
        self.get_logger().info('Service clients Declared')


    def _subscriber_callback(self, message):
        """
        Process an incoming message from the '/ariac/orders' topic.

        """
        self.get_logger().info('Order Received')
        order_id = message.id
        order_type = message.type
        order_priority = message.priority
        order_kitting_task = message.kitting_task

        order_info = orderInfo(order_id, order_type, order_priority, order_kitting_task)

        # Store the order in the dictionary
        self.orders[order_id] = order_info

        # Check if the received order matches the specified order_id from the parameter
        if order_id == self.order_id:
            self.get_logger().info(f'Processing order with ID: {self.order_id}')
            # print(self.orders[self.order_id])
            print(order_info)
            self.get_logger().info(f"{self.color} {self.part}")
            
        else:
            self.get_logger().info(f'Storing order with ID: {order_id}')

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
            # self.subscriber = self.create_subscription(Order, '/ariac/orders', self._order_callback, qos_profile=10)

        else:
            self.get_logger().warn('Unable to start competition')

def main(args=None):
    """ creates an instance of the node class 

    Args:
        args (_type_, optional):  None.
    """
    rclpy.init(args=args)
    node = MyNode('order_reader')
    rclpy.spin(node)
    node.destroy_node()
