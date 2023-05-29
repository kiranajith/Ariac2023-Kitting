import rclpy
from rclpy.node import Node
from ariac_msgs.msg import Order

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

    def __str__(self) -> str:
        kitting_task_str = f' agv_number: {self.kitting_task.agv_number}\n  tray_id: {self.kitting_task.tray_id}\n  destination: {self.kitting_task.destination}\n  parts:\n'

        part_colors = ['red', 'green', 'blue', 'orange', 'purple']
        part_types = [None, None, None, None, None, None, None, None, None, None, 'battery', 'pump', 'sensor', 'regulator']

        for part in self.kitting_task.parts:
            part_color = part_colors[part.part.color]
            part_type = part_types[part.part.type]
            kitting_task_str += f' - part:\n   color: {part_color}\n   type: {part_type}\n   quadrant: {part.quadrant}\n'

        return f'id : {self.id}\ntype : {self.type}\npriority : {self.priority}\nkitting_task :\n {kitting_task_str}'

class MyNode(Node):
    """
    A ROS2 node that subscribes to the '/ariac/orders' topic and processes incoming orders using the orderInfo class.

    """

    def __init__(self, node_name):
        """
        Initialize the node with the given name.
        """
        super().__init__(node_name)
        self.declare_parameter('order_id', '1')
        self.order_id = self.get_parameter('order_id').value

        # Subscriber
        self.subscriber = self.create_subscription(Order, '/ariac/orders', self._subscriber_callback, qos_profile=10)

        # Dictionary to store orders
        self.orders = {}

        self.part_colors = ['red', 'green', 'blue', 'orange', 'purple']
        self.part_types = [None, None, None, None, None, None, None, None, None, None, 'battery', 'pump', 'sensor', 'regulator']


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
            print(order_info)
            for part in self.orders[self.order_id].kitting_task.parts:
                part_color = self.part_colors[part.part.color]
                part_type = self.part_types[part.part.type]
                print(f" - Color: {part_color}, Type: {part_type}")

        else:
            self.get_logger().info(f'Storing order with ID: {order_id}')

def main(args=None):
    """ creates an instance of the node class 

    Args:
        args (_type_, optional):  None.
    """
    rclpy.init(args=args)
    node = MyNode('order_reader')
    rclpy.spin(node)
    node.destroy_node()
   
