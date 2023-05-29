import rclpy
from rclpy.node import Node
from ariac_msgs.msg import Order 
from custom_interfaces.msg import SavedOrder, PartQuantity

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
        self.part_type_selected = None
        self.agv_number = None
        self.tray_id = None
        # self.parts_types = []
        # self.part_colors = []
        
    def __str__(self) -> str:


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
                self.part_type_selected = 'battery'
            elif part.part.type == 11:
                self.part_type_selected = 'pump'
            elif part.part.type == 12:
                self.part_type_selected = 'sensor'
            elif part.part.type == 13:
                self.part_type_selected = 'regulator'
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
            Part: [{self.part_type_selected},{self.part_color_selected}]
            Quadrant: 3
            ------
            Part: [{self.part_type_selected},{self.part_color_selected}]
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
        self.declare_parameter('order_id', '1')
        self.order_id = self.get_parameter('order_id').value
        print(self.order_id)

        self.publisher = self.create_publisher(SavedOrder, '/CustomTopic', 10)

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


    # def _subscriber_callback(self, message):
    #     """
    #     Process an incoming message from the '/ariac/orders' topic.

    #     """
    #     self.get_logger().info('Order Received')
    #     order_id = message.id
    #     order_type = message.type
    #     order_priority = message.priority
    #     order_kitting_task = message.kitting_task

    #     order_info = orderInfo(order_id, order_type, order_priority, order_kitting_task)

    #     # Store the order in the dictionary
    #     self.orders[order_id] = order_info

    #     # Check if the received order matches the specified order_id from the parameter
    #     if order_id == self.order_id:
    #         self.get_logger().info(f'Processing order with ID: {self.order_id}')
    #         print(order_info)

    #         # print(self.orders[self.order_id])
    #         saved_order_msg = SavedOrder()
    #         saved_order_msg.agv_number = int(order_info.agv_number)
    #         saved_order_msg.tray_id = int(order_info.tray_id)
    #         saved_order_msg.part_type = order_info.part_type_selected
    #         saved_order_msg.part_color = order_info.part_color_selected
    #         saved_order_msg.quadrant = order_info.kitting_task.parts[0].quadrant
            
    #     # Publish the custom message
    #         self.publisher.publish(saved_order_msg)

    #         # print(order_info)
  
    #         self.get_logger().info(f"{self.color} {self.part}")
            
    #     else:
    #         self.get_logger().info(f'Storing order with ID: {order_id}')

    # def _subscriber_callback(self, message):
    #     # Process the incoming message and create an order_info object
    #     order_id = message.id
    #     order_type = message.type
    #     order_priority = message.priority
    #     order_kitting_task = message.kitting_task
    #     order_info = orderInfo(order_id, order_type, order_priority, order_kitting_task)

    #     # Store the order in the dictionary
    #     self.orders[order_id] = order_info

    #     # Check if the received order matches the specified order_id from the parameter
    #     if order_id == self.order_id:
    #         self.get_logger().info(f'Processing order with ID: {self.order_id}')
    #         print(order_info)

    #         # Create a SavedOrder message
    #         saved_order_msg = SavedOrder()

    #         # Set AGV number, tray ID, part type, part color, and quadrant
    #         saved_order_msg.agv_number = int(order_info.agv_number)
    #         saved_order_msg.tray_id = int(order_info.tray_id)
        

    #         # Set kitting task
    #         saved_order_msg.kitting_task = [
    #             PartQuantity(part=part_info["part"], quantity=part_info["quantity"])
    #             for part_info in order_info.kitting_task
    #         ]

    #         # Publish the custom message
    #         self.publisher.publish(saved_order_msg)

    #         self.get_logger().info(f"{order_info.part_color_selected} {order_info.part_type_selected}")
    #     else:
    #         self.get_logger().info(f'Storing order with ID: {order_id}')
    # def _subscriber_callback(self, message):
    #     """
    #     Process an incoming message from the '/ariac/orders' topic.

    #     """
    #     self.get_logger().info('Order Received')
    #     order_id = message.id
    #     order_type = message.type
    #     order_priority = message.priority
    #     order_kitting_task = message.kitting_task

    #     order_info = orderInfo(order_id, order_type, order_priority, order_kitting_task)

    #     # Store the order in the dictionary
    #     self.orders[order_id] = order_info

    #     # Check if the received order matches the specified order_id from the parameter
    #     if order_id == self.order_id:
    #         self.get_logger().info(f'Processing order with ID: {self.order_id}')
    #         print(order_info)

    #         # Create a SavedOrder message
    #         saved_order_msg = SavedOrder()
    #         saved_order_msg.order_id = int(order_id)

    #         # Set AGV number and tray ID
    #         if order_info.agv_number is not None:
    #             saved_order_msg.agv_number = int(order_info.agv_number)
    #         if order_info.tray_id is not None:
    #             saved_order_msg.tray_id = int(order_info.tray_id)

    #         # Create a PartQuantity list from the orderInfo.kitting_task.parts list
    #         part_quantity_list = []
    #         for part in order_info.kitting_task.parts:
    #             part_quantity = PartQuantity()
    #             part_type_color = f'{order_info.part_type_selected}, {order_info.part_color_selected}'
    #             part_quantity.part = part_type_color
    #             part_quantity.quantity = 1  # Assuming a quantity of 1 for each part
    #             part_quantity_list.append(part_quantity)

    #         # Assign the PartQuantity list to the saved_order_msg.kitting_task field
    #         saved_order_msg.kitting_task = part_quantity_list

    #         # Publish the custom message
    #         self.publisher.publish(saved_order_msg)

    #         self.get_logger().info(f"{order_info.part_color_selected} {order_info.part_type_selected}")

    #     else:
    #         self.get_logger().info(f'Storing order with ID: {order_id}')
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

            # Create a SavedOrder message
            saved_order_msg = SavedOrder()
            saved_order_msg.order_id = int(order_id)

            # Set AGV number and tray ID
            if order_info.agv_number is not None:
                saved_order_msg.agv_number = int(order_info.agv_number)
            if order_info.tray_id is not None:
                saved_order_msg.tray_id = int(order_info.tray_id)

            # Create a PartQuantity list from the orderInfo.kitting_task.parts list
            part_quantity_list = []
            for part in order_info.kitting_task.parts:
                # Update part_type_selected and part_color_selected based on the current part
                if part.part.type == 10:
                    part_type_selected = 'battery'
                elif part.part.type == 11:
                    part_type_selected = 'pump'
                elif part.part.type == 12:
                    part_type_selected = 'sensor'
                elif part.part.type == 13:
                    part_type_selected = 'regulator'
                else:
                    part_type_selected = None

                if part.part.color == 0:
                    part_color_selected = 'red'
                elif part.part.color == 1:
                    part_color_selected = 'green'
                elif part.part.color == 2:
                    part_color_selected = 'blue'
                elif part.part.color == 3:
                    part_color_selected = 'orange'
                elif part.part.color == 4:
                    part_color_selected = 'purple'
                else:
                    part_color_selected = None
                
                quadrant = part.quadrant


                part_quantity = PartQuantity()
                part_type_color = f'{part_type_selected}, {part_color_selected}'
                part_quantity.part = part_type_color
                #part_quantity.quantity = 1  # Assuming a quantity of 1 for each part
                part_quantity.quadrant = quadrant

                part_quantity_list.append(part_quantity)

            # Assign the PartQuantity list to the saved_order_msg.kitting_task field
            saved_order_msg.kitting_task = part_quantity_list

            # Publish the custom message
            self.publisher.publish(saved_order_msg)

            self.get_logger().info(f"{part_color_selected} {part_type_selected}")

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
