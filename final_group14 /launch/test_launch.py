#  pull in some Python launch modules.

from launch import LaunchDescription
from launch_ros.actions import Node
# this function is needed

def generate_launch_description():
    ld = LaunchDescription() # instantiate a Launchdescription object
    start_comp = Node( # declare your Node
    package="final_group14", # package name
    executable="start_comp" # executable as set in setup.py
    )
    read_order = Node( # declare your Node
    package="final_group14", # package name
    executable="read_orders" # executable as set in setup.py
    ) 
    ld.add_action(start_comp) # add each Node to the LaunchDescription object
    ld.add_action(read_order)
    return ld