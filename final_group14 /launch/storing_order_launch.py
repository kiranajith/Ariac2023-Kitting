#  pull in some Python launch modules.
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
# this function is needed

def generate_launch_description():
    ld = LaunchDescription() # instantiate a Launchdescription object

    config  = os.path.join(
        get_package_share_directory('')
    )
    
    read_orders = Node( # declare your Node
    package="final_group14", # package name
    executable="read_orders" # executable as set in setup.py
    )
    locate_part = Node( # declare your Node
    package="final_group14", # package name
    executable="locate_part" # executable as set in setup.py
    ) 
    locate_tray = Node( # declare your Node
    package="final_group14", # package name
    executable="locate_tray" # executable as set in setup.py
    ) 
    # add each Node to the LaunchDescription object
    ld.add_action(read_orders)
    ld.add_action(locate_part)
    ld.add_action(locate_tray)
    return ld