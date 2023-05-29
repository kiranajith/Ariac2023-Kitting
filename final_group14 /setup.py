from setuptools import setup
import os
from glob import glob

package_name = 'final_group14'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kiran Ajith',
    maintainer_email='kiran99@umd.edu',
    description='This package is used to simulate the Kitting Part of the ARIAC Competiiton',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'read_orders = final_group14.read_orders:main' ,
            'start_comp = final_group14.start_comp:main' ,
            'locate_part = final_group14.locate_part:main',
            'locate_tray = final_group14.locate_tray:main' 


        ],
    
    },
)
