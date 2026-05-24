from setuptools import find_packages, setup

package_name = 'rb23_ros_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='denis',
    maintainer_email='denis.mosconi@ifsp.edu.br',
    description='ROS 2 driver for GuardBot RollerBot 23',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rb23_driver_node = rb23_ros_driver.rb23_driver_node:main',
        ],
    },
)
