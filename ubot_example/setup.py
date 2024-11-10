from setuptools import find_packages
from setuptools import setup

package_name = 'ubot_example'

setup(
    name=package_name,
    version='2.1.4',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # To be added
        # ('share/' + package_name + '/launch', glob.glob(os.path.join('launch',
        #                                                 'ubot_interactive_marker.launch.py'))),
        # ('share/' + package_name + '/launch', glob.glob(os.path.join('launch',
        #                                                 'ubot_obstacle_detection.launch.py'))),
        # ('share/' + package_name + '/rviz', glob.glob(os.path.join('rviz',
        #                                               'ubot_interactive_marker.rviz'))),
    ],
    install_requires=['setuptools', 'launch'],
    zip_safe=True,
    author=['VV S'],
    author_email=['vvs@umd.edu'],
    maintainer='VV S',
    maintainer_email='vvs@umd.edu',
    keywords=['ROS', 'ROS2', 'examples', 'rclpy'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description=(
        'Examples of Different ubot Usage.'
    ),
    license='Apache License, Version 2.0',
    entry_points={
        'console_scripts': [
            # To be added
            # 'ubot_interactive_marker = \
            #   ubot_example.ubot_interactive_marker.main:main',
            'ubot_obstacle_detection = \
                ubot_example.ubot_obstacle_detection.main:main',
            'ubot_patrol_client = \
                ubot_example.ubot_patrol_client.main:main',
            'ubot_patrol_server = \
                ubot_example.ubot_patrol_server.main:main',
            'ubot_position_control = \
                ubot_example.ubot_position_control.main:main',
        ],
    },
)
