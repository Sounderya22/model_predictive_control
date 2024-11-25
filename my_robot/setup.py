from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'my_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name,'launch'),
         glob(os.path.join('launch','*.launch.py'))),
        (os.path.join('share',package_name,'urdf'),
         glob(os.path.join('urdf','*.xacro'))),
         (os.path.join('share',package_name,'urdf'),
         glob(os.path.join('urdf','*.gazebo'))),
        (os.path.join('share',package_name,'worlds'),
         glob(os.path.join('worlds','*.world'))),
        (os.path.join('share',package_name,'meshes'),
         glob(os.path.join('meshes','*.dae'))),
        (os.path.join('share',package_name,'meshes'),
         glob(os.path.join('meshes','*.png'))),
        (os.path.join('share',package_name,'meshes'),
         glob(os.path.join('meshes','*.stl'))),
        (os.path.join('share',package_name,'config'),
         glob(os.path.join('config','*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='khuzema',
    maintainer_email='khabib@umd.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop = my_robot.scripts.teleop:main',
            'pd_control = my_robot.scripts.pd_control:main',
            'mpc_control = my_robot.scripts.mpc_control:main'
        ],
    },
)
