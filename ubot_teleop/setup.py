from setuptools import find_packages
from setuptools import setup

package_name = 'ubot_teleop'

setup(
    name=package_name,
    version='2.1.4',
    packages=find_packages(exclude=[]),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
    ],
    zip_safe=True,
    author='VV S',
    author_email='vvs@umd.edu',
    maintainer='VV S',
    maintainer_email='vvs@umd.edu',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description=(
        'Teleoperation node using keyboard for ubot.'
    ),
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_keyboard = ubot_teleop.script.teleop_keyboard:main'
        ],
    },
)
