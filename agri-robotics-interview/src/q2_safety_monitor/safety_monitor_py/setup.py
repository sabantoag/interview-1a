from setuptools import find_packages, setup

package_name = 'safety_monitor_py'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/q2_demo.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jason Lambert',
    maintainer_email='jason.lambert@sabantoag.com',
    description=(
        'Interview exercise: ROS2 tractor safety monitor with range '
        'checks, debouncing, and geofencing.'
    ),
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'telemetry_publisher_node = safety_monitor_py.telemetry_publisher_node:main',
            'safety_monitor_node = safety_monitor_py.safety_monitor_node:main',
        ],
    },
)
