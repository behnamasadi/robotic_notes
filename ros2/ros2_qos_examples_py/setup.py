from setuptools import find_packages, setup

package_name = 'ros2_qos_examples_py'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Behnam Asadi',
    maintainer_email='behnam.asadi@gmail.com',
    description='QoS demos in Python: SensorDataQoS publisher and subscriber.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'camera_publisher = ros2_qos_examples_py.camera_publisher:main',
            'latest_frame_subscriber = ros2_qos_examples_py.latest_frame_subscriber:main',
        ],
    },
)
