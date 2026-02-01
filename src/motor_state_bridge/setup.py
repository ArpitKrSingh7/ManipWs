from setuptools import find_packages, setup

package_name = 'motor_state_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name + '/config', ['config/joint_map.yaml']),
        ('share/' + package_name + '/launch', ['launch/bridge.launch.py']),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='arpit',
    maintainer_email='arpit@gmail.com',
    description='Bridge ESP32 motor states to /joint_states',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'bridge_node = motor_state_bridge.bridge_node:main',
        'trajectory_to_esp = motor_state_bridge.trajectory_to_esp:main',
    ],
    },
)
