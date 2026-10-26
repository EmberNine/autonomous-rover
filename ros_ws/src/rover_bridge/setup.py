from setuptools import setup

package_name = 'rover_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='armando',
    maintainer_email='cardonaarmando95@gmail.com',
    description='ROS 2 bridge to the rover Nucleo (binary link v1)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={'console_scripts': ['bridge = rover_bridge.node:main',
                                    'sim_adapter = rover_bridge.sim_adapter:main']},
)
