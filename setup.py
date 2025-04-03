from setuptools import find_packages, setup

package_name = 'ur_asu'


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools',  'rclpy', 'moveit_ros_planning_interface'],
    zip_safe=True,
    maintainer='kpatnaik',
    maintainer_email='kpatnaik@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = ur_asu.testmove_ur5e:main',
            'ik_solver = ur_asu.ik_solver:main'
        ],
    },
)
