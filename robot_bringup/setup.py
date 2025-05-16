from setuptools import find_packages, setup

package_name = 'robot_bringup'

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
    maintainer='dhruvil',
    maintainer_email='dhruvil@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ee_marker = robot_bringup.marker:main',
            'hand_tracking = robot_bringup.hand_tracking:main',
            'teleop_moveit = robot_bringup.teleop_moveit:main',
            'test_ik = robot_bringup.test_ik:main',
            'validate = robot_bringup.validate_chain:main'
      
        ],
    },
)
