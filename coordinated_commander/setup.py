from setuptools import setup

package_name = 'coordinated_commander'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jmotes',
    maintainer_email='jmotes@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'example_waypoint_follower = coordinated_commander.example_waypoint_follower:main',
            'example_group_waypoint_follower = coordinated_commander.example_group_waypoint_follower:main',
            'ppl_group_path_follower = coordinated_commander.follow_ppl_group_path:main'
        ],
    },
)
