from setuptools import setup

package_name = 'factory_sim'

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
    maintainer='ingui',
    maintainer_email='ingui@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'workstation_node = factory_sim.workstation_node:main',
            'workstation_node_client = factory_sim.workstation_node_test_client:main', 
            'agv_agent = factory_sim.agv_agent:main',
            'dummy_allocator = factory_sim.dummy_allocator:main',
        ],
    },
)
