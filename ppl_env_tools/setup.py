from setuptools import setup

package_name = 'ppl_env_tools'

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
    maintainer_email='jmotes2@illinois.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
          'ppl_env_to_nav_map = ppl_env_tools.ppl_env_to_map.py:main'
        ],
    },
)
