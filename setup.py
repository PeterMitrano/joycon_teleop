from setuptools import setup

package_name = 'joycon_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Peter Mitrano',
    author_email='pmitrano@umich.edu',
    keywords=['ROS'],
    description='ROS2 Python package for joycon teleop of robots with end-effector velocity controllers',
    entry_points={
        'console_scripts': [
            'joycon_teleop_node = joycon_teleop.joycon_teleop_node:main'
        ],
    },
)
