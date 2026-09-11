from glob import glob
from setuptools import setup

setup(
    name='aprl_robot_sim', version='1.1.0', packages=['aprl_robot_sim'],
    data_files=[('share/ament_index/resource_index/packages', ['resource/aprl_robot_sim']),
                ('share/aprl_robot_sim', ['package.xml']),
                ('share/aprl_robot_sim/examples', glob('examples/*.json')),
                ('share/aprl_robot_sim/urdf', glob('urdf/*.urdf')),
                ('share/aprl_robot_sim/meshes', glob('meshes/*')),
                ('share/aprl_robot_sim/rviz', glob('rviz/*.rviz'))],
    install_requires=['setuptools'], zip_safe=False,
    maintainer='APRL', maintainer_email='aprl@example.invalid', license='MIT',
    description='ROS 2 Jazzy controls and live examples for the APRL simulator',
    entry_points={'console_scripts': [f'{name} = aprl_robot_sim.{name}:main'
                                     for name in ('teleop', 'arm_teleop', 'route', 'verify')]},
)
