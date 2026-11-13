from glob import glob
from setuptools import find_packages, setup

package_name = 'ipex_mission'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/launch', glob('launch/*_launch.py')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Cosimo Sgambelluri',
    maintainer_email='cosimo.sgambelluri@edu.unifi.it',
    description='Nasa\'s IPEX rover simulation. The rover selects the best area to dig and proceeds to automatically dig and haul regolith.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'mission = ipex_mission.mission_node:main',
            'tag_localizer = ipex_mission.tag_localizer:main',
            'clahe = ipex_mission.clahe_node:main',
        ],
    },
)
