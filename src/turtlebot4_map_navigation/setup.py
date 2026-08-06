from glob import glob

from setuptools import find_packages, setup

package_name = 'turtlebot4_map_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rokey',
    maintainer_email='youngkim99@kakao.com',
    description='Map-based Nav2 navigation for TurtleBot 4.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'localization_initializer = '
            'turtlebot4_map_navigation.localization_initializer:main',
            'navigation_initializer = '
            'turtlebot4_map_navigation.navigation_initializer:main',
        ],
    },
)
