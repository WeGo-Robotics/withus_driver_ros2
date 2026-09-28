from glob import glob

from setuptools import find_packages, setup

package_name = 'withus_examples'

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
    maintainer='wego',
    maintainer_email='chanmgin@wego-robotics.com',
    description='Withus ROS 2 status monitoring and keyboard control examples.',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'status_monitor = withus_examples.status_monitor:main',
            'keyboard_teleop = withus_examples.keyboard_teleop:main',
        ],
    },
)
