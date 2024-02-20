from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'withus_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('lib/' + package_name, ['can_bringup.sh']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))  # Include the shell script
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='WeGo',
    maintainer_email='go.support@wego-robotics.com',
    description='CAN controller',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'send_command = withus_control.send_command:main',
            'recv_status = withus_control.status_recv:main',
            'pyqt_control = withus_control.pyqt_control:main',
        ],
    },
)

