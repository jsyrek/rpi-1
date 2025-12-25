from setuptools import find_packages, setup
from glob import glob

package_name = 'mks_motor_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Kopiowanie folderu launch - zmieniamy glob!
        ('share/' + package_name + '/launch', glob('launch/*.py')),  # <-- zapewnia kopiowanie obu plików *.py
        # Kopiowanie plików URDF
        ('share/' + package_name + '/urdf', glob('mks_motor_control/URDF/*.urdf')),
        # Kopiowanie plików YAML
        ('share/' + package_name + '/config', glob('mks_motor_control/config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jarek',
    maintainer_email='jarek@todo.todo',
    description='Sterowanie dwukołowym robotem przez CAN z obsługą ROS2_control i Nav2',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'motor_driver = mks_motor_control.motor_driver:main',
            'motor_driver_speed = mks_motor_control.motor_driver_speed:main',
            'rotate_robot_360 = mks_motor_control.rotate_robot_360:main',
        ],
    },
)
