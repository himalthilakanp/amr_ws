from setuptools import setup

package_name = 'amr_keyboard_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Keyboard control node for AMR via Arduino',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
             'keyboard_motor_control = amr_keyboard_control.keyboard_motor_control:main',
        'serial_bridge = amr_keyboard_control.serial_bridge:main',
        'serial_test = amr_keyboard_control.serial_test:main',
        'simple_keyboard = amr_keyboard_control.simple_keyboard:main',
        ],
    },
)
