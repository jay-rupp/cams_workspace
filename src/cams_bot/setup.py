from setuptools import find_packages, setup

package_name = 'cams_bot'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jason',
    maintainer_email='jason@todo.com',
    description='CAMS Bot control nodes',
    license='MIT',
    entry_points={
        'console_scripts': [
            'joystick_node = cams_bot.joystick_node:main',
            'control_node  = cams_bot.control_node:main',
        ],
    },
)
