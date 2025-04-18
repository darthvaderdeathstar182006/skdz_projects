from setuptools import find_packages, setup

package_name = 'my_robo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools','websockets', 'asyncio' ],
    zip_safe=True,
    maintainer='sanathkumar-desai',
    maintainer_email='sanathkumar-desai@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "tutu_spawn = my_robo.turtle_spawner:main",
            "tutu_reach = my_robo.turtle_controller:main",
            "ard_serial = my_robo.arduino_serial:main",
            "ard_sender = my_robo.arduino_sender_serial:main",
            "esp_sender = my_robo.espwebsocket:main"
        ],
    },
)
