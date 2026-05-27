from setuptools import find_packages, setup

package_name = 'unknown_px4_offboard_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ghosts',
    maintainer_email='kasiphat.watt@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
    'console_scripts': [
        'offboard_takeoff = unknown_px4_offboard_py.offboard_takeoff:main',
        'px4_teleop_offboard = unknown_px4_offboard_py.px4_teleop_offboard:main',
    ],
},
)
