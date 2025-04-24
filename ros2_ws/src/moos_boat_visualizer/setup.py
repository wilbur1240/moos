from setuptools import find_packages, setup

package_name = 'moos_boat_visualizer'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/moos_boat_visualizer.launch.py']),
        ('share/' + package_name + '/urdf', ['urdf/boat.urdf.xacro']),
        ('share/' + package_name + '/meshes', ['meshes/boat.dae']),
        ('share/' + package_name + '/rviz', ['rviz/moos_boat.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'moos_boat_pose_publisher = moos_boat_visualizer.moos_boat_publisher:main',
        ],
    },
)
