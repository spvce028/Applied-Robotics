from setuptools import find_packages, setup

package_name = 'depalletizing_pick_place'

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
    maintainer='robotics',
    maintainer_email='robotics@todo.todo',
    description='Pick-and-place primitives and execution nodes for depalletizing robot.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'depalletizing_node = depalletizing_pick_place.main_node:main',
        ],
    },
)