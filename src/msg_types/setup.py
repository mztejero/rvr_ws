from setuptools import find_packages, setup

package_name = 'msg_types'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mztejero',
    maintainer_email='mztejero@todo.todo',
    description='Custom message types for ROS 2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={},
)