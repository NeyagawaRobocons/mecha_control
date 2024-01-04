from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mecha_control'

setup(
    name=package_name,
    version='0.0.0',
    # pythonのパッケージディレクトリを指定、testはテストコードを入れておくディレクトリなので除外する。
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('./launch/*.launch.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tonto2423',
    maintainer_email='tnt.kosen2423@gmail.com',
    description='A package for controlling mechanism with ROS2 and Python',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dummy_node = mecha_control.dummy_node:main',
            'debug_controller = mecha_control.debug_controller:main',
            'debug_sequence_controller = mecha_control.debug_sequence_controller:main',
        ],
    },
)