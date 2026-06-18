from setuptools import setup
import os
from glob import glob

package_name = 'robot_2_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),

        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'materials/scripts'), glob('materials/scripts/*')),
        (os.path.join('share', package_name, 'materials/textures'), glob('materials/textures/*')),

        (os.path.join('share', package_name, 'models/saban_floor'), glob('models/saban_floor/*.*')),
        (os.path.join('share', package_name, 'models/saban_floor/materials/scripts'), glob('models/saban_floor/materials/scripts/*')),
        (os.path.join('share', package_name, 'models/saban_floor/materials/textures'), glob('models/saban_floor/materials/textures/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='author',
    maintainer_email='todo@todo.com',
    description='The ' + package_name + ' package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'serial_bridge = robot_2_description.serial_bridge:main',
            'detect_line = robot_2_description.detect_line:main',
            'detect_laptop = robot_2_description.detect_laptop:main',
            'detect_line_new = robot_2_description.detect_line_new:main',
        ],
    },
)
