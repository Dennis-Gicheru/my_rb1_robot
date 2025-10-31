import os
from glob import glob
from setuptools import setup
from setuptools import find_packages


package_name = 'my_rb1_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['tests', 'tests.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dennis_Gicheru',
    maintainer_email='dennisgicheru254@gmail.com',
    description='my cylinder robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
         'console_scripts': [
        ],
    },
)
