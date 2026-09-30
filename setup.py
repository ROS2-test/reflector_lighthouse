from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'reflector_lighthouse'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools', 'pyyaml', 'numpy'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Reflective marker (lighthouse) localization, GOZ-Shuttle style',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'lighthouse_node = reflector_lighthouse.lighthouse_node:main',
            'marker_mapper_node = reflector_lighthouse.marker_mapper_node:main',
        ],
    },
)
