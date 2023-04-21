import os
from glob import glob
from setuptools import setup

package_name = 'monitor'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alejandro Corpas Calvo',
    maintainer_email='al.corpas@alumnos.upm.es',
    description='The monitoring package',
    license='BSD-3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'execution_monitor = monitor.execution_monitor:main'
        ],
    },
)
