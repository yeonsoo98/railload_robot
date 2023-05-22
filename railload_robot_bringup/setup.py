import os
from glob import glob

from setuptools import setup

pkg_name = 'railload_robot_bringup'

setup(
    name=pkg_name,
    version='0.0.0',
    packages=[pkg_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + pkg_name]),
        ('share/' + pkg_name, ['package.xml']),
        (os.path.join('share/', pkg_name, 'launch'), glob('launch/*')),
        (os.path.join('share/', pkg_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yeonsoo98',
    maintainer_email='yeonso981@naver.com',
    description='Railload Robot common bringup package.',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'driving = '+ pkg_name + '.driving:main',
            'keyboard_command='+ pkg_name + '.keyboard_command:main',
            'joystick_command='+ pkg_name + '.joystick_command:main',
            'tracker='+ pkg_name + '.tracker:main',
        ],
    },
)
