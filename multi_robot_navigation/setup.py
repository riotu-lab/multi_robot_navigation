from setuptools import setup 
import os
from glob import glob

package_name = 'multi_robot_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share/' + package_name,'launch'), glob('launch/*.launch.py')),
        (os.path.join('share/' + package_name,'config/'), glob('./config/*')),
        (os.path.join('share/' + package_name,'models/'), glob('./models/*')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='FatimahAlahmed',
    maintainer_email='falahmed@psu.edu.sa',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [


        ],
    },
)
