from setuptools import setup
import os
from glob import glob

package_name = 'nyurdt_state_test'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jr-buntu',
    maintainer_email='jr-buntu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'single-machine=nyurdt_state_test.single_state_machine:main',
            'nested-machine=nyurdt_state_test.nested_state_machine:main'
        ],
    },
)
