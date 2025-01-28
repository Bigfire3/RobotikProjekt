from setuptools import setup
import os
from glob import glob

package_name = 'my_launch_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name + '/launch', ['launch/multi_node_launch.py']),
        (os.path.join("share", package_name, "launch"), glob("my_launch_package/launch/*.py"))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fabian',
    maintainer_email='your_email@example.com',
    description='My Launch Package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
