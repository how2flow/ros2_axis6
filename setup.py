from setuptools import setup
from glob import glob

package_name = 'axis6'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch/', glob('launch/*.py')),
        ('share/' + package_name + '/meshes/', glob('meshes/*.dae')),
        ('share/' + package_name + '/rviz/', glob('rviz/*.rviz')),
        ('share/' + package_name + '/urdf/', glob('urdf/*.urdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='how2flow',
    maintainer_email='steve@how2flow.net',
    description='Control 6-axis robot with pca9685 (motor: MG996R)',
    license='Apache 2.0, The MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'commander = axis6.commander:main',
            'operator = axis6.operator:main',
            'state_publisher = axis6.state_publisher:main',
        ],
    },
)
