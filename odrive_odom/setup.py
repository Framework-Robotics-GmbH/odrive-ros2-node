from setuptools import setup

package_name = 'odrive_odom'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/odrive_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ronald Luc',
    maintainer_email='ubuntu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom = odrive_odom.odom:main',
            'init = odrive_odom.init:main',
        ],
    },
)
