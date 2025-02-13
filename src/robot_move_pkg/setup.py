from setuptools import find_packages, setup

package_name = 'robot_move_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jhj',
    maintainer_email='happyijun@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'block = robot_move_pkg.block:main',
            'gear = robot_move_pkg.gear:main',
            'jenga = robot_move_pkg.jenga:main',
            'cup = robot_move_pkg.cup:main',
            'cup01 = robot_move_pkg.cup01:main',
            'cup02 = robot_move_pkg.cup02:main',
        ],
    },
)
