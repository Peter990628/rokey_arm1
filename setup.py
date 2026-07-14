from setuptools import find_packages, setup

package_name = 'rokey'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        (
            'share/' + package_name,
            ['package.xml'],
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='PETER',
    maintainer_email='haebyuk35@gmail.com',
    description='Pharmacy assistant robot control nodes',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            
            'task_manager_bridge = rokey.task_manager_bridge:main',
            'manipulator_test_2 = rokey.manipulator_test_2:main',
            'pour_pills_rotate = rokey.pour_pills_rotate:main',
            'test_spin_lid = rokey.test_spin_lid:main',
            'test_storage_grasp = rokey.test_storage_grasp:main',
            'test_tcp = rokey.test_tcp:main',
            'go_home = rokey.go_home:main',
            'test1 = rokey.test1:main',
            'opener = rokey.opener:main',
            'robot_total = rokey.robot_total:main',
        ],
    },
)
