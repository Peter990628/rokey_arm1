from setuptools import find_packages, setup

package_name = 'rokey'

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
    maintainer='PETER',
    maintainer_email='haebyuk35@gmail.com',
    description='move_node',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'move = rokey.move:main',
            'grip_test = rokey.grip_test:main',
            'gear = rokey.gear:main',
            'go_home = rokey.drug_peter.go_home:main',
            'paper_bag_test = rokey.drug_peter.paper_bag_test:main',
            'task_manager_bridge = rokey.drug_peter.task_manager_bridge:main',
            'manipulator_test_2 = rokey.manipulator_test_2:main',
            'pour_pills_rotate = rokey.pour_pills_rotate:main',
            'test1 = rokey.test1:main',
        ],
    },
)
