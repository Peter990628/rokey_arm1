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
            'pour_pills_rotate = rokey.pour_pills_rotate:main',
            'test1 = rokey.test1:main',
            'task_manager_bridge.py = rokey.task_manager_bridge:main',
            'task_manager_test.py = rokey.task_manager_test:main',
            'task_manager_fianl.py = rokey.task_manager_fianl:main',
            ],
        },
    )
