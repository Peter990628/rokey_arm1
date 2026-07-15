from setuptools import find_packages, setup

package_name = "rokey"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        (
            "share/" + package_name,
            ["package.xml"],
        ),
    ],
    install_requires=["setuptools", "requests"],
    zip_safe=True,
    maintainer="PETER",
    maintainer_email="haebyuk35@gmail.com",
    description="Pharmacy assistant robot control and backend bridge nodes",
    url="https://github.com/Peter990628/rokey_arm1",
    license="Apache-2.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "final = rokey.final:main",
            "task_manager_bridge_final = rokey.task_manager_bridge_final:main",
        ],
    },
)
