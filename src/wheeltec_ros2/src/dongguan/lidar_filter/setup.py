from setuptools import find_packages, setup

package_name = "lidar_filter"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="wouter",
    maintainer_email="3747759+wouter1602@users.noreply.github.com",
    description="TODO: Package description",
    license="Apache-2.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "talker = lidar_filter.publisher_member_function:main",
            "listener = lidar_filter.subscriber_member_function:main",
            "lidar_filter = lidar_filter.lidar_filter_node:main",
        ],
    },
)
