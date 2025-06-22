from setuptools import find_packages, setup

package_name = "webrtc_bridge"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=[
        "setuptools",
        "aiortc >=1.13.0, <2",
        "aiohttp >=3.12.13, <4",
    ],
    zip_safe=True,
    maintainer="root",
    maintainer_email="a@tatarinov.co",
    description="TODO: Package description",
    license="TODO: License declaration",
    # tests_require=['pytest'],
    entry_points={
        "console_scripts": [
            "webrtc_bridge_local_node = webrtc_bridge.webrtc_bridge_local_node:main"
        ],
    },
)
