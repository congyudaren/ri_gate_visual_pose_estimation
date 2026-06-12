from setuptools import find_packages, setup

package_name = "roi_lidar_fixture"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["tests", "tests.*"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ROI LiDAR Fixture Maintainers",
    maintainer_email="dev@example.com",
    description="Standalone static-scene upstream sensor fixture publisher for roi_lidar_corner development.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "static_scene_fixture_publisher = roi_lidar_fixture.static_scene_fixture_publisher:main",
        ],
    },
)
