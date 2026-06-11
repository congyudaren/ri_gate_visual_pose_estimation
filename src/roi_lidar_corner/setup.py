from setuptools import find_packages, setup

setup(
    name="roi_lidar_corner",
    version="0.1.0",
    packages=find_packages(exclude=["test", "test.*"]),
    install_requires=["setuptools"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/roi_lidar_corner"]),
        ("share/roi_lidar_corner", ["package.xml"]),
    ],
    entry_points={
        "console_scripts": [
            "lidar_projection_exposure_capture = roi_lidar_corner.lidar_projection_exposure_capture:main",
            "offline_front_face_validation = roi_lidar_corner.offline_front_face_validation:main",
        ],
    },
)
