from glob import glob

from setuptools import setup

package_name = "ur5e_pose_commander"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/urdf", glob("urdf/*.xacro")),
        ("share/" + package_name + "/srdf", glob("srdf/*.xacro")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/isaac", glob("isaac/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="v-b-naik",
    maintainer_email="vishalnaik1005@gmail.com",
    description="Command the UR5e end effector to a goal position and orientation via MoveIt.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "pose_commander = ur5e_pose_commander.pose_commander:main",
        ],
    },
)
