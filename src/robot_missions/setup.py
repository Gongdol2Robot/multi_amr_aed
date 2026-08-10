from setuptools import find_packages, setup


package_name = "robot_missions"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    extras_require={"test": ["pytest"]},
    zip_safe=True,
    maintainer="multi_amr_aed team",
    maintainer_email="team@example.com",
    description="Nav2 mission executor for the Multi-AMR AED system.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "mission_executor = robot_missions.mission_executor:main",
            "search_and_detect = robot_missions.search_and_detect_node:main",
        ],
    },
)
