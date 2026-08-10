from setuptools import find_packages, setup


package_name = "mission_manager"

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
    description="Dynamic role assignment for the Multi-AMR AED system.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "mission_manager = mission_manager.manager_node:main",
        ],
    },
)
