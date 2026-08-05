from setuptools import find_packages, setup


package_name = "emergency_alert"

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
    zip_safe=True,
    maintainer="multi_amr_aed team",
    maintainer_email="team@example.com",
    description="Emergency audio alert node for TurtleBot4.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "siren = emergency_alert.siren_node:main",
        ],
    },
)

