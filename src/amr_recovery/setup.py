from setuptools import find_packages, setup


package_name = "amr_recovery"

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
    description="Heartbeat, network, and Nav2 recovery management.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "recovery_manager = amr_recovery.recovery_manager:main",
        ],
    },
)
