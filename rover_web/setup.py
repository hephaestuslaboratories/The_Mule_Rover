from setuptools import setup

package_name = "rover_web"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    package_data={package_name: ["static/*"]},
    include_package_data=True,
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="Gian",
    maintainer_email="put-your-email-here@example.com",
    description="Phone-browser cockpit for the rover.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "server = rover_web.server:main",
        ],
    },
)
