from setuptools import setup

package_name = "rover_places"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Gian",
    maintainer_email="put-your-email-here@example.com",
    description="Named-location store and Nav2 goal dispatcher for the rover.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "save = rover_places.save_place:main",
            "goto = rover_places.goto_place:main",
            "list = rover_places.manage_places:list_main",
            "delete = rover_places.manage_places:delete_main",
            "rename = rover_places.manage_places:rename_main",
        ],
    },
)
