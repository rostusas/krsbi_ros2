from setuptools import find_packages, setup

package_name = 'krsbi_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rostu',
    maintainer_email='rostu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "object_detection_node = krsbi_pkg.object_detection_node:main",
            "robot_movement_manual_node = krsbi_pkg.robot_movement_manual_node:main",
            "manual_ui_node = krsbi_pkg.manual_node_ui:main"
        ],
    },
)
