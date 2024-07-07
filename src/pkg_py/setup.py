from setuptools import find_packages, setup

package_name = 'pkg_py'

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
    maintainer='shonde',
    maintainer_email='shonde@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "py_node = pkg_py.first:main",
            "robot_new = pkg_py.robot_new:main",
            "smart = pkg_py.smartPhone:main",
            "camera_publisher = pkg_py.computer_vision_node:main"
        ],
    },
)
