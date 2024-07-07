from setuptools import find_packages, setup

package_name = 'cv_node'

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
            'img_publisher = cv_node.webcam_pub:main',
            'img_subscriber = cv_node.webcam_sub:main',
            'gesture_ = cv_node.gesture:main',
            'gesture_publisher = cv_node.gesture_publisher:main',
            'gesture_subscriber = cv_node.gesture_subscriber:main',

        ],
    },
)
