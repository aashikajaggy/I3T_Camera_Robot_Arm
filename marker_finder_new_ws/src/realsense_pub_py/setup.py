from setuptools import find_packages, setup

package_name = 'realsense_pub_py'

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
    maintainer='ryleighbyrne',
    maintainer_email='tianyi.hu@duke.edu',
    description='TODO: Package description',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
             'publisher_node = realsense_pub_py.marker_finder_publisher:main',
             'subscriber_node = realsense_pub_py.marker_finder_subscriber:main'
        ],
    },
)
