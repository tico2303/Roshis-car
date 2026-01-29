from setuptools import find_packages, setup

package_name = 'daro_ndjson_bridge'

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
    maintainer='daro',
    maintainer_email='test@gmail.com',
    description='Bridge between Daros ROS and esp32 via ndjson bridge',
    license='and registration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'ndjson_bridge = daro_ndjson_bridge.ndjson_bridge_node:main'
        ],
    },
)
