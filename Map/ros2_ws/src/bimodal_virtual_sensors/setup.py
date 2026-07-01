from setuptools import setup

package_name = 'bimodal_virtual_sensors'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nuaa',
    maintainer_email='nuaa@example.com',
    description='Virtual sensors for the bimodal 3D planning baseline.',
    license='Apache-2.0',
    entry_points={'console_scripts': ['virtual_sensor_node = bimodal_virtual_sensors.virtual_sensor_node:main']},
)
