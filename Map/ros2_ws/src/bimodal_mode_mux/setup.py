from setuptools import setup

package_name = 'bimodal_mode_mux'

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
    description='Temporary mode mux for bimodal planning.',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'bimodal_mode_mux_node = bimodal_mode_mux.bimodal_mode_mux_node:main',
        'simple_mode_commander_node = bimodal_mode_mux.simple_mode_commander_node:main',
    ]},
)
