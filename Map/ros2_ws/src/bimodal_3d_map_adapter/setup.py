from setuptools import setup

package_name = 'bimodal_3d_map_adapter'

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
    description='Fallback 3D map adapter and backend bridge.',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'fallback_3d_map_adapter_node = bimodal_3d_map_adapter.fallback_3d_map_adapter_node:main',
        'map_backend_bridge_node = bimodal_3d_map_adapter.map_backend_bridge_node:main',
        'octomap_pointcloud_backend_node = bimodal_3d_map_adapter.octomap_pointcloud_backend_node:main',
    ]},
)
