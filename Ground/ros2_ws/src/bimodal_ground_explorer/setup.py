from setuptools import setup

package_name = 'bimodal_ground_explorer'

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
    description='Ground 3D frontier baseline.',
    license='Apache-2.0',
    entry_points={'console_scripts': ['ground_3d_frontier_node = bimodal_ground_explorer.ground_3d_frontier_node:main']},
)
