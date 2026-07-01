from setuptools import setup

package_name = 'bimodal_air_adapter'

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
    description='Air exploration wrapper skeleton.',
    license='Apache-2.0',
    entry_points={'console_scripts': ['air_exploration_stub_node = bimodal_air_adapter.air_exploration_stub_node:main']},
)
