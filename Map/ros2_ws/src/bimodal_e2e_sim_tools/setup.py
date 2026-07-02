from setuptools import setup

package_name = 'bimodal_e2e_sim_tools'

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
    description='Closed-loop simulation tools for bimodal 3D planner integration tests.',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'fake_path_executor_node = bimodal_e2e_sim_tools.fake_path_executor_node:main',
        'e2e_metrics_logger_node = bimodal_e2e_sim_tools.e2e_metrics_logger_node:main',
        'visual_tf_guard_node = bimodal_e2e_sim_tools.visual_tf_guard_node:main',
        'demo_explainability_overlay_node = bimodal_e2e_sim_tools.demo_explainability_overlay_node:main',
    ]},
)
