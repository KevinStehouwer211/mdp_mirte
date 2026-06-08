from glob import glob
from setuptools import setup

ros_package_name = 'gh_twin_data_storage'
python_package_name = 'gh_twin_data_storage_nodes'

setup(
    name=python_package_name,
    version='0.1.0',
    packages=[python_package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + ros_package_name]),
        ('share/' + ros_package_name, ['package.xml']),
        ('share/' + ros_package_name + '/launch',
         glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='MIRTE Team',
    author_email='maintainer@example.com',
    maintainer='MIRTE Team',
    maintainer_email='maintainer@example.com',
    description='Typed data storage nodes for gh_twin',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'typedb_node = gh_twin_data_storage_nodes.typedb_node:main',
            'sql_db_node = gh_twin_data_storage_nodes.sql_db_node:main',
            'nosql_db_node = gh_twin_data_storage_nodes.nosql_db_node:main',
            'pddl_planner_node = gh_twin_data_storage_nodes.pddl_planner_node:main',
            'plan_executor_node = gh_twin_data_storage_nodes.plan_executor_node:main',
        ],
    },
)
