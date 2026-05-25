from setuptools import setup

package_name = 'gh_twin_data_storage'

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
    author='MIRTE Team',
    author_email='maintainer@example.com',
    maintainer='MIRTE Team',
    maintainer_email='maintainer@example.com',
    description='Typed data storage nodes for gh_twin',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'typedb_node = gh_twin_data_storage.typedb_node:main',
            'sql_db_node = gh_twin_data_storage.sql_db_node:main',
            'nosql_db_node = gh_twin_data_storage.nosql_db_node:main',
            'pddl_manager_node = gh_twin_data_storage.pddl_manager_node:main',
        ],
    },
)
