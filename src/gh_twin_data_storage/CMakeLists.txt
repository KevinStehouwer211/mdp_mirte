cmake_minimum_required(VERSION 3.8)
project(gh_twin_data_storage)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS gh_twin_data_storage_nodes/typedb_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME typedb_node
)

install(PROGRAMS gh_twin_data_storage_nodes/sql_db_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME sql_db_node
)

install(PROGRAMS gh_twin_data_storage_nodes/nosql_db_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME nosql_db_node
)

install(PROGRAMS gh_twin_data_storage_nodes/pddl_planner_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME pddl_planner_node
)

install(PROGRAMS gh_twin_data_storage_nodes/plan_executor_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME plan_executor_node
)

ament_python_install_package(gh_twin_data_storage_nodes)

ament_package()
