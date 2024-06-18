cmake_minimum_required(VERSION 3.22)
project(uniq_master_moveit)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)  # Добавляем зависимость rclpy

ament_package()

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

# Add your Python scripts
install(PROGRAMS
  scripts/follow_trajectory_publisher.py  # Указываем путь к Python-скриптам
  scripts/xyzrpw.py  # Указываем путь к Python-скриптам
  DESTINATION lib/${PROJECT_NAME}
)
