cmake_minimum_required(VERSION 3.10)
project(explorer_r2_sim)

find_package(ament_cmake REQUIRED)

# Pure data package — no compiled targets. Just install the assets so
# `ros2 launch explorer_r2_sim cave.launch.py` can find them via
# get_package_share_directory().
install(DIRECTORY launch config rviz worlds models scenarios
  DESTINATION share/${PROJECT_NAME}
)

# Python scripts go to lib/${PROJECT_NAME}/ so they're runnable via
# `ros2 run explorer_r2_sim <script>`.
install(PROGRAMS
  scripts/gt_to_path.py
  scripts/lidar_field_adapter.py
  scripts/record_run.sh
  scripts/eval.sh
  scripts/analyze_bag.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
