cmake_minimum_required(VERSION 3.8)
project(franka_mover)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dipendenze
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(franka_description REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(Python3 REQUIRED)


# Install project launch files
install(
  DIRECTORY
    launch config srdf rviz
  DESTINATION share/${PROJECT_NAME}
)

# Aggiungi gli script Python
install(
  PROGRAMS
    scripts/basic/franka_mover_target_point_pose.py  # muove robot in un punto target con certa posa rispetto fr_link0
    
    scripts/basic/franka_mover_topic_PointStamped_pose.py  # muove robot in coordinate ascoltate da topic tipo PointStamped e posso modificare l'orientamento manualmente (no trasformata da sistema visione a robot: Topic trasmette in Frame di sistema visione)
    
    scripts/basic/franka_mover_topic_PoseStamped.py  # muove robot secondo coordinate e orientamento da topic tipo PoseStamped. (no trasformata da sistema visione a robot: Topic trasmette in Frame di sistema visione)
    
    scripts/0_realsense_starter.py  # nodo che avvia la realsense
    
    scripts/1_aruco_calibrate.py # calibrazione fatta con aruco (salva fr3_camera_calibration.yaml che contiene la posizione del frame camera_link rispetto alla base del robot fr3_link0
    
    scripts/3_apple_recognizer_noCv2_PoseStamped.py # python program che riconosce le mele e invia coordinate e posa topic PoseStamped. Cv2 è commentato. per vedere i frame della videocamera in tempo reale scommentare tutte righe con "Cv2."

    scripts/basic/franka_mover_topic_PoseStamped_transformed.py # riceve da /apple_coordinates_realsense [PoseStamped] coordinate mela, le trasforma in sistema riferimento manipolatore robot, le invia a /apple_coordinates_robot [PointStamped] e muove il robot in questo punto  
    
    scripts/2_franka_mover_topic_PoseStamped_transformed_grasp.py # riceve da /apple_coordinates_realsense [PoseStamped] coordinate mela, le trasforma in sistema riferimento manipolatore robot, le invia a /apple_coordinates_robot [PointStamped] e muove il robot in questo punto. Afferra la mela.
    
    scripts/basic/franka_gripper.py #aziona il gripper del FR3
    
    scripts/basic/franka_grasper.py #aziona il gripper PER IL GRASP del FR3
    
    scripts/basic/prova_trasform.py  # trasformazione statica tra camera_link e fr3_link0
    
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(${PROJECT_NAME}_srdf_tests test/srdf_tests.py)
endif()

  
ament_package()
