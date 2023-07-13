cmake_minimum_required(VERSION 2.8.3)
project(dron_cinematico)

find_package(catkin REQUIRED COMPONENTS
  roscpp 
  rospy 
  std_msgs
  tf
  hector_uav_msgs #Required for "enable_motors" service
)

catkin_package( CATKIN_DEPENDS roscpp rospy std_msgs )

include_directories(
  ${catkin_INCLUDE_DIRS}
)

include_directories(${catkin_INCLUDE_DIRS})


#Se realiza una circunferencia
add_executable(movimiento_circular src/movimiento_circular/movimiento_circular.cpp)
target_link_libraries(movimiento_circular ${catkin_LIBRARIES})

#Se realiza una lemniscata (infinito)
add_executable(lemniscata_helice src/lemniscata_helice/lemniscata_helice.cpp)
target_link_libraries(lemniscata_helice ${catkin_LIBRARIES})

#Se realiza una circunferencia
add_executable(lemniscata_helice_grafica src/lemniscata_helice_grafica/lemniscata_helice_grafica.cpp)
target_link_libraries(lemniscata_helice_grafica ${catkin_LIBRARIES})

#Se realiza una circunferencia
add_executable(lider_seguidor src/lider_seguidor/lider_seguidor.cpp)
target_link_libraries(lider_seguidor ${catkin_LIBRARIES})

#Se realiza una circunferencia
add_executable(lider_seguidor_grafica src/lider_seguidor_grafica/lider_seguidor_grafica.cpp)
target_link_libraries(lider_seguidor_grafica ${catkin_LIBRARIES})

