
## Add the bullet source directories to include
include_directories(
  ${Boost_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/lib/bullet/src
)

## dicephys

set(dicephys_Header_Files__
  Dice.h
  DicePhys.h
  World.h
  WorldObject.h
)
source_group("Header Files" FILES ${dicephys_Header_Files__})

set(dicephys_Source_Files__
  Dice.cpp
  DicePhys.cpp
  World.cpp
  WorldObject.cpp
)
source_group("Source Files" FILES ${dicephys_Source_Files__})

set(dicephys_Files
  ${dicephys_Header_Files__}
  ${dicephys_Source_Files__}
)

add_executable(dicephys
  ${dicephys_Files}
)

add_dependencies(dicephys BulletCollision BulletDynamics LinearMath)

target_link_libraries(dicephys
  ${Boost_LIBRARIES}
  BulletCollision
  BulletDynamics
  LinearMath
)