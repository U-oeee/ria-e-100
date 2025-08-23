cmake_minimum_required(VERSION 3.5)
project(e100_description)

find_package(ament_cmake REQUIRED)

# urdf, launch 디렉토리 전체 설치 (.xacro 포함)
install(
  DIRECTORY urdf launch
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.urdf"
                 PATTERN "*.urdf.xacro"
                 PATTERN "*.launch"
                 PATTERN "*.py"
)

# meshes 디렉토리에서 .stl과 .dae만 설치
install(
  DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.stl" PATTERN "*.dae"
)

ament_package()

