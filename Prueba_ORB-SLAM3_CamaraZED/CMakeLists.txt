cmake_minimum_required(VERSION 3.10)
project(ZED_ORB_SLAM3)

# Agregar compilación con C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Variables para facilitar personalización
set(ORB_SLAM3_DIR /home/gabo/Dev/ORB_SLAM3)   # Ruta relativa al directorio del proyecto
set(OPENCV_INCLUDE_DIR /usr/local/include/opencv4)    # OpenCV include
set(ZED_INCLUDE_DIR /usr/local/zed/include)          # ZED SDK include
set(ZED_LIBRARY_DIR /usr/local/zed/lib)              # ZED SDK lib
set(OPENCV_LIB_DIR /usr/lib/x86_64-linux-gnu)        # OpenCV lib (modificar según el sistema)

# Mensajes para verificar configuración
message(STATUS "ORB-SLAM3 Directory: ${ORB_SLAM3_DIR}")
message(STATUS "OpenCV Include Directory: ${OPENCV_INCLUDE_DIR}")
message(STATUS "ZED Include Directory: ${ZED_INCLUDE_DIR}")

# Incluir directorios
include_directories(
    ${ORB_SLAM3_DIR}/Thirdparty/DBoW2
    ${ORB_SLAM3_DIR}/Thirdparty/DBoW2/DBoW2
    ${ORB_SLAM3_DIR}/Thirdparty/g2o
    ${ORB_SLAM3_DIR}/Thirdparty/g2o/g2o
    ${ORB_SLAM3_DIR}/Thirdparty/Sophus/sophus
    /usr/include/eigen3 
    ${OPENCV_INCLUDE_DIR}      # OpenCV
    ${ZED_INCLUDE_DIR}         # Para el SDK de ZED
    ${ORB_SLAM3_DIR}/include   # Include de ORB-SLAM3
    /usr/local/cuda/include
    ${Pangolin_INCLUDE_DIRS}   # Añade Pangolin
    
)

# Establecer la política CMP0072 para preferir GLVND
cmake_policy(SET CMP0072 NEW)


find_package(OpenGL REQUIRED)
find_package(Pangolin REQUIRED)

# Buscar las bibliotecas de ZED SDK
# find_library(SL_CORE_LIB sl_core PATHS /usr/local/zed/lib)
# find_library(SL_ZED_LIB sl_zed PATHS /usr/local/zed/lib)
find_package(ZED 4 REQUIRED)

if(NOT ZED_FOUND)
    message(FATAL_ERROR "ZED SDK not found. Please ensure it is installed correctly.")
endif()

# if(NOT SL_CORE_LIB)
#     message(FATAL_ERROR "sl_core library not found in /usr/local/zed/lib")
# endif()

# if(NOT SL_ZED_LIB)
#     message(FATAL_ERROR "sl_zed library not found in /usr/local/zed/lib")
# endif()

message(STATUS "ZED Libraries: ${ZED_LIBRARIES}")
message(STATUS "ZED Include Directories: ${ZED_INCLUDE_DIRS}")


# Agregar bibliotecas necesarias
link_directories(
    ${OPENCV_LIB_DIR}          # OpenCV
    ${ZED_LIBRARY_DIR}         # Para el SDK de ZED
    ${ORB_SLAM3_DIR}/lib       # Lib de ORB-SLAM3
    /usr/local/cuda/lib64
)
include_directories(/usr/local/cuda/include)

# Ejecutable
add_executable(stereo_inertial_zed stereo_inertial_zed.cc)

# Enlazar bibliotecas
target_link_libraries(stereo_inertial_zed
    ${OPENGL_LIBRARIES}
    ${ORB_SLAM3_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so  # Biblioteca DBoW2
    ${ORB_SLAM3_DIR}/Thirdparty/g2o/lib/libg2o.so      # Biblioteca g2o
    ORB_SLAM3                                          # Biblioteca principal de ORB-SLAM3
    ${ZED_LIBRARIES}
    opencv_core
    opencv_imgproc
    opencv_highgui
    opencv_features2d
    opencv_calib3d
    opencv_imgcodecs
    ${Pangolin_LIBRARIES}
)