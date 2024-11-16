cmake_minimum_required(VERSION 3.10)
project(ZED_ORB_SLAM3)

# Agregar compilación con C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Incluir directorios
include_directories(
    /home/pipeub/Dev/ORB_SLAM3/Thirdparty/DBoW2
    /home/pipeub/Dev/ORB_SLAM3/Thirdparty/g2o
    /usr/local/include/opencv4          # OpenCV
    /usr/local/zed/include              # Para el SDK de ZED
    /home/pipeub/Dev/ORB_SLAM3/include  # Ruta al include de ORB-SLAM3
)

# Agregar bibliotecas necesarias
link_directories(
    /usr/lib
    /usr/lib/x86_64-linux-gnu         # OpenCV
    /usr/local/zed/lib              # Para el SDK de ZED
    /home/pipeub/Dev/ORB_SLAM3/lib  # Ruta al lib de ORB-SLAM3
)

# Ejecutable
add_executable(zed_orbslam3 zed_orbslam3.cpp)

# Enlazar bibliotecas
target_link_libraries(zed_orbslam3
    ORB_SLAM3  # Biblioteca principal de ORB-SLAM3
    sl_core    # SDK de ZED
    sl_zed     # SDK de ZED
    opencv_core
    opencv_imgproc
    opencv_highgui
    opencv_features2d
    opencv_calib3d
    opencv_imgcodecs
)
