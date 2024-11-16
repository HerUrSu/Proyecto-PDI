#include <sl/Camera.hpp>        // Librería para manejar la cámara ZED
#include <System.h>  // Librería de ORB-SLAM3
#include <ImuTypes.h>
#include <opencv2/opencv.hpp>  // OpenCV para manejo de imágenes

// Función para convertir las imágenes del SDK de ZED a formato OpenCV
cv::Mat slMat2cvMat(sl::Mat& input) {
    // Conversión de los datos de la cámara ZED a formato Mat de OpenCV
    return cv::Mat(input.getHeight(), input.getWidth(), CV_8UC4, input.getPtr<sl::uchar1>(sl::MEM_CPU));
}

int main(int argc, char** argv) {
    // Verificación de argumentos de entrada
    if (argc != 3) {
        std::cerr << "Uso: ./zed_orbslam3 path_to_vocabulary path_to_settings_file" << std::endl;
        return -1;
    }

    // Inicializar ORB-SLAM3 en modo estéreo-inercial
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::STEREO_IMU, true);

    // Crear objeto para interactuar con la cámara ZED
    sl::Camera zed;

    // Parámetros de inicialización de la cámara
    sl::InitParameters init_params;
    init_params.camera_resolution = sl::RESOLUTION::HD720;  // Resolución: 1280x720
    init_params.depth_mode = sl::DEPTH_MODE::NONE;          // No usar mapa de profundidad
    init_params.coordinate_units = sl::UNIT::METER;         // Unidad métrica
    init_params.camera_fps = 30;                            // Velocidad de captura

    // Abrir la cámara
    if (zed.open(init_params) != sl::ERROR_CODE::SUCCESS) {
        std::cerr << "Error al abrir la cámara ZED" << std::endl;
        return -1;
    }

    // Configurar buffers de imágenes y datos IMU
    sl::Mat left_image, right_image;
    sl::SensorsData sensors_data;
    cv::Mat cv_left, cv_right;

    // Ciclo principal de captura y procesamiento
    while (true) {
        // Capturar imágenes
        if (zed.grab() == sl::ERROR_CODE::SUCCESS) {
            // Obtener imágenes izquierda y derecha
            zed.retrieveImage(left_image, sl::VIEW::LEFT);
            zed.retrieveImage(right_image, sl::VIEW::RIGHT);

            // Convertir imágenes a formato OpenCV
            cv_left = slMat2cvMat(left_image);
            cv_right = slMat2cvMat(right_image);

            // Extraer datos del IMU
            zed.getSensorsData(sensors_data, sl::TIME_REFERENCE::IMAGE);

            // Crear un objeto IMU del tipo de ORB-SLAM3
            ORB_SLAM3::IMU::Point imu_data(
                sensors_data.imu.angular_velocity[0], sensors_data.imu.angular_velocity[1],
                sensors_data.imu.angular_velocity[2], sensors_data.imu.linear_acceleration[0],
                sensors_data.imu.linear_acceleration[1], sensors_data.imu.linear_acceleration[2],
                sensors_data.getTimestamp(sl::TIME_REFERENCE::IMAGE) * 1e-9);

            // Procesar datos en ORB-SLAM3
            SLAM.TrackStereoWithIMU(cv_left, cv_right, imu_data, sensors_data.getTimestamp(sl::TIME_REFERENCE::IMAGE) * 1e-9);
        }

        // Si deseas agregar una opción para salir:
        char key = cv::waitKey(1);
        if (key == 27) break;  // Salir si se presiona "ESC"
    }

    // Apagar el sistema ORB-SLAM3 y cerrar la cámara
    SLAM.Shutdown();
    zed.close();

    return 0;
}

