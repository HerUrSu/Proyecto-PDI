#include <sl/Camera.hpp>
#include <System.h>
#include <opencv2/opencv.hpp>
#include <signal.h>
#include <iostream>
#include <vector>

using namespace std;
using namespace sl;

bool b_continue_session;

void exit_loop_handler(int s) {
    cout << "Finishing session" << endl;
    b_continue_session = false;
}

class TimestampHandler {
public:
    Timestamp last_imu_timestamp;

    bool isNew(const SensorsData::IMUData &imu_data) {
        if (imu_data.timestamp > last_imu_timestamp) {
            last_imu_timestamp = imu_data.timestamp;
            return true;
        }
        return false;
    }
};

int main(int argc, char **argv) {
    if (argc < 3 || argc > 4) {
        cerr << endl
            << "Usage: ./stereo_inertial_zed path_to_vocabulary path_to_settings (trajectory_file_name)"
            << endl;
        return 1;
    }

    string file_name;
    if (argc == 4) {
        file_name = string(argv[argc - 1]);
    }

    // Configurar señal para salir del bucle
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = exit_loop_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);

    b_continue_session = true;

    // Inicializa la cámara ZED
    Camera zed;
    InitParameters init_params;
    init_params.camera_resolution = RESOLUTION::HD720;
    init_params.coordinate_units = UNIT::METER;
    init_params.depth_mode = DEPTH_MODE::NONE;  // No necesitamos profundidad para ORB-SLAM3
    init_params.camera_fps = 15;

    ERROR_CODE err = zed.open(init_params);
    if (err != ERROR_CODE::SUCCESS) {
        cerr << "No se pudo inicializar la cámara ZED: " << toString(err) << endl;
        return 1;
    }

    int width = zed.getCameraInformation().camera_configuration.resolution.width;
    int height = zed.getCameraInformation().camera_configuration.resolution.height;

    // Configura ORB-SLAM3 en modo stereo-inertial
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::IMU_STEREO, true, 0, file_name);
    float imageScale = SLAM.GetImageScale();

    // Matrices para imágenes izquierda y derecha
    Mat left_image, right_image;
    left_image.alloc(width, height, MAT_TYPE::U8_C1);
    right_image.alloc(width, height, MAT_TYPE::U8_C1);

    cv::Mat imLeftCV, imRightCV;

    // Variables para capturar datos del IMU
    vector<ORB_SLAM3::IMU::Point> imu_measurements;
    sl::SensorsData sensors_data;
    TimestampHandler ts_handler;

    // Bucle principal
    while (b_continue_session && !SLAM.isShutDown()) {
        // Captura imágenes estéreo de la cámara ZED
        if (zed.grab() == ERROR_CODE::SUCCESS) {
            // Capturar datos del IMU
            if (zed.getSensorsData(sensors_data, TIME_REFERENCE::IMAGE) == ERROR_CODE::SUCCESS) {
                auto imu_data = sensors_data.imu;
                if (ts_handler.isNew(imu_data) && imu_data.is_available) { // Verificar si hay nuevos datos válidos
                    double imu_timestamp = imu_data.timestamp.getMilliseconds() * 1e-3;
                    imu_measurements.emplace_back(
                        imu_data.angular_velocity.x, imu_data.angular_velocity.y, imu_data.angular_velocity.z,
                        imu_data.linear_acceleration.x, imu_data.linear_acceleration.y, imu_data.linear_acceleration.z,
                        imu_timestamp);

                    // Debug: Mostrar datos del IMU
                    // cout << "IMU Timestamp: " << imu_timestamp
                    //      << " | Angular Velocity: [" << imu_data.angular_velocity.x << ", " << imu_data.angular_velocity.y << ", " << imu_data.angular_velocity.z
                    //      << "] | Linear Acceleration: [" << imu_data.linear_acceleration.x << ", " << imu_data.linear_acceleration.y << ", " << imu_data.linear_acceleration.z
                    //      << "]" << endl;
                } else {
                    cout << "IMU data not available or not new!" << endl;
                }
            }

            // Capturar imágenes
            zed.retrieveImage(left_image, VIEW::LEFT_GRAY);
            zed.retrieveImage(right_image, VIEW::RIGHT_GRAY);

            // Convertir imágenes a OpenCV
            imLeftCV = cv::Mat(left_image.getHeight(), left_image.getWidth(), CV_8UC1, left_image.getPtr<sl::uchar1>());
            imRightCV = cv::Mat(right_image.getHeight(), right_image.getWidth(), CV_8UC1, right_image.getPtr<sl::uchar1>());

            double timestamp = zed.getTimestamp(TIME_REFERENCE::IMAGE).getMilliseconds() * 1e-3;

            // Escalar imágenes si es necesario
            if (imageScale != 1.f) {
                cv::resize(imLeftCV, imLeftCV, cv::Size(), imageScale, imageScale);
                cv::resize(imRightCV, imRightCV, cv::Size(), imageScale, imageScale);
            }

            // Enviar imágenes y datos del IMU al sistema ORB-SLAM3
            if (!imu_measurements.empty()) {
                SLAM.TrackStereo(imLeftCV, imRightCV, timestamp, imu_measurements);
            } else {
                cout << "Skipping frame due to empty IMU measurements!" << endl;
            }

            imu_measurements.clear();  // Limpiar mediciones IMU procesadas
        }
    }

    // Apagar sistema ORB-SLAM3
    cout << "Cerrando sistema..." << endl;
    SLAM.Shutdown();
    zed.close();
    if (!file_name.empty()) {
        cout << "Saving trajectory to: " << file_name << endl;
        SLAM.SaveTrajectoryTUM(file_name); // Guarda la trayectoria en formato TUM
        SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory_" + file_name);
    } else {
        SLAM.SaveTrajectoryTUM("CameraTrajectory.txt"); // Nombre por defecto
        SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    }

    return 0;
}
