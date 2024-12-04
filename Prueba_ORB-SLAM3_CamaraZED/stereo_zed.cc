#include <sl/Camera.hpp>
#include <System.h>
#include <opencv2/opencv.hpp>
#include <signal.h>
#include <iostream>

using namespace std;
using namespace sl;

bool b_continue_session;

void exit_loop_handler(int s) {
    cout << "Finishing session" << endl;
    b_continue_session = false;
}

int main(int argc, char **argv) {
    if (argc < 3 || argc > 4) {
        cerr << endl
             << "Usage: ./stereo_zed path_to_vocabulary path_to_settings (trajectory_file_name)"
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

    sl::CalibrationParameters calibration_params = zed.getCameraInformation().camera_configuration.calibration_parameters;

    // Valores de la cámara izquierda (Camera1)
    std::cout << "Camera1.fx: " << calibration_params.left_cam.fx << std::endl;
    std::cout << "Camera1.fy: " << calibration_params.left_cam.fy << std::endl;
    std::cout << "Camera1.cx: " << calibration_params.left_cam.cx << std::endl;
    std::cout << "Camera1.cy: " << calibration_params.left_cam.cy << std::endl;
    std::cout << "Camera1.k1: " << calibration_params.left_cam.disto[0] << std::endl;
    std::cout << "Camera1.k2: " << calibration_params.left_cam.disto[1] << std::endl;
    std::cout << "Camera1.p1: " << calibration_params.left_cam.disto[2] << std::endl;
    std::cout << "Camera1.p2: " << calibration_params.left_cam.disto[3] << std::endl;
    std::cout << "Camera1.k3: " << calibration_params.left_cam.disto[4] << std::endl;

    // Valores de la cámara derecha (Camera2)
    std::cout << "Camera2.fx: " << calibration_params.right_cam.fx << std::endl;
    std::cout << "Camera2.fy: " << calibration_params.right_cam.fy << std::endl;
    std::cout << "Camera2.cx: " << calibration_params.right_cam.cx << std::endl;
    std::cout << "Camera2.cy: " << calibration_params.right_cam.cy << std::endl;
    std::cout << "Camera2.k1: " << calibration_params.right_cam.disto[0] << std::endl;
    std::cout << "Camera2.k2: " << calibration_params.right_cam.disto[1] << std::endl;
    std::cout << "Camera2.p1: " << calibration_params.right_cam.disto[2] << std::endl;
    std::cout << "Camera2.p2: " << calibration_params.right_cam.disto[3] << std::endl;
    std::cout << "Camera2.k3: " << calibration_params.right_cam.disto[4] << std::endl;

    // Obtener parámetros de traslación y rotación
    auto translation = calibration_params.stereo_transform.getTranslation();
    auto rotation = calibration_params.stereo_transform.getRotationVector();

    // Imprimir los valores de traslación
    std::cout << "Translation tx: " << translation[0] << " meters" << std::endl;
    std::cout << "Translation ty: " << translation[1] << " meters" << std::endl;
    std::cout << "Translation tz: " << translation[2] << " meters" << std::endl;

    // Imprimir los valores de rotación (en radianes)
    std::cout << "Rotation rx: " << rotation[0] << " radians" << std::endl;
    std::cout << "Rotation ry: " << rotation[1] << " radians" << std::endl;
    std::cout << "Rotation rz: " << rotation[2] << " radians" << std::endl;

    int width = zed.getCameraInformation().camera_configuration.resolution.width;
    int height = zed.getCameraInformation().camera_configuration.resolution.height;

    // Configura ORB-SLAM3 en modo estéreo
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::STEREO, true, 0, file_name);
    float imageScale = SLAM.GetImageScale();

    // Matrices para imágenes izquierda y derecha
    Mat left_image, right_image;
    left_image.alloc(width, height, MAT_TYPE::U8_C1);
    right_image.alloc(width, height, MAT_TYPE::U8_C1);

    cv::Mat imLeftCV, imRightCV;

    // Bucle principal
    while (b_continue_session && !SLAM.isShutDown()) {
        // Captura imágenes estéreo de la cámara ZED
        if (zed.grab() == ERROR_CODE::SUCCESS) {
            zed.retrieveImage(left_image, VIEW::LEFT_GRAY);
            zed.retrieveImage(right_image, VIEW::RIGHT_GRAY);

            // Convierte las imágenes de ZED a OpenCV
            imLeftCV = cv::Mat(left_image.getHeight(), left_image.getWidth(), CV_8UC1, left_image.getPtr<sl::uchar1>());
            imRightCV = cv::Mat(right_image.getHeight(), right_image.getWidth(), CV_8UC1, right_image.getPtr<sl::uchar1>());

            double timestamp = zed.getTimestamp(TIME_REFERENCE::IMAGE).getMilliseconds() * 1e-3;
            cv::imshow("ZED View", imLeftCV);
            // Escalar imágenes si es necesario
            if (imageScale != 1.f) {
                cv::resize(imLeftCV, imLeftCV, cv::Size(), imageScale, imageScale);
                cv::resize(imRightCV, imRightCV, cv::Size(), imageScale, imageScale);
            }

            // Enviar imágenes al sistema ORB-SLAM3
            SLAM.TrackStereo(imLeftCV, imRightCV, timestamp);
            //std::cout << "Image size: " << imLeftCV.cols << "x" << imLeftCV.rows << std::endl;

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