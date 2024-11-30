#include <sl/Camera.hpp>
#include <System.h>
#include <opencv2/opencv.hpp>
#include <signal.h>
#include <iostream>
#include <mutex>
#include <condition_variable>

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
             << "Usage: ./stereo_inertial_zed path_to_vocabulary path_to_settings (trajectory_file_name)"
             << endl;
        return 1;
    }

    string file_name;
    if (argc == 4) {
        file_name = string(argv[argc - 1]);
    }

    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = exit_loop_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);

    b_continue_session = true;

    // Inicia la cámara ZED
    Camera zed;
    InitParameters init_params;
    init_params.camera_resolution = RESOLUTION::HD720;
    init_params.coordinate_units = UNIT::METER;
    init_params.depth_mode = DEPTH_MODE::PERFORMANCE;
    //init_params.input_type = INPUT_TYPE::USB; Esta linea da error
    init_params.camera_fps = 30;

    ERROR_CODE err = zed.open(init_params);
    if (err != ERROR_CODE::SUCCESS) {
        cout << "No se pudo inicializar la cámara ZED: " << toString(err) << endl;
        return 1;
    }

    RuntimeParameters runtime_params;
    runtime_params = RuntimeParameters();
    //runtime_params.sensing_mode = SENSING_MODE::STANDARD;

    int width = zed.getCameraInformation().camera_configuration.resolution.width;
    int height = zed.getCameraInformation().camera_configuration.resolution.height;

    // Matrices para las imágenes
    Mat left_image(width, height, MAT_TYPE::U8_C1);
    Mat right_image(width, height, MAT_TYPE::U8_C1);

    cv::Mat imLeftCV, imRightCV;

    // Configura ORB-SLAM3
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::IMU_STEREO, true, 0, file_name);
    float imageScale = SLAM.GetImageScale();

    while (!SLAM.isShutDown()) {
        if (!b_continue_session) break;

        // Captura las imágenes izquierda y derecha
        if (zed.grab(runtime_params) == ERROR_CODE::SUCCESS) {
            zed.retrieveImage(left_image, VIEW::LEFT_GRAY);
            zed.retrieveImage(right_image, VIEW::RIGHT_GRAY);

            // Convierte las imágenes de ZED a OpenCV
            imLeftCV = cv::Mat(left_image.getHeight(), left_image.getWidth(), CV_8UC1, left_image.getPtr<sl::uchar1>());
            imRightCV = cv::Mat(right_image.getHeight(), right_image.getWidth(), CV_8UC1, right_image.getPtr<sl::uchar1>());

            double timestamp = zed.getTimestamp(TIME_REFERENCE::IMAGE).getMilliseconds() * 1e-3;

            // Escalar imágenes si es necesario
            if (imageScale != 1.f) {
                cv::resize(imLeftCV, imLeftCV, cv::Size(), imageScale, imageScale);
                cv::resize(imRightCV, imRightCV, cv::Size(), imageScale, imageScale);
            }

            // Envía las imágenes al sistema ORB-SLAM3
            SLAM.TrackStereo(imLeftCV, imRightCV, timestamp);
        }
    }

    cout << "Cerrando sistema..." << endl;
    SLAM.Shutdown();
    zed.close();

    return 0;
}
