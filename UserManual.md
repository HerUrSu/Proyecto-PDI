# Manual de Usuario para ORB-SLAM3 junto con cámara ZED
## 1. Realizar el build del proyecto
Dirigirse al directorio donde se encuentra **CMakeLists.txt**. Editar los directorios del archivo para que coincidan con los del usuario.  
Luego, buildear el proyecto de la siguiente forma:
```
mkdir build
cd build
make
cmake ..
```
## 2. Conectar la Cámara ZED por USB

## 3. Correr el programa
Luego, para correr el código, dentro de la carpeta build realizar el siguiente comando:
```
./stereo_zed path_to_vocabulary path_to_settings (trajectory_file_name)
```
Donde path_to_vocabulary corresponde al vocabulario de ORB-SLAM3, incluído al descargarlo. path_to_settings corresponde al archivo .yaml con los datos intrínsicos de la cámara. trajectory_file_name correspone al nombre del archivo donde se almacenarán los datos de posición adquiridos.  
Ejemplo:
```
./stereo_zed /home/gabo/Dev/ORB_SLAM3/Vocabulary/ORBvoc.txt /home/gabo/Dev/ORB_SLAM3/Examples/Stereo-Inertial/ZED_2i.yaml trayectoria.txt 
```


