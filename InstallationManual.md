# Guia Para instalacion de ORB-SLAM3 [Ubuntu 22.04.2 LTS] para utilizar con ZED SDK

## 1. Instalacion de dependencias:

```shell
sudo apt update
sudo apt-get install build-essential
sudo apt-get install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
sudo apt-get install python2-dev libpython2.7-dev libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libdc1394-22-dev libopenjp2-7-dev
sudo apt-get install libglew-dev libboost-all-dev libssl-dev
sudo apt install libeigen3-dev libblas-dev liblapack-dev libgstreamer1.0-dev libpango1.0-0 libepoxy-dev
```

---

Creamos una carpeta "Dev" donde instalaremos lo necesario para ORB-SLAM3.

## 2. Instalar OpenCV 4.6.0

```shell
cd ~
mkdir Dev
cd Dev
git clone https://github.com/opencv/opencv.git
cd opencv
```
Se instala especificamente la version 4.6.0
```shell
git checkout 4.6.0
```
Se compila:
```shell
mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=Release -D WITH_CUDA=OFF -D CMAKE_INSTALL_PREFIX=/usr/local ..
```
Se crea ejecutable:
```shell
make -j $(nproc)
```
Se ejecuta:
```shell
sudo make install
```
---

## 3. Instalar Pangolin

```shell
cd ~/Dev
git clone https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin 
mkdir build 
cd build 
```
Se compila:
```shell
cmake .. -D CMAKE_BUILD_TYPE=Release
```
Se crea ejecutable:
```shell
make -j $(nproc)
```
Se ejecuta:
```shell
sudo make install
```

---

## 4. ORB-SLAM 3

```shell
cd ~/Dev
git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git 
cd ORB_SLAM3
```

```shell
sed -i 's/++11/++14/g' CMakeLists.txt
```

```shell
./build.sh
```
#### (Opcional) Descargar dataset de prueba...

```shell
cd ~
mkdir -p Datasets/EuRoc
cd Datasets/EuRoc/
wget -c http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/machine_hall/MH_01_easy/MH_01_easy.zip
```
Se descomprime:
```shell
mkdir MH01
unzip MH_01_easy.zip -d MH01/
```
### Run simulation

```shell
cd ~/Dev/ORB_SLAM3
# Stereo + Inertial
./Examples/Stereo-Inertial/stereo_inertial_euroc ./Vocabulary/ORBvoc.txt ./Examples/Stereo-Inertial/EuRoC.yaml ~/Datasets/EuRoc/MH01 ./Examples/Stereo-Inertial/EuRoC_TimeStamps/MH01.txt dataset-MH01_stereoi
```

## 5. ZED SDK

Se descarga de la pagina oficial StereoLabs el ZED SDK:

[Stereolabs - ZED SDK]([URL](https://www.stereolabs.com/en-cl/developers/release))

Se descarga la version 4.2.2 dejando un archivo:

```shell
ZED_SDK_Ubuntu22_cuda12.1_v4.2.2.zstd.run
```
Se va al directorio de la descarga:
```shell
sudo apt install zstd
chmod +x ZED_SDK_Ubuntu22_cuda12.1_v4.2.2.zstd.run
```
Se instala:
```shell
./ZED_SDK_Ubuntu22_cuda12.1_v4.2.2.zstd.run
```

## 5. Archivo de enlace entre ORB-SLAM3 y Camara ZED 2i

Se mueve la carpeta "Prueba_ORB-SLAM3_CamaraZED" hasta el directorio:



 Luego de esto se puede acceder al Manual de Usuario para utilizar el programa.
