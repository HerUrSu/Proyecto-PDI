# Guia ORB-SLAM3 [Ubuntu 22.04.2 LTS]

## 1. Instalacion de ORB-SLAM 3 en Ubuntu 22.04

Instalacion de dependencias.

```shell
sudo apt update
```

```shell
sudo apt-get install build-essential
sudo apt-get install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
sudo apt-get install python2-dev libpython2.7-dev libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libdc1394-22-dev libopenjp2-7-dev
sudo apt-get install libglew-dev libboost-all-dev libssl-dev
sudo apt install libeigen3-dev
```

---

Problemas?

```shell
sudo apt install libblas-dev liblapack-dev
```

```shell
sudo apt install libgstreamer1.0-dev
```

Problemas Pangolin:

```shell
sudo apt install libpango1.0-0
```

```shell
sudo apt install libepoxy-dev
```

## 2. Instalar OpenCV 4.6.0

```shell
cd ~
mkdir Dev
cd Dev
git clone https://github.com/opencv/opencv.git
cd opencv
```

```shell
git checkout 4.4.0
```

```shell
mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=Release -D WITH_CUDA=OFF -D CMAKE_INSTALL_PREFIX=/usr/local ..
```

```shell
make -j $(nproc)
```

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

```shell
cmake .. -D CMAKE_BUILD_TYPE=Release
```

```shell
make -j $(nproc)
```

```shell
sudo make install
```

---

### 4. ORB-SLAM 3

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

---

# Descargar dataset para probar...

```shell
cd ~
mkdir -p Datasets/EuRoc
cd Datasets/EuRoc/
```

```shell
wget -c http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/machine_hall/MH_01_easy/MH_01_easy.zip
```

```shell
mkdir MH01
unzip MH_01_easy.zip -d MH01/
```

---

## Run simulation

```shell
cd ~/Dev/ORB_SLAM3

# Pick of them below that you want to run

# Mono
./Examples/Monocular/mono_euroc ./Vocabulary/ORBvoc.txt ./Examples/Monocular/EuRoC.yaml ~/Datasets/EuRoc/MH01 ./Examples/Monocular/EuRoC_TimeStamps/MH01.txt dataset-MH01_mono

# Mono + Inertial
./Examples/Monocular-Inertial/mono_inertial_euroc ./Vocabulary/ORBvoc.txt ./Examples/Monocular-Inertial/EuRoC.yaml ~/Datasets/EuRoc/MH01 ./Examples/Monocular-Inertial/EuRoC_TimeStamps/MH01.txt dataset-MH01_monoi

# Stereo
./Examples/Stereo/stereo_euroc ./Vocabulary/ORBvoc.txt ./Examples/Stereo/EuRoC.yaml ~/Datasets/EuRoc/MH01 ./Examples/Stereo/EuRoC_TimeStamps/MH01.txt dataset-MH01_stereo

# Stereo + Inertial
./Examples/Stereo-Inertial/stereo_inertial_euroc ./Vocabulary/ORBvoc.txt ./Examples/Stereo-Inertial/EuRoC.yaml ~/Datasets/EuRoc/MH01 ./Examples/Stereo-Inertial/EuRoC_TimeStamps/MH01.txt dataset-MH01_stereoi
```

## Extra

Se necesita numpy and matplotlib, se instala pytho2.7 y pip2.7

```shell
sudo apt install curl
```

```shell
cd ~/Desktop
curl https://bootstrap.pypa.io/2.7/get-pip.py --output get-pip.py
```

Se instala PIP

```shell
sudo python2 get-pip.py
```

Se instala numpy and matplotlib

```shell
pip2.7 install numpy matplotlib
```

Problemas para python2.7?

https://linux.how2shout.com/how-to-install-python-2-7-on-ubuntu-24-04-noble-lts-linux/

# 
