# Ubuntu 20 Setup

## Preview
![](Demo.gif)

Preview video link:
https://youtu.be/GYLC4GT7ZFI

## Download Sample Data
https://drive.google.com/drive/folders/1qSi78IHS9fudoTOxwGGemzoMeQFTR7C7?usp=sharing

## Dependencies

```
sudo apt install cmake
sudo apt install g++
sudo apt install libgflags-dev
sudo apt install libopengl-dev
sudo apt install mesa-common-dev
sudo apt install libqt5opengl5-dev
sudo apt install libomp5
```

Also download and install Intel's MKL via OneApi, or alternatively remove or comment out the lines in the top CMakeLists.txt that are about finding and using Intel's MKL.

## Build

Build with CMake:
```
mkdir release
cd release
cmake ..
make -j
```

## Usage

Use the converter executable to convert a ply file to an octree file and the viewer executable to view it. You can use --help on any of the compiled executables to show you which command line arguments are needed to be specified. 
