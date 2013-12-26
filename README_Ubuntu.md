Compiling Fabscan in Ubuntu:
==================================
- Make sure you have Ubuntu 12.10 or 13.04 because the version 1.6 of the pcl library is not supported in 13.10 anymore (Tested with x86 version of ubuntu)

Prerequisites
-------------
1. Install Qt 5.0.1
```bash
wget http://download.qt-project.org/archive/qt/5.0/5.0.1/qt-linux-opensource-5.0.1-x86-offline.run
sudo chmod +x qt-linux-opensource-5.0.1-x86-offline.run
sudo ./qt-linux-opensource-5.0.1-x86-offline.run
```

2. Install OpenCV (inspired from [samontab.com](http://www.samontab.com/web/2012/06/installing-opencv-2-4-1-ubuntu-12-04-lts/))
```bash
sudo apt-get update
sudo apt-get upgrade
sudo apt-get install libeigen3-dev build-essential libgtk2.0-dev libjpeg-dev libtiff4-dev libjasper-dev libopenexr-dev cmake python-dev python-numpy python-tk libtbb-dev yasm libfaac-dev libopencore-amrnb-dev libopencore-amrwb-dev libtheora-dev libvorbis-dev libxvidcore-dev libx264-dev sphinx-common texlive-latex-extra libv4l-dev libdc1394-22-dev libavcodec-dev libavformat-dev libswscale-dev
cd ~
wget http://downloads.sourceforge.net/project/opencvlibrary/opencv-unix/2.4.4/OpenCV-2.4.4a.tar.bz2
tar -xvf OpenCV-2.4.4a.tar.bz2
cd opencv-2.4.4
mkdir build
cd build
cmake -D WITH_TBB=ON -D BUILD_NEW_PYTHON_SUPPORT=ON -D WITH_V4L=ON -D INSTALL_C_EXAMPLES=OFF -D INSTALL_PYTHON_EXAMPLES=OFF -D BUILD_EXAMPLES=OFF -D WITH_QT=ON -D WITH_OPENGL=ON ..
sudo make install
```
  
3. Install PCL
```bash
sudo add-apt-repository ppa:v-launchpad-jochen-sprickerhof-de/pcl
sudo apt-get update
sudo apt-get install libpcl-1.6-all
```

4. Install uvccapture
```bash
sudo apt-get install uvccapture
```
to test run the following command: `uvccapture -d/dev/video0 -oshot.jpg`

5. Give permissions for serial port
```bash
sudo adduser USER_NAME dialout
```
log out and back in

6. Install git
```bash
sudo apt-get install git
```

Compiling
-------------------
- Get the code: `git clone https://github.com/francisengelmann/FabScan100.git`
- Open 'FabScan100/qtTest/qtTest.pro' in QtCreator
- If asked select "Qt 5.0.1" Kit, then click "Configure Project".
- In the left panel, click on "Projects", then "Manage Kits". Select from AutoDetected "Qt5.0.1", click clone, then in the "Compiler" popupmenu select "GCC x86 32bit". Then press OK to exit.
- Again in the "Projects" tab, select the newly created Kit from the "Add Kit" popupmenu.
- Run the project by clicking on the green arrow on the bottom left.
- Wait for it to compile, this can take a few minutes.
