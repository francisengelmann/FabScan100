Compiling Fabscan in Ubuntu 12.10:
==================================
- Make sure you have Ubuntu 12.10 because the version 1.6 of the pcl library is not supported in 13.10 anymore


1. Install Qt
  * Install Qt-5.0.1 libraries http://qt-project.org/downloads

2. Install OpenCV (inspired from http://www.samontab.com/web/2012/06/installing-opencv-2-4-1-ubuntu-12-04-lts/)
  * sudo apt-get update
  * sudo apt-get upgrade
  * sudo apt-get install libeigen3-dev build-essential libgtk2.0-dev libjpeg-dev libtiff4-dev libjasper-dev libopenexr-dev cmake python-dev python-numpy python-tk libtbb-dev yasm libfaac-dev libopencore-amrnb-dev libopencore-amrwb-dev libtheora-dev libvorbis-dev libxvidcore-dev libx264-dev sphinx-common texlive-latex-extra libv4l-dev libdc1394-22-dev libavcodec-dev libavformat-dev libswscale-dev

  * cd ~
  * wget http://downloads.sourceforge.net/project/opencvlibrary/opencv-unix/2.4.4/OpenCV-2.4.4-beta.tar.bz2
  * tar -xvf OpenCV-2.4.4-beta.tar.bz2
  * cd OpenCV-2.4.4

  * mkdir build
  * cd build
  * cmake -D WITH_TBB=ON -D BUILD_NEW_PYTHON_SUPPORT=ON -D WITH_V4L=ON -D INSTALL_C_EXAMPLES=OFF -D INSTALL_PYTHON_EXAMPLES=OFF -D BUILD_EXAMPLES=OFF -D WITH_QT=ON -D WITH_OPENGL=ON ..
  
3. Install PCL
  * sudo add-apt-repository ppa:v-launchpad-jochen-sprickerhof-de/pcl
  * sudo apt-get update
  * sudo apt-get install libpcl-all

4. Install uvccapture
  * sudo apt-get install uvccapture
  * to test run the following command: uvccapture -d/dev/video0 -oshot.jpg

5. Give permissions for serial port
  * sudo adduser USER_NAME dialout
  * log out and back in
