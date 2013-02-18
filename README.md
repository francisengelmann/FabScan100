FabScan100
==========
The FabScan100 software is planned to run on Mac and Linux (and maybe also Windows).
This git repository stores all the files for the project, software as well as the hardware.

Also check these links for more information:
* http://hci.rwth-aachen.de/fabscan100
* http://groups.google.com/group/fabscan

Progress
========

FabScan100 is now functional! Runs on Mac and Ubuntu, still needs to be ported to Windows.

Current Setup on Mac: Qt 5.0.1 with Qt-Creator 2.6.2

Current Setup on Ubuntu: Qt 5.0.1 with Qt-Creator 2.6.2 

Current Setup on Windows: not tested, who wants to do this ?

Qt 4.8 is needed for QGLFunctions.

Installation
===========

Install "Qt Creator" and "Qt Libraries Qt 5.0" from http://qt-project.org/downloads.
Make sure you have OpenGL installed. Preinstalled on Ubuntu, Mac. What about Windows?

Mac OS X 10.8.2 with XCode 4.6 installed
========================================
1. Install Homebrew: http://mxcl.github.com/homebrew/
The next steps need to be executed in Terminal.
Terminal is located in Applications/Utilities/Terminal.app
2. Install OpenCV using homebrew: brew install opencv
3. Install Boost: brew install boost
4. Install FLANN: brew install flann
5. Install VTK: brew install vtk
6. Install eigen: brew install eigen
7. Install the experimental version from trunck of PCL 1.7

  * svn co http://svn.pointclouds.org/pcl/trunk pcl-trunk
  * cd pcl-trunk && mkdir build && cd build
  * cmake -DCMAKE_BUILD_TYPE=Release .. 
  * make
  * sudo make install

At this point we have installed all the dependencies needed for FabScan. We can now proceed to actually compiling FabScan:

8. Install git which is a tool to access source repositories: brew install git
9. Then download the code using git: git clone git://github.com/francisengelmann/FabScan100.git
10. Open qtTest/qtTest.pro in "Qt Creator".
11. If asked select "Qt 5.0.1" Kit, then click "Configure Project".
12. In the left panel, click on "Projects", then "Manage Kits". Select from AutoDetected "Qt5.0.1", click clone, then in the "Compiler" popupmenu select "GCC x86 64bit". Then press OK to exit.
13. Again in the "Projects" tab, select the newly created Kit from the "Add Kit" popupmenu.
14. Run the project by clicking on the green arrow on the bottom left.
15. Wait for it to compile, this can take a few minutes.
 
Ubuntu 12.10:
=======
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

Windows:
========
People are working on this :)
 
Other platforms
===============

Make sure to adapt the .pro file e.g. correct lib and header pathes.

Get the source code
===================

In order to compile the code you need git:
Mac: brew install git
Ubunutu: sudo apt-get install git
Windows: ?

To get read-only access to the code, type into the console:
git clone git://github.com/francisengelmann/FabScan100.git

If you also want to modify the code please send me an email with your ssh public key so i can give you access.

