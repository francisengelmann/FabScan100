FabScan100
==========
The FabScan100 software is planned to run on Mac and Linux (and maybe also Windows).
This git repository stores all the files for the project, software as well as the hardware.

Also check these links for more information:
* http://hci.rwth-aachen.de/fabscan100
* http://groups.google.com/group/fabscan

Progress
========

The following checklist shows the projects progress:
    
* OpenGL - works on Mac, works on Ubuntu, not tested on Windows
* OpenCV - works on Mac, works on Ubuntu, not tested on Windows
* PCL - works on Mac, works on Ubuntu, not tested on Windows
* SerialPort - works on Mac, works on Ubuntu, not tested on Windows
* accessing arbitrary webcams - not implemented on Mac, works on Ubuntu (but slow), not tested on Windows
* Serial talk to FabScan - should work, needs to be tested
* Scanning - needs to be implemented

Current Setup on Mac: Qt 4.8.1 with Qt-Creator 2.4.1
Current Setup on Ubuntu: Qt 4.8.0 with Qt-Creator 2.4.1
Current Setup on Windows: not tested, who wants to do this ?

Qt 4.8 is needed for QGLFunctions.

Installation
===========

Install Qt Creator from http://qt-project.org/downloads
Make sure you have OpenGL installed. Preinstalled on Ubuntu, Mac. What about Windows?

Mac OS X 10.8.1 with XCode 4.5 installed
========================================
1. Install Homebrew: http://mxcl.github.com/homebrew/
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

Ubuntu 12.04 LTS:
=======
1. Install Qt
  * Qt-Creator from Ubuntu Software Center
  * install the qt-4.8 libraries from qt-project.org

2. Install OpenCV following this guide: http://www.samontab.com/web/2012/06/installing-opencv-2-4-1-ubuntu-12-04-lts/

  Important: OpenCV 2.4.2 is the most recent at the time of writing, so just replace 2.4.1 with 2.4.2 everywhere it occurs.
  
3. Install PCL
  * sudo apt-get install libeigen3-dev
  * sudo add-apt-repository ppa:v-launchpad-jochen-sprickerhof-de/pcl
  * sudo apt-get update
  * sudo apt-get install libpcl-all

4. Install uvccapture
  * sudo apt-get install uvccapture
  * to test run the following command: uvccapture -d/dev/video0 -oshot.jpg

Windows:
========
 Send me an email in case you want to help me here.

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
git clone git://github.com/francisengelmann/FabScan.git

If you also want to modify the code please send me an email with your ssh public key so i can give you access.