FabScan1.5
==========

This is basically just a small project where I try to combine the different libraies in one program on multiple platforms.
The Libraries I plan to use are the following:
* OpenGL - works on Mac, works on Ubuntu, not tested on Windows
* OpenCV - works on Mac, not tested on Ubuntu, not tested on Windows
* PCL - not tested on Mac, not tested on Ubunutu, not tested on Windows

Current Setup on Mac: Qt 4.7 with Qt-Creator 2.4

Current Setup on Ubuntu: Qt 4.8 with Qt-Creator 2.4.1

Current Setup on Windows: not tested, who wants to do this ?

Installation
===========

For all platforms:
==================

Install Qt Creator from http://qt-project.org/downloads
Make sure you have OpenGL installed. Preinstalled on Ubunutu, Mac. What about Windows?

Mac:
===
1. Install Homebrew: http://mxcl.github.com/homebrew/
2. Install OpenCV using homebrew: brew install opencv
3. Install PCL using homebrew: see http://www.pointclouds.org/documentation/tutorials/installing_homebrew.php

Ubuntu:
=======
1. Install OpenCV following this guide: http://www.samontab.com/web/2012/06/installing-opencv-2-4-1-ubuntu-12-04-lts/
  (OpenCV 2.4.2 is the most recent at time of writing this)
