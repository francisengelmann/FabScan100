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

Compiling
=========
- Mac OS X: [README_Mac.md](README_Mac.md)
- Ubuntu: [README_Ubuntu.md](README_Ubuntu.md)
- Windows: [README_Windows.md](README_Windows.md)
- Other platforms: Make sure to adapt the .pro file e.g. correct lib and header pathes.

Get the source code
===================

In order to compile the code you need git:
- Mac: brew install git
- Ubunutu: sudo apt-get install git
- Windows: use any git client you like to
 - [Git](https://help.github.com/articles/set-up-git#platform-windows)
 - [TortoiseGit](https://code.google.com/p/tortoisegit/)
- To get read-only access to the code, type into the console:
git clone git://github.com/francisengelmann/FabScan100.git
- If you also want to modify the code please send me an email with your ssh public key so i can give you access.
- If you don't want to make any changes you can simply download the code by clicking the "Download ZIP" button on the right side of this text
