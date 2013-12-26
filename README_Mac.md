Compiling Fabscan in Mac OS X
=============================

Mac OS X 10.8.2 with XCode 4.6 installed
----------------------------------------
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
