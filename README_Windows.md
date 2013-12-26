Compiling Fabscan in Windows
============================

Prerequisites
-------------
Download and install all the libraries to C:\libs (no relative paths at the moment)
- C:\libs\pcl-1.6.0 == [PCL-1.6.0-AllInOne-msvc2010-win32.exe](http://sourceforge.net/projects/pointclouds/files/1.6.0/PCL-1.6.0-AllInOne-msvc2010-win32.exe/download)
- C:\libs\opencv-2.4.2 == [OpenCV-2.4.2.exe](http://sourceforge.net/projects/opencvlibrary/files/opencv-win/2.4.2/OpenCV-2.4.2.exe/download)
- C:\libs\Qt\Qt5.2.0 == [qt-windows-opensource-5.2.0-msvc2010_opengl-x86-offline.exe](http://download.qt-project.org/official_releases/qt/5.2/5.2.0/qt-windows-opensource-5.2.0-msvc2010_opengl-x86-offline.exe)
- C:\libs\openni-1.3.2.1 == C:\libs\pcl-1.6.0\3rdParty\OpenNI\OpenNI-Win32-1.3.2-Dev.msi
 - Should be started during the installation of PCL
- [Visual C++ 2010 Express](http://go.microsoft.com/?linkid=9709949)
- [Microsoft Windows SDK for Windows 7](http://www.microsoft.com/en-us/download/details.aspx?id=8442) (if you want debugging support - check "Windows Debugging Tools" is checked)
 

Compiling
---------
- Clone this repository to your disk
- Open the [qtTest.pro](qtTest/qtTest.pro) file in QtCreator
- Make sure under "Configure Projects" / "Projects" the MSVC2010 Compiler Kit is selected
- Compile

Copy DLLs
---------
(Is QtCreator capable of post-build-events like Visual Studio?)

Copy the following files to your output directory:
- Boost
 - C:\libs\pcl-1.6.0\3rdParty\Boost\lib\boost_filesystem-vc100-mt-1_49.dll
 - C:\libs\pcl-1.6.0\3rdParty\Boost\lib\boost_system-vc100-mt-1_49.dll
 - C:\libs\pcl-1.6.0\3rdParty\Boost\lib\boost_system-vc100-mt-gd-1_49.dll
- Qt
 - C:\libs\Qt\Qt5.2.0\5.2.0\msvc2010_opengl\bin\icudt51.dll
 - C:\libs\Qt\Qt5.2.0\5.2.0\msvc2010_opengl\bin\icuin51.dll
 - C:\libs\Qt\Qt5.2.0\5.2.0\msvc2010_opengl\bin\icuuc51.dll
 - C:\libs\Qt\Qt5.2.0\5.2.0\msvc2010_opengl\bin\Qt5Cored.dll
 - C:\libs\Qt\Qt5.2.0\5.2.0\msvc2010_opengl\bin\Qt5Guid.dll
 - C:\libs\Qt\Qt5.2.0\5.2.0\msvc2010_opengl\bin\Qt5Multimediad.dll
 - C:\libs\Qt\Qt5.2.0\5.2.0\msvc2010_opengl\bin\Qt5MultimediaWidgetsd.dll
 - C:\libs\Qt\Qt5.2.0\5.2.0\msvc2010_opengl\bin\Qt5Networkd.dll
 - C:\libs\Qt\Qt5.2.0\5.2.0\msvc2010_opengl\bin\Qt5OpenGLd.dll
 - C:\libs\Qt\Qt5.2.0\5.2.0\msvc2010_opengl\bin\Qt5Widgetsd.dll
- OpenCV
 - C:\libs\opencv-2.4.2\opencv\build\x86\vc10\bin\opencv_core242.dll
 - C:\libs\opencv-2.4.2\opencv\build\x86\vc10\bin\opencv_highgui242.dll
 - C:\libs\opencv-2.4.2\opencv\build\x86\vc10\bin\opencv_imgproc242.dll
 - C:\libs\opencv-2.4.2\opencv\build\common\tbb\ia32\vc10\tbb.dll
- PCL
 - C:\libs\pcl-1.6.0\bin\pcl_common_release.dll
 - C:\libs\pcl-1.6.0\bin\pcl_features_release.dll
 - C:\libs\pcl-1.6.0\bin\pcl_io_ply_release.dll
 - C:\libs\pcl-1.6.0\bin\pcl_io_release.dll
 - C:\libs\pcl-1.6.0\bin\pcl_kdtree_release.dll
 - C:\libs\pcl-1.6.0\bin\pcl_octree_release.dll
 - C:\libs\pcl-1.6.0\bin\pcl_search_release.dll
 - C:\libs\pcl-1.6.0\bin\pcl_surface_release.dll
- OpenNI
 - C:\libs\openni-1.3.2.1\Bin\OpenNI.dll

 Copy platform dll
------------------
- Create a directory "platforms" in the output directory
- Copy the Qt platform plugin dlls to this new directory
 - C:\libs\Qt\Qt5.2.0\5.2.0\msvc2010_opengl\plugins\platforms

Download binaries
-----------------
Alternatively you can download my precompiled binaries:
- [Fabscan_Windows @ Mega.co.nz](https://mega.co.nz/#F!fgRQQJ5S!C375-2_Srylrs6iLOwyYjw)

More information
----------------
- My dev system is a Windows 7 SP1 x64 Virtual Machine
- Be sure to use the versions I mentioned. QtCreator of 5.0.2 didn't work for me because there is a problem with Boost's macros
- Be careful when you copy the dlls. I wasted some hours because a dll of the wrong architecture

TODO
----
- Copy the dlls by a postbuild process
- Make the paths in qtTest.pro relative or based on a environment variable or something
- Test if the project still compiles under linux/macosx (but should due to only small changes and the use of preprocessor constants for os-specific code fragments)
- Ensure the the debug libs/dlls are used on a debug build and the release libs/dlls are used on a release build
- Find out why debugging does not work, even if the debugger is recognized within QtCreator

Known Issues
------------
- If you have a ATI/AMD graphic card and the app crashes have a look at the windows event log. It may complain about a atioglxx.dll. Have a look in C:\Windows\System32 if it's there. If not: reinstall your graphic driver. If this doesn't resolve the problem search for atioglxx (no extension) in the unpacked driver installer, find a file called atioglxx.dl_. Copy and rename it to atioglxx.dll. Move it to C:\Windows\System32
