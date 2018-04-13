# Sparse3D

Sparse3D is an offline reconstruction system. It is able to robustly reconstruction sparse RGB-D input data (e.g. fast-moving scanning, small inter-frame overlap and etc). Please check our published paper for more details.

'''
Our Reference Paper.
'''

A video demo has been published in xxxxx


# 1. Prerequisites

We have tested this libraries on Windows10 x64 operation system with Microsoft Visual Studio 2013 (x86 solution). It should be easy to compile in other OS.

The following install instruction only works on Windows10 X64 OS and Microsoft Visual Studio 2013 (x86 solution).

For your convenience, here is the 3rdParty download link:

https://drive.google.com/open?id=1mQyDLpdVoQKWVBcRU9iTdNWRnw1liQKY


OpenCV2.7.13
--------------------
1. Install 3rdParty/opencv-2.4.13.exe

2. Modify your machine's environment variable and add X:\opencv2.4.13\opencv\build\x86\vc12\bin and X:\opencv2.4.13\opencv\build\x64\vc12\bin

3. Modify VS2013 solution properties. Configure Include Directories, Library Directory and Additional Dependencies.


PCL1.8
-------------------------
1. Install 3rdParty/PCL-1.8.0-AllInOne-msvc2013-win32.exe. If you don't have OPENNI2, please install it during PCL1.8 installation.

2. Modify your machine's environment variable and add X:\PCL 1.8.0\bin and X:\OpenNI2\Redist

3. Modify VS2013 solution properties.


G2O
------------------------
1. unzip 3rdParty/g2o.zip

2. Modify your machine's environment variable and add X:\g2o\bin\Release and X:\g2o\Debug

3. Modify VS2013 solution properties.


Armadillo7.800.1
------------------------
1. unzip 2rdParty/armadillo-7.800.1.zip

2. Modify your machine's environment variable and add X:\armadillo-7.800.1\LAPACK_Debug_x86 and X:\armadillo-7.800.1\LAPACK_Release_x86

3. Modify VS2013 solution properties.



# 2. Build
When you finish all of environment configuration, you can use VS2013 to build the whole project. If all goes well, you can run it without any errors.


# 3. Datasets
We have published our experiment datasets on XXXXXXXXXXXXXXX.



