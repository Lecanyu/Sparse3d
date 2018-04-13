# Low-frame-reconstruction

Low frame reconstruction is an offline reconstruction system based on RGB-D input data. The whole system is constituted by three main components:
feature detection, feature correspondence and a novel global optimization.

After optimization, you will get a precise camera trajectory even though it is faced with very low frame rate situation. Our experiment demostrate our method achieved an impressive result and more robust than state-of-the-art SLAM and reconstruction system. 

If you want to get a triangle mesh model, you can use TSDF to integrate all of pointclouds and then use marching cube to meshing. For TSDF we provide source code and execute program. For marching cube we only provide execute program, it is a CUDA program which comes from PCL.  

# 1. Prerequisites

We have tested the libraries in Windows10 x64 operation system and Microsoft Visual Studio 2013. And it should be easy to compile in other OS like linux, Mac OS etc. We include all of dependencies in 3rdParty file folder. You can easily install them. 

The following install tutorials which we provide are only tested in Windows10 X64 OS and Microsoft Visual Studio 2013.

For your convenience, here is the 3rdParty download link:
https://pan.baidu.com/s/1qYqds76

OpenCV2.7.13
--------------------
1. Install 3rdParty/opencv-2.4.13.exe

2. Modify your machine's environment variable and add X:\opencv2.4.13\opencv\build\x86\vc12\bin and X:\opencv2.4.13\opencv\build\x64\vc12\bin

3. Modify VS2013 solution properties. Configure Include Directories, Library Directory and Additional Dependencies. In src directory, you will see a series of VS2013 project, there is .sln file in every sub-project. Open it, and then check VS2013 project solution properties, what you should do is imitate existing configuration.


PCL1.8
-------------------------
1. Install 3rdParty/PCL-1.8.0-AllInOne-msvc2013-win32.exe. If you don't have OPENNI2, don't forget install it when you are installing PCL1.8

2. Modify your machine's environment variable and add X:\PCL 1.8.0\bin and X:\OpenNI2\Redist

3. The same with section OpenCV2.7.13 -->3


G2O
------------------------
1. unzip 3rdParty/g2o.zip

2. Modify your machine's environment variable and add X:\g2o\bin\Release and X:\g2o\Debug

3. The same with section OpenCV2.7.13 -->3


Armadillo7.800.1
------------------------
it is used to solve sparse matrix. You must install it.

1. unzip 2rdParty/armadillo-7.800.1.zip

2. Modify your machine's environment variable and add X:\armadillo-7.800.1\LAPACK_Debug_x86 and X:\armadillo-7.800.1\LAPACK_Release_x86

3. The same with section OpenCV2.7.13 -->3



# 2. Build
When you finish all of environment configuration, you can use VS2013 to build the whole project. If all goes well, you will can run it.

# 3. Unfinished Dataset Example
Due to the size limitation, please download all of dataset from the link: (forthcoming......) 

Here are several unfinished dataset (link: https://pan.baidu.com/s/1o7VO8tG). They are downsampled by 2. 


# 4. Attention
Since the original project is 32-bits, you should notice the memory limitation. After we tested, if you want to reconstruct more than 1100 images, the program will crash because it will use >1.5GB memory. 

One solution is modeify VS2013 solution's properities. Find Link->System->Enable Large Addresses, and set it to Yes. Rebuild it and then you may can run it without any exception. However, if you expect your memeory usage will more than 2GB or 3GB, the large addresses may not solve the memeory problem. Please build x64 program.



