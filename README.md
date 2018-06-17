# Sparse3D

Sparse3D is an offline reconstruction system. It is able to robustly reconstruction sparse RGB-D input data (e.g. fast-moving scanning, small inter-frame overlap and etc). 

Please check our published paper for more details.

___[Sparse3D: A new global model for matching sparse RGB-D dataset with small inter-frame overlap](https://www.sciencedirect.com/science/article/pii/S0010448518302276)___


Here is the video demo
https://youtu.be/qk8iQzommPM


# 1. Prerequisites

We have tested this libraries on Windows10 x64 operation system with Microsoft Visual Studio 2013 (x86 solution). It should be easy to compile in other OS.

The following install instruction only works on Windows10 X64 OS and Microsoft Visual Studio 2013 (x86 solution).

For your convenience, the 3rd dependencies can be downloaded [here](https://drive.google.com/open?id=1mQyDLpdVoQKWVBcRU9iTdNWRnw1liQKY).


Modify environment variables
--------------------
After Install/unzip all of dependencies. You should create some environment variables and specify correct path. Please refer "env.txt" for more details.


# 2. Build
When you finish all of environment configuration, you can use VS2013 to build the whole project. If all goes well, you can run it without any errors.


# 3. Datasets
Our experiment datasets can be downloaded [here](https://drive.google.com/open?id=1q1NG6_shWjXbohKKMSA9b7OHsfc54L6G).


# 4. Citation
If this implementation is useful in your research, please cite

```
@article{le2018sparse3d,
  title={Sparse3D: A new global model for matching sparse RGB-D dataset with small inter-frame overlap},
  author={Le, Canyu and Li, Xin},
  journal={Computer-Aided Design},
  volume={102},
  pages={33--43},
  year={2018},
  publisher={Elsevier}
}
```

