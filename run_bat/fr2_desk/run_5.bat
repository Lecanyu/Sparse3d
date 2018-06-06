@echo off
cd /d %~dp0


set STARTTIME=%TIME%
..\Png2Cloud.exe ../../rgbd_dataset_freiburg2_desk/sandbox_5/depth/ ../../rgbd_dataset_freiburg2_desk/sandbox_5/color/ ../../rgbd_dataset_freiburg2_desk/sandbox_5/camera_para.txt ../../rgbd_dataset_freiburg2_desk/sandbox_5/pointcloud/ ../../rgbd_dataset_freiburg2_desk/sandbox_5/pointcloud_ds/ ../../rgbd_dataset_freiburg2_desk/sandbox_5/pointcloud_xyzn/ ../../rgbd_dataset_freiburg2_desk/sandbox_5/pointcloud_ds_xyzn/
set ENDTIME=%TIME%
for /F "tokens=1-4 delims=:.," %%a in ("%STARTTIME%") do (
   set /A "start=(((%%a*60)+1%%b %% 100)*60+1%%c %% 100)*100+1%%d %% 100"
)
for /F "tokens=1-4 delims=:.," %%a in ("%ENDTIME%") do (
   set /A "end=(((%%a*60)+1%%b %% 100)*60+1%%c %% 100)*100+1%%d %% 100"
)
set /A elapsed=end-start
set /A hh=elapsed/(60*60*100), rest=elapsed%%(60*60*100), mm=rest/(60*100), rest%%=60*100, ss=rest/100, cc=rest%%100
if %hh% lss 10 set hh=0%hh%
if %mm% lss 10 set mm=0%mm%
if %ss% lss 10 set ss=0%ss%
if %cc% lss 10 set cc=0%cc%
set DURATION=%hh%:%mm%:%ss%,%cc%
echo Png2Cloud time cost > ../../rgbd_dataset_freiburg2_desk/sandbox_5/time_Png2Cloud.txt
echo Start    : %STARTTIME% >> ../../rgbd_dataset_freiburg2_desk/sandbox_5/time_Png2Cloud.txt
echo Finish   : %ENDTIME% >> ../../rgbd_dataset_freiburg2_desk/sandbox_5/time_Png2Cloud.txt
echo          --------------- >> ../../rgbd_dataset_freiburg2_desk/sandbox_5/time_Png2Cloud.txt
echo Duration : %DURATION%  >> ../../rgbd_dataset_freiburg2_desk/sandbox_5/time_Png2Cloud.txt



set STARTTIME=%TIME%
..\Sift.exe ../../rgbd_dataset_freiburg2_desk/sandbox_5/color/ ../../rgbd_dataset_freiburg2_desk/sandbox_5/sift_correspondence.txt
set ENDTIME=%TIME%
for /F "tokens=1-4 delims=:.," %%a in ("%STARTTIME%") do (
   set /A "start=(((%%a*60)+1%%b %% 100)*60+1%%c %% 100)*100+1%%d %% 100"
)
for /F "tokens=1-4 delims=:.," %%a in ("%ENDTIME%") do (
   set /A "end=(((%%a*60)+1%%b %% 100)*60+1%%c %% 100)*100+1%%d %% 100"
)
set /A elapsed=end-start
set /A hh=elapsed/(60*60*100), rest=elapsed%%(60*60*100), mm=rest/(60*100), rest%%=60*100, ss=rest/100, cc=rest%%100
if %hh% lss 10 set hh=0%hh%
if %mm% lss 10 set mm=0%mm%
if %ss% lss 10 set ss=0%ss%
if %cc% lss 10 set cc=0%cc%
set DURATION=%hh%:%mm%:%ss%,%cc%
echo Sift time cost > ../../rgbd_dataset_freiburg2_desk/sandbox_5/time_Sift.txt
echo Start    : %STARTTIME% >> ../../rgbd_dataset_freiburg2_desk/sandbox_5/time_Sift.txt
echo Finish   : %ENDTIME% >> ../../rgbd_dataset_freiburg2_desk/sandbox_5/time_Sift.txt
echo          --------------- >> ../../rgbd_dataset_freiburg2_desk/sandbox_5/time_Sift.txt
echo Duration : %DURATION%  >> ../../rgbd_dataset_freiburg2_desk/sandbox_5/time_Sift.txt



set STARTTIME=%TIME%
..\ShiTomasi.exe ../../rgbd_dataset_freiburg2_desk/sandbox_5/color/ ../../rgbd_dataset_freiburg2_desk/sandbox_5/corner_correspondence.txt
set ENDTIME=%TIME%
for /F "tokens=1-4 delims=:.," %%a in ("%STARTTIME%") do (
   set /A "start=(((%%a*60)+1%%b %% 100)*60+1%%c %% 100)*100+1%%d %% 100"
)
for /F "tokens=1-4 delims=:.," %%a in ("%ENDTIME%") do (
   set /A "end=(((%%a*60)+1%%b %% 100)*60+1%%c %% 100)*100+1%%d %% 100"
)
set /A elapsed=end-start
set /A hh=elapsed/(60*60*100), rest=elapsed%%(60*60*100), mm=rest/(60*100), rest%%=60*100, ss=rest/100, cc=rest%%100
if %hh% lss 10 set hh=0%hh%
if %mm% lss 10 set mm=0%mm%
if %ss% lss 10 set ss=0%ss%
if %cc% lss 10 set cc=0%cc%
set DURATION=%hh%:%mm%:%ss%,%cc%
echo ShiTomasi time cost > ../../rgbd_dataset_freiburg2_desk/sandbox_5/time_ShiTomasi.txt
echo Start    : %STARTTIME% >> ../../rgbd_dataset_freiburg2_desk/sandbox_5/time_ShiTomasi.txt
echo Finish   : %ENDTIME% >> ../../rgbd_dataset_freiburg2_desk/sandbox_5/time_ShiTomasi.txt
echo          --------------- >> ../../rgbd_dataset_freiburg2_desk/sandbox_5/time_ShiTomasi.txt
echo Duration : %DURATION%  >> ../../rgbd_dataset_freiburg2_desk/sandbox_5/time_ShiTomasi.txt



set STARTTIME=%TIME%
..\Narf.exe ../../rgbd_dataset_freiburg2_desk/sandbox_5/pointcloud_xyzn/ ../../rgbd_dataset_freiburg2_desk/sandbox_5/keypoint/ ../../rgbd_dataset_freiburg2_desk/sandbox_5/geometric_descriptor/
set ENDTIME=%TIME%
for /F "tokens=1-4 delims=:.," %%a in ("%STARTTIME%") do (
   set /A "start=(((%%a*60)+1%%b %% 100)*60+1%%c %% 100)*100+1%%d %% 100"
)
for /F "tokens=1-4 delims=:.," %%a in ("%ENDTIME%") do (
   set /A "end=(((%%a*60)+1%%b %% 100)*60+1%%c %% 100)*100+1%%d %% 100"
)
set /A elapsed=end-start
set /A hh=elapsed/(60*60*100), rest=elapsed%%(60*60*100), mm=rest/(60*100), rest%%=60*100, ss=rest/100, cc=rest%%100
if %hh% lss 10 set hh=0%hh%
if %mm% lss 10 set mm=0%mm%
if %ss% lss 10 set ss=0%ss%
if %cc% lss 10 set cc=0%cc%
set DURATION=%hh%:%mm%:%ss%,%cc%
echo Narf time cost > ../../rgbd_dataset_freiburg2_desk/sandbox_5/time_Narf.txt
echo Start    : %STARTTIME% >> ../../rgbd_dataset_freiburg2_desk/sandbox_5/time_Narf.txt
echo Finish   : %ENDTIME% >> ../../rgbd_dataset_freiburg2_desk/sandbox_5/time_Narf.txt
echo          --------------- >> ../../rgbd_dataset_freiburg2_desk/sandbox_5/time_Narf.txt
echo Duration : %DURATION%  >> ../../rgbd_dataset_freiburg2_desk/sandbox_5/time_Narf.txt


set STARTTIME=%TIME%
::SIFT matching
..\ColorCorrespondence.exe ../../rgbd_dataset_freiburg2_desk/sandbox_5/pointcloud/ ../../rgbd_dataset_freiburg2_desk/sandbox_5/pointcloud_ds/ ../../rgbd_dataset_freiburg2_desk/sandbox_5/depth/ ../../rgbd_dataset_freiburg2_desk/sandbox_5/sift_correspondence.txt ../../rgbd_dataset_freiburg2_desk/sandbox_5/camera_para.txt 4.0 ../../rgbd_dataset_freiburg2_desk/sandbox_5/sift_traj.txt ../../rgbd_dataset_freiburg2_desk/sandbox_5/sift_info.txt
set ENDTIME=%TIME%
for /F "tokens=1-4 delims=:.," %%a in ("%STARTTIME%") do (
   set /A "start=(((%%a*60)+1%%b %% 100)*60+1%%c %% 100)*100+1%%d %% 100"
)
for /F "tokens=1-4 delims=:.," %%a in ("%ENDTIME%") do (
   set /A "end=(((%%a*60)+1%%b %% 100)*60+1%%c %% 100)*100+1%%d %% 100"
)
set /A elapsed=end-start
set /A hh=elapsed/(60*60*100), rest=elapsed%%(60*60*100), mm=rest/(60*100), rest%%=60*100, ss=rest/100, cc=rest%%100
if %hh% lss 10 set hh=0%hh%
if %mm% lss 10 set mm=0%mm%
if %ss% lss 10 set ss=0%ss%
if %cc% lss 10 set cc=0%cc%
set DURATION=%hh%:%mm%:%ss%,%cc%
echo matching_SIFT time cost > ../../rgbd_dataset_freiburg2_desk/sandbox_5/time_matching_SIFT.txt
echo Start    : %STARTTIME% >> ../../rgbd_dataset_freiburg2_desk/sandbox_5/time_matching_SIFT.txt
echo Finish   : %ENDTIME% >> ../../rgbd_dataset_freiburg2_desk/sandbox_5/time_matching_SIFT.txt
echo          --------------- >> ../../rgbd_dataset_freiburg2_desk/sandbox_5/time_matching_SIFT.txt
echo Duration : %DURATION%  >> ../../rgbd_dataset_freiburg2_desk/sandbox_5/time_matching_SIFT.txt



set STARTTIME=%TIME%
::ShiTomasi matching
..\ColorCorrespondence.exe ../../rgbd_dataset_freiburg2_desk/sandbox_5/pointcloud/ ../../rgbd_dataset_freiburg2_desk/sandbox_5/pointcloud_ds/ ../../rgbd_dataset_freiburg2_desk/sandbox_5/depth/ ../../rgbd_dataset_freiburg2_desk/sandbox_5/corner_correspondence.txt ../../rgbd_dataset_freiburg2_desk/sandbox_5/camera_para.txt 4.0 ../../rgbd_dataset_freiburg2_desk/sandbox_5/corner_traj.txt ../../rgbd_dataset_freiburg2_desk/sandbox_5/corner_info.txt
set ENDTIME=%TIME%
for /F "tokens=1-4 delims=:.," %%a in ("%STARTTIME%") do (
   set /A "start=(((%%a*60)+1%%b %% 100)*60+1%%c %% 100)*100+1%%d %% 100"
)
for /F "tokens=1-4 delims=:.," %%a in ("%ENDTIME%") do (
   set /A "end=(((%%a*60)+1%%b %% 100)*60+1%%c %% 100)*100+1%%d %% 100"
)
set /A elapsed=end-start
set /A hh=elapsed/(60*60*100), rest=elapsed%%(60*60*100), mm=rest/(60*100), rest%%=60*100, ss=rest/100, cc=rest%%100
if %hh% lss 10 set hh=0%hh%
if %mm% lss 10 set mm=0%mm%
if %ss% lss 10 set ss=0%ss%
if %cc% lss 10 set cc=0%cc%
set DURATION=%hh%:%mm%:%ss%,%cc%
echo matching_Shi time cost > ../../rgbd_dataset_freiburg2_desk/sandbox_5/time_matching_Shi.txt
echo Start    : %STARTTIME% >> ../../rgbd_dataset_freiburg2_desk/sandbox_5/time_matching_Shi.txt
echo Finish   : %ENDTIME% >> ../../rgbd_dataset_freiburg2_desk/sandbox_5/time_matching_Shi.txt
echo          --------------- >> ../../rgbd_dataset_freiburg2_desk/sandbox_5/time_matching_Shi.txt
echo Duration : %DURATION%  >> ../../rgbd_dataset_freiburg2_desk/sandbox_5/time_matching_Shi.txt


set STARTTIME=%TIME%
..\GeometricCorrespondence.exe ../../rgbd_dataset_freiburg2_desk/sandbox_5/pointcloud/ ../../rgbd_dataset_freiburg2_desk/sandbox_5/pointcloud_ds/ ../../rgbd_dataset_freiburg2_desk/sandbox_5/keypoint/ ../../rgbd_dataset_freiburg2_desk/sandbox_5/geometric_descriptor/ ../../rgbd_dataset_freiburg2_desk/sandbox_5/camera_para.txt 4.0 ../../rgbd_dataset_freiburg2_desk/sandbox_5/narf_traj.txt ../../rgbd_dataset_freiburg2_desk/sandbox_5/narf_info.txt
set ENDTIME=%TIME%
for /F "tokens=1-4 delims=:.," %%a in ("%STARTTIME%") do (
   set /A "start=(((%%a*60)+1%%b %% 100)*60+1%%c %% 100)*100+1%%d %% 100"
)
for /F "tokens=1-4 delims=:.," %%a in ("%ENDTIME%") do (
   set /A "end=(((%%a*60)+1%%b %% 100)*60+1%%c %% 100)*100+1%%d %% 100"
)
set /A elapsed=end-start
set /A hh=elapsed/(60*60*100), rest=elapsed%%(60*60*100), mm=rest/(60*100), rest%%=60*100, ss=rest/100, cc=rest%%100
if %hh% lss 10 set hh=0%hh%
if %mm% lss 10 set mm=0%mm%
if %ss% lss 10 set ss=0%ss%
if %cc% lss 10 set cc=0%cc%
set DURATION=%hh%:%mm%:%ss%,%cc%
echo matching_NARF time cost > ../../rgbd_dataset_freiburg2_desk/sandbox_5/time_matching_NARF.txt
echo Start    : %STARTTIME% >> ../../rgbd_dataset_freiburg2_desk/sandbox_5/time_matching_NARF.txt
echo Finish   : %ENDTIME% >> ../../rgbd_dataset_freiburg2_desk/sandbox_5/time_matching_NARF.txt
echo          --------------- >> ../../rgbd_dataset_freiburg2_desk/sandbox_5/time_matching_NARF.txt
echo Duration : %DURATION%  >> ../../rgbd_dataset_freiburg2_desk/sandbox_5/time_matching_NARF.txt


set STARTTIME=%TIME%
..\MergeInfo.exe ../../rgbd_dataset_freiburg2_desk/sandbox_5/pointcloud_ds/ ../../rgbd_dataset_freiburg2_desk/sandbox_5/narf_traj.txt ../../rgbd_dataset_freiburg2_desk/sandbox_5/narf_info.txt ../../rgbd_dataset_freiburg2_desk/sandbox_5/sift_traj.txt ../../rgbd_dataset_freiburg2_desk/sandbox_5/sift_info.txt ../../rgbd_dataset_freiburg2_desk/sandbox_5/corner_traj.txt ../../rgbd_dataset_freiburg2_desk/sandbox_5/corner_info.txt ../../rgbd_dataset_freiburg2_desk/sandbox_5/traj.txt ../../rgbd_dataset_freiburg2_desk/sandbox_5/info.txt
set ENDTIME=%TIME%
for /F "tokens=1-4 delims=:.," %%a in ("%STARTTIME%") do (
   set /A "start=(((%%a*60)+1%%b %% 100)*60+1%%c %% 100)*100+1%%d %% 100"
)
for /F "tokens=1-4 delims=:.," %%a in ("%ENDTIME%") do (
   set /A "end=(((%%a*60)+1%%b %% 100)*60+1%%c %% 100)*100+1%%d %% 100"
)
set /A elapsed=end-start
set /A hh=elapsed/(60*60*100), rest=elapsed%%(60*60*100), mm=rest/(60*100), rest%%=60*100, ss=rest/100, cc=rest%%100
if %hh% lss 10 set hh=0%hh%
if %mm% lss 10 set mm=0%mm%
if %ss% lss 10 set ss=0%ss%
if %cc% lss 10 set cc=0%cc%
set DURATION=%hh%:%mm%:%ss%,%cc%
echo MergeInfo time cost > ../../rgbd_dataset_freiburg2_desk/sandbox_5/time_MergeInfo.txt
echo Start    : %STARTTIME% >> ../../rgbd_dataset_freiburg2_desk/sandbox_5/time_MergeInfo.txt
echo Finish   : %ENDTIME% >> ../../rgbd_dataset_freiburg2_desk/sandbox_5/time_MergeInfo.txt
echo          --------------- >> ../../rgbd_dataset_freiburg2_desk/sandbox_5/time_MergeInfo.txt
echo Duration : %DURATION%  >> ../../rgbd_dataset_freiburg2_desk/sandbox_5/time_MergeInfo.txt



set STARTTIME=%TIME%
..\GlobalOptimizer1.exe ../../rgbd_dataset_freiburg2_desk/sandbox_5/pointcloud/ ../../rgbd_dataset_freiburg2_desk/sandbox_5/traj.txt ../../rgbd_dataset_freiburg2_desk/sandbox_5/info.txt ../../rgbd_dataset_freiburg2_desk/sandbox_5/pose_opt1.txt ../../rgbd_dataset_freiburg2_desk/sandbox_5/fail_build_complete_model.txt ../../rgbd_dataset_freiburg2_desk/sandbox_5/selected_edge_opt1.txt 0.35 0.5 0.35 0.001
set ENDTIME=%TIME%
for /F "tokens=1-4 delims=:.," %%a in ("%STARTTIME%") do (
   set /A "start=(((%%a*60)+1%%b %% 100)*60+1%%c %% 100)*100+1%%d %% 100"
)
for /F "tokens=1-4 delims=:.," %%a in ("%ENDTIME%") do (
   set /A "end=(((%%a*60)+1%%b %% 100)*60+1%%c %% 100)*100+1%%d %% 100"
)
set /A elapsed=end-start
set /A hh=elapsed/(60*60*100), rest=elapsed%%(60*60*100), mm=rest/(60*100), rest%%=60*100, ss=rest/100, cc=rest%%100
if %hh% lss 10 set hh=0%hh%
if %mm% lss 10 set mm=0%mm%
if %ss% lss 10 set ss=0%ss%
if %cc% lss 10 set cc=0%cc%
set DURATION=%hh%:%mm%:%ss%,%cc%
echo opt1 time cost > ../../rgbd_dataset_freiburg2_desk/sandbox_5/time_opt1.txt
echo Start    : %STARTTIME% >> ../../rgbd_dataset_freiburg2_desk/sandbox_5/time_opt1.txt
echo Finish   : %ENDTIME% >> ../../rgbd_dataset_freiburg2_desk/sandbox_5/time_opt1.txt
echo          --------------- >> ../../rgbd_dataset_freiburg2_desk/sandbox_5/time_opt1.txt
echo Duration : %DURATION%  >> ../../rgbd_dataset_freiburg2_desk/sandbox_5/time_opt1.txt


if exist ../../rgbd_dataset_freiburg2_desk/sandbox_5/fail_build_complete_model.txt goto OVER



set STARTTIME=%TIME%
..\GlobalOptimizer2.exe ../../rgbd_dataset_freiburg2_desk/sandbox_5/pose_opt1.txt ../../rgbd_dataset_freiburg2_desk/sandbox_5/pointcloud/ ../../rgbd_dataset_freiburg2_desk/sandbox_5/traj.txt ../../rgbd_dataset_freiburg2_desk/sandbox_5/info.txt ../../rgbd_dataset_freiburg2_desk/sandbox_5/pose_opt2.txt ../../rgbd_dataset_freiburg2_desk/sandbox_5/traj_remain_opt2.txt ../../rgbd_dataset_freiburg2_desk/sandbox_5/info_remain_opt2.txt 50.0
set ENDTIME=%TIME%
for /F "tokens=1-4 delims=:.," %%a in ("%STARTTIME%") do (
   set /A "start=(((%%a*60)+1%%b %% 100)*60+1%%c %% 100)*100+1%%d %% 100"
)
for /F "tokens=1-4 delims=:.," %%a in ("%ENDTIME%") do (
   set /A "end=(((%%a*60)+1%%b %% 100)*60+1%%c %% 100)*100+1%%d %% 100"
)
set /A elapsed=end-start
set /A hh=elapsed/(60*60*100), rest=elapsed%%(60*60*100), mm=rest/(60*100), rest%%=60*100, ss=rest/100, cc=rest%%100
if %hh% lss 10 set hh=0%hh%
if %mm% lss 10 set mm=0%mm%
if %ss% lss 10 set ss=0%ss%
if %cc% lss 10 set cc=0%cc%
set DURATION=%hh%:%mm%:%ss%,%cc%
echo opt2 time cost > ../../rgbd_dataset_freiburg2_desk/sandbox_5/time_opt2.txt
echo Start    : %STARTTIME% >> ../../rgbd_dataset_freiburg2_desk/sandbox_5/time_opt2.txt
echo Finish   : %ENDTIME% >> ../../rgbd_dataset_freiburg2_desk/sandbox_5/time_opt2.txt
echo          --------------- >> ../../rgbd_dataset_freiburg2_desk/sandbox_5/time_opt2.txt
echo Duration : %DURATION%  >> ../../rgbd_dataset_freiburg2_desk/sandbox_5/time_opt2.txt

:OVER