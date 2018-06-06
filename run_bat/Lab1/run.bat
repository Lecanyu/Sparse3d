@echo off
cd /d %~dp0


set STARTTIME=%TIME%
..\Png2Cloud.exe ../../Lab1/depth/ ../../Lab1/color/ ../../Lab1/camera_para.txt ../../Lab1/pointcloud/ ../../Lab1/pointcloud_ds/ ../../Lab1/pointcloud_xyzn/ ../../Lab1/pointcloud_ds_xyzn/
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
echo Png2Cloud time cost > ../../Lab1/time_Png2Cloud.txt
echo Start    : %STARTTIME% >> ../../Lab1/time_Png2Cloud.txt
echo Finish   : %ENDTIME% >> ../../Lab1/time_Png2Cloud.txt
echo          --------------- >> ../../Lab1/time_Png2Cloud.txt
echo Duration : %DURATION%  >> ../../Lab1/time_Png2Cloud.txt



set STARTTIME=%TIME%
..\Sift.exe ../../Lab1/color/ ../../Lab1/sift_correspondence.txt
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
echo Sift time cost > ../../Lab1/time_Sift.txt
echo Start    : %STARTTIME% >> ../../Lab1/time_Sift.txt
echo Finish   : %ENDTIME% >> ../../Lab1/time_Sift.txt
echo          --------------- >> ../../Lab1/time_Sift.txt
echo Duration : %DURATION%  >> ../../Lab1/time_Sift.txt



set STARTTIME=%TIME%
..\ShiTomasi.exe ../../Lab1/color/ ../../Lab1/corner_correspondence.txt
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
echo ShiTomasi time cost > ../../Lab1/time_ShiTomasi.txt
echo Start    : %STARTTIME% >> ../../Lab1/time_ShiTomasi.txt
echo Finish   : %ENDTIME% >> ../../Lab1/time_ShiTomasi.txt
echo          --------------- >> ../../Lab1/time_ShiTomasi.txt
echo Duration : %DURATION%  >> ../../Lab1/time_ShiTomasi.txt



set STARTTIME=%TIME%
..\Narf.exe ../../Lab1/pointcloud_xyzn/ ../../Lab1/keypoint/ ../../Lab1/geometric_descriptor/
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
echo Narf time cost > ../../Lab1/time_Narf.txt
echo Start    : %STARTTIME% >> ../../Lab1/time_Narf.txt
echo Finish   : %ENDTIME% >> ../../Lab1/time_Narf.txt
echo          --------------- >> ../../Lab1/time_Narf.txt
echo Duration : %DURATION%  >> ../../Lab1/time_Narf.txt


set STARTTIME=%TIME%
::SIFT matching
..\ColorCorrespondence.exe ../../Lab1/pointcloud/ ../../Lab1/pointcloud_ds/ ../../Lab1/depth/ ../../Lab1/sift_correspondence.txt ../../Lab1/camera_para.txt 4.0 ../../Lab1/sift_traj.txt ../../Lab1/sift_info.txt
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
echo matching_SIFT time cost > ../../Lab1/time_matching_SIFT.txt
echo Start    : %STARTTIME% >> ../../Lab1/time_matching_SIFT.txt
echo Finish   : %ENDTIME% >> ../../Lab1/time_matching_SIFT.txt
echo          --------------- >> ../../Lab1/time_matching_SIFT.txt
echo Duration : %DURATION%  >> ../../Lab1/time_matching_SIFT.txt



set STARTTIME=%TIME%
::ShiTomasi matching
..\ColorCorrespondence.exe ../../Lab1/pointcloud/ ../../Lab1/pointcloud_ds/ ../../Lab1/depth/ ../../Lab1/corner_correspondence.txt ../../Lab1/camera_para.txt 4.0 ../../Lab1/corner_traj.txt ../../Lab1/corner_info.txt
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
echo matching_Shi time cost > ../../Lab1/time_matching_Shi.txt
echo Start    : %STARTTIME% >> ../../Lab1/time_matching_Shi.txt
echo Finish   : %ENDTIME% >> ../../Lab1/time_matching_Shi.txt
echo          --------------- >> ../../Lab1/time_matching_Shi.txt
echo Duration : %DURATION%  >> ../../Lab1/time_matching_Shi.txt


set STARTTIME=%TIME%
..\GeometricCorrespondence.exe ../../Lab1/pointcloud/ ../../Lab1/pointcloud_ds/ ../../Lab1/keypoint/ ../../Lab1/geometric_descriptor/ ../../Lab1/camera_para.txt 4.0 ../../Lab1/narf_traj.txt ../../Lab1/narf_info.txt
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
echo matching_NARF time cost > ../../Lab1/time_matching_NARF.txt
echo Start    : %STARTTIME% >> ../../Lab1/time_matching_NARF.txt
echo Finish   : %ENDTIME% >> ../../Lab1/time_matching_NARF.txt
echo          --------------- >> ../../Lab1/time_matching_NARF.txt
echo Duration : %DURATION%  >> ../../Lab1/time_matching_NARF.txt


set STARTTIME=%TIME%
..\MergeInfo.exe ../../Lab1/pointcloud_ds/ ../../Lab1/narf_traj.txt ../../Lab1/narf_info.txt ../../Lab1/sift_traj.txt ../../Lab1/sift_info.txt ../../Lab1/corner_traj.txt ../../Lab1/corner_info.txt ../../Lab1/traj.txt ../../Lab1/info.txt
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
echo MergeInfo time cost > ../../Lab1/time_MergeInfo.txt
echo Start    : %STARTTIME% >> ../../Lab1/time_MergeInfo.txt
echo Finish   : %ENDTIME% >> ../../Lab1/time_MergeInfo.txt
echo          --------------- >> ../../Lab1/time_MergeInfo.txt
echo Duration : %DURATION%  >> ../../Lab1/time_MergeInfo.txt



set STARTTIME=%TIME%
..\GlobalOptimizer1.exe ../../Lab1/pointcloud/ ../../Lab1/traj.txt ../../Lab1/info.txt ../../Lab1/pose_opt1.txt ../../Lab1/fail_build_complete_model.txt ../../Lab1/selected_edge_opt1.txt 0.35 0.5 0.35 0.005
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
echo opt1 time cost > ../../Lab1/time_opt1.txt
echo Start    : %STARTTIME% >> ../../Lab1/time_opt1.txt
echo Finish   : %ENDTIME% >> ../../Lab1/time_opt1.txt
echo          --------------- >> ../../Lab1/time_opt1.txt
echo Duration : %DURATION%  >> ../../Lab1/time_opt1.txt


if exist ../../Lab1/fail_build_complete_model.txt goto OVER



set STARTTIME=%TIME%
..\GlobalOptimizer2.exe ../../Lab1/pose_opt1.txt ../../Lab1/pointcloud/ ../../Lab1/traj.txt ../../Lab1/info.txt ../../Lab1/pose_opt2.txt ../../Lab1/traj_remain_opt2.txt ../../Lab1/info_remain_opt2.txt 50.0
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
echo opt2 time cost > ../../Lab1/time_opt2.txt
echo Start    : %STARTTIME% >> ../../Lab1/time_opt2.txt
echo Finish   : %ENDTIME% >> ../../Lab1/time_opt2.txt
echo          --------------- >> ../../Lab1/time_opt2.txt
echo Duration : %DURATION%  >> ../../Lab1/time_opt2.txt

:OVER