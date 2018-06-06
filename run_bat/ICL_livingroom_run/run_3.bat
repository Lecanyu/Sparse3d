@echo off
cd /d %~dp0


set STARTTIME=%TIME%
..\Png2Cloud.exe ../../ICL_NUIM_living_room/sandbox_3/depth/ ../../ICL_NUIM_living_room/sandbox_3/color/ ../../ICL_NUIM_living_room/sandbox_3/camera_para.txt ../../ICL_NUIM_living_room/sandbox_3/pointcloud/ ../../ICL_NUIM_living_room/sandbox_3/pointcloud_ds/ ../../ICL_NUIM_living_room/sandbox_3/pointcloud_xyzn/ ../../ICL_NUIM_living_room/sandbox_3/pointcloud_ds_xyzn/
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
echo Png2Cloud time cost > ../../ICL_NUIM_living_room/sandbox_3/time_Png2Cloud.txt
echo Start    : %STARTTIME% >> ../../ICL_NUIM_living_room/sandbox_3/time_Png2Cloud.txt
echo Finish   : %ENDTIME% >> ../../ICL_NUIM_living_room/sandbox_3/time_Png2Cloud.txt
echo          --------------- >> ../../ICL_NUIM_living_room/sandbox_3/time_Png2Cloud.txt
echo Duration : %DURATION%  >> ../../ICL_NUIM_living_room/sandbox_3/time_Png2Cloud.txt



set STARTTIME=%TIME%
..\Sift.exe ../../ICL_NUIM_living_room/sandbox_3/color/ ../../ICL_NUIM_living_room/sandbox_3/sift_correspondence.txt
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
echo Sift time cost > ../../ICL_NUIM_living_room/sandbox_3/time_Sift.txt
echo Start    : %STARTTIME% >> ../../ICL_NUIM_living_room/sandbox_3/time_Sift.txt
echo Finish   : %ENDTIME% >> ../../ICL_NUIM_living_room/sandbox_3/time_Sift.txt
echo          --------------- >> ../../ICL_NUIM_living_room/sandbox_3/time_Sift.txt
echo Duration : %DURATION%  >> ../../ICL_NUIM_living_room/sandbox_3/time_Sift.txt



set STARTTIME=%TIME%
..\ShiTomasi.exe ../../ICL_NUIM_living_room/sandbox_3/color/ ../../ICL_NUIM_living_room/sandbox_3/corner_correspondence.txt
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
echo ShiTomasi time cost > ../../ICL_NUIM_living_room/sandbox_3/time_ShiTomasi.txt
echo Start    : %STARTTIME% >> ../../ICL_NUIM_living_room/sandbox_3/time_ShiTomasi.txt
echo Finish   : %ENDTIME% >> ../../ICL_NUIM_living_room/sandbox_3/time_ShiTomasi.txt
echo          --------------- >> ../../ICL_NUIM_living_room/sandbox_3/time_ShiTomasi.txt
echo Duration : %DURATION%  >> ../../ICL_NUIM_living_room/sandbox_3/time_ShiTomasi.txt



set STARTTIME=%TIME%
..\Narf.exe ../../ICL_NUIM_living_room/sandbox_3/pointcloud_xyzn/ ../../ICL_NUIM_living_room/sandbox_3/keypoint/ ../../ICL_NUIM_living_room/sandbox_3/geometric_descriptor/
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
echo Narf time cost > ../../ICL_NUIM_living_room/sandbox_3/time_Narf.txt
echo Start    : %STARTTIME% >> ../../ICL_NUIM_living_room/sandbox_3/time_Narf.txt
echo Finish   : %ENDTIME% >> ../../ICL_NUIM_living_room/sandbox_3/time_Narf.txt
echo          --------------- >> ../../ICL_NUIM_living_room/sandbox_3/time_Narf.txt
echo Duration : %DURATION%  >> ../../ICL_NUIM_living_room/sandbox_3/time_Narf.txt


set STARTTIME=%TIME%
::SIFT matching
..\ColorCorrespondence.exe ../../ICL_NUIM_living_room/sandbox_3/pointcloud/ ../../ICL_NUIM_living_room/sandbox_3/pointcloud_ds/ ../../ICL_NUIM_living_room/sandbox_3/depth/ ../../ICL_NUIM_living_room/sandbox_3/sift_correspondence.txt ../../ICL_NUIM_living_room/sandbox_3/camera_para.txt -4.0 ../../ICL_NUIM_living_room/sandbox_3/sift_traj.txt ../../ICL_NUIM_living_room/sandbox_3/sift_info.txt
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
echo matching_SIFT time cost > ../../ICL_NUIM_living_room/sandbox_3/time_matching_SIFT.txt
echo Start    : %STARTTIME% >> ../../ICL_NUIM_living_room/sandbox_3/time_matching_SIFT.txt
echo Finish   : %ENDTIME% >> ../../ICL_NUIM_living_room/sandbox_3/time_matching_SIFT.txt
echo          --------------- >> ../../ICL_NUIM_living_room/sandbox_3/time_matching_SIFT.txt
echo Duration : %DURATION%  >> ../../ICL_NUIM_living_room/sandbox_3/time_matching_SIFT.txt



set STARTTIME=%TIME%
::ShiTomasi matching
..\ColorCorrespondence.exe ../../ICL_NUIM_living_room/sandbox_3/pointcloud/ ../../ICL_NUIM_living_room/sandbox_3/pointcloud_ds/ ../../ICL_NUIM_living_room/sandbox_3/depth/ ../../ICL_NUIM_living_room/sandbox_3/corner_correspondence.txt ../../ICL_NUIM_living_room/sandbox_3/camera_para.txt -4.0 ../../ICL_NUIM_living_room/sandbox_3/corner_traj.txt ../../ICL_NUIM_living_room/sandbox_3/corner_info.txt
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
echo matching_Shi time cost > ../../ICL_NUIM_living_room/sandbox_3/time_matching_Shi.txt
echo Start    : %STARTTIME% >> ../../ICL_NUIM_living_room/sandbox_3/time_matching_Shi.txt
echo Finish   : %ENDTIME% >> ../../ICL_NUIM_living_room/sandbox_3/time_matching_Shi.txt
echo          --------------- >> ../../ICL_NUIM_living_room/sandbox_3/time_matching_Shi.txt
echo Duration : %DURATION%  >> ../../ICL_NUIM_living_room/sandbox_3/time_matching_Shi.txt


set STARTTIME=%TIME%
..\GeometricCorrespondence.exe ../../ICL_NUIM_living_room/sandbox_3/pointcloud/ ../../ICL_NUIM_living_room/sandbox_3/pointcloud_ds/ ../../ICL_NUIM_living_room/sandbox_3/keypoint/ ../../ICL_NUIM_living_room/sandbox_3/geometric_descriptor/ ../../ICL_NUIM_living_room/sandbox_3/camera_para.txt -4.0 ../../ICL_NUIM_living_room/sandbox_3/narf_traj.txt ../../ICL_NUIM_living_room/sandbox_3/narf_info.txt
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
echo matching_NARF time cost > ../../ICL_NUIM_living_room/sandbox_3/time_matching_NARF.txt
echo Start    : %STARTTIME% >> ../../ICL_NUIM_living_room/sandbox_3/time_matching_NARF.txt
echo Finish   : %ENDTIME% >> ../../ICL_NUIM_living_room/sandbox_3/time_matching_NARF.txt
echo          --------------- >> ../../ICL_NUIM_living_room/sandbox_3/time_matching_NARF.txt
echo Duration : %DURATION%  >> ../../ICL_NUIM_living_room/sandbox_3/time_matching_NARF.txt


set STARTTIME=%TIME%
..\MergeInfo.exe ../../ICL_NUIM_living_room/sandbox_3/pointcloud_ds/ ../../ICL_NUIM_living_room/sandbox_3/narf_traj.txt ../../ICL_NUIM_living_room/sandbox_3/narf_info.txt ../../ICL_NUIM_living_room/sandbox_3/sift_traj.txt ../../ICL_NUIM_living_room/sandbox_3/sift_info.txt ../../ICL_NUIM_living_room/sandbox_3/corner_traj.txt ../../ICL_NUIM_living_room/sandbox_3/corner_info.txt ../../ICL_NUIM_living_room/sandbox_3/traj.txt ../../ICL_NUIM_living_room/sandbox_3/info.txt
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
echo MergeInfo time cost > ../../ICL_NUIM_living_room/sandbox_3/time_MergeInfo.txt
echo Start    : %STARTTIME% >> ../../ICL_NUIM_living_room/sandbox_3/time_MergeInfo.txt
echo Finish   : %ENDTIME% >> ../../ICL_NUIM_living_room/sandbox_3/time_MergeInfo.txt
echo          --------------- >> ../../ICL_NUIM_living_room/sandbox_3/time_MergeInfo.txt
echo Duration : %DURATION%  >> ../../ICL_NUIM_living_room/sandbox_3/time_MergeInfo.txt



set STARTTIME=%TIME%
..\GlobalOptimizer1.exe ../../ICL_NUIM_living_room/sandbox_3/pointcloud/ ../../ICL_NUIM_living_room/sandbox_3/traj.txt ../../ICL_NUIM_living_room/sandbox_3/info.txt ../../ICL_NUIM_living_room/sandbox_3/pose_opt1.txt ../../ICL_NUIM_living_room/sandbox_3/fail_build_complete_model.txt ../../ICL_NUIM_living_room/sandbox_3/selected_edge_opt1.txt 0.35 0.55 0.35 0.001
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
echo opt1 time cost > ../../ICL_NUIM_living_room/sandbox_3/time_opt1.txt
echo Start    : %STARTTIME% >> ../../ICL_NUIM_living_room/sandbox_3/time_opt1.txt
echo Finish   : %ENDTIME% >> ../../ICL_NUIM_living_room/sandbox_3/time_opt1.txt
echo          --------------- >> ../../ICL_NUIM_living_room/sandbox_3/time_opt1.txt
echo Duration : %DURATION%  >> ../../ICL_NUIM_living_room/sandbox_3/time_opt1.txt


if exist ../../ICL_NUIM_living_room/sandbox_3/fail_build_complete_model.txt goto OVER



set STARTTIME=%TIME%
..\GlobalOptimizer2.exe ../../ICL_NUIM_living_room/sandbox_3/pose_opt1.txt ../../ICL_NUIM_living_room/sandbox_3/pointcloud/ ../../ICL_NUIM_living_room/sandbox_3/traj.txt ../../ICL_NUIM_living_room/sandbox_3/info.txt ../../ICL_NUIM_living_room/sandbox_3/pose_opt2.txt ../../ICL_NUIM_living_room/sandbox_3/traj_remain_opt2.txt ../../ICL_NUIM_living_room/sandbox_3/info_remain_opt2.txt 50.0
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
echo opt2 time cost > ../../ICL_NUIM_living_room/sandbox_3/time_opt2.txt
echo Start    : %STARTTIME% >> ../../ICL_NUIM_living_room/sandbox_3/time_opt2.txt
echo Finish   : %ENDTIME% >> ../../ICL_NUIM_living_room/sandbox_3/time_opt2.txt
echo          --------------- >> ../../ICL_NUIM_living_room/sandbox_3/time_opt2.txt
echo Duration : %DURATION%  >> ../../ICL_NUIM_living_room/sandbox_3/time_opt2.txt

:OVER