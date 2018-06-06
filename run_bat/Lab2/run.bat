@echo off
cd /d %~dp0
..\Png2Cloud.exe ../../Lab2/depth/ ../../Lab2/color/ ../../Lab2/camera_para.txt ../../Lab2/pointcloud/ ../../Lab2/pointcloud_ds/ ../../Lab2/pointcloud_xyzn/ ../../Lab2/pointcloud_ds_xyzn/
..\Sift.exe ../../Lab2/color/ ../../Lab2/sift_correspondence.txt
..\ShiTomasi.exe ../../Lab2/color/ ../../Lab2/corner_correspondence.txt
..\Narf.exe ../../Lab2/pointcloud_xyzn/ ../../Lab2/keypoint/ ../../Lab2/geometric_descriptor/
::SIFT matching
..\ColorCorrespondence.exe ../../Lab2/pointcloud/ ../../Lab2/pointcloud_ds/ ../../Lab2/depth/ ../../Lab2/sift_correspondence.txt ../../Lab2/camera_para.txt 4.0 ../../Lab2/sift_traj.txt ../../Lab2/sift_info.txt
::ShiTomasi matching
..\ColorCorrespondence.exe ../../Lab2/pointcloud/ ../../Lab2/pointcloud_ds/ ../../Lab2/depth/ ../../Lab2/corner_correspondence.txt ../../Lab2/camera_para.txt 4.0 ../../Lab2/corner_traj.txt ../../Lab2/corner_info.txt
..\GeometricCorrespondence.exe ../../Lab2/pointcloud/ ../../Lab2/pointcloud_ds/ ../../Lab2/keypoint/ ../../Lab2/geometric_descriptor/ ../../Lab2/camera_para.txt 4.0 ../../Lab2/narf_traj.txt ../../Lab2/narf_info.txt
..\MergeInfo.exe ../../Lab2/pointcloud_ds/ ../../Lab2/narf_traj.txt ../../Lab2/narf_info.txt ../../Lab2/sift_traj.txt ../../Lab2/sift_info.txt ../../Lab2/corner_traj.txt ../../Lab2/corner_info.txt ../../Lab2/traj.txt ../../Lab2/info.txt
..\GlobalOptimizer1.exe ../../Lab2/pointcloud/ ../../Lab2/traj.txt ../../Lab2/info.txt ../../Lab2/pose_opt1.txt ../../Lab2/fail_build_complete_model.txt ../../Lab2/selected_edge_opt1.txt 0.35 0.5 0.35 0.0025
if exist ../../Lab2/fail_build_complete_model.txt goto OVER
..\GlobalOptimizer2.exe ../../Lab2/pose_opt1.txt ../../Lab2/pointcloud/ ../../Lab2/traj.txt ../../Lab2/info.txt ../../Lab2/pose_opt2.txt ../../Lab2/traj_remain_opt2.txt ../../Lab2/info_remain_opt2.txt 50.0
:OVER