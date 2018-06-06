@echo off
cd /d %~dp0
..\Png2Cloud.exe ../../Lab1/depth/ ../../Lab1/color/ ../../Lab1/camera_para.txt ../../Lab1/pointcloud/ ../../Lab1/pointcloud_ds/ ../../Lab1/pointcloud_xyzn/ ../../Lab1/pointcloud_ds_xyzn/
..\Sift.exe ../../Lab1/color/ ../../Lab1/sift_correspondence.txt
..\ShiTomasi.exe ../../Lab1/color/ ../../Lab1/corner_correspondence.txt
..\Narf.exe ../../Lab1/pointcloud_xyzn/ ../../Lab1/keypoint/ ../../Lab1/geometric_descriptor/
::SIFT matching
..\ColorCorrespondence.exe ../../Lab1/pointcloud/ ../../Lab1/pointcloud_ds/ ../../Lab1/depth/ ../../Lab1/sift_correspondence.txt ../../Lab1/camera_para.txt 4.0 ../../Lab1/sift_traj.txt ../../Lab1/sift_info.txt
::ShiTomasi matching
..\ColorCorrespondence.exe ../../Lab1/pointcloud/ ../../Lab1/pointcloud_ds/ ../../Lab1/depth/ ../../Lab1/corner_correspondence.txt ../../Lab1/camera_para.txt 4.0 ../../Lab1/corner_traj.txt ../../Lab1/corner_info.txt
..\GeometricCorrespondence.exe ../../Lab1/pointcloud/ ../../Lab1/pointcloud_ds/ ../../Lab1/keypoint/ ../../Lab1/geometric_descriptor/ ../../Lab1/camera_para.txt 4.0 ../../Lab1/narf_traj.txt ../../Lab1/narf_info.txt
..\MergeInfo.exe ../../Lab1/pointcloud_ds/ ../../Lab1/narf_traj.txt ../../Lab1/narf_info.txt ../../Lab1/sift_traj.txt ../../Lab1/sift_info.txt ../../Lab1/corner_traj.txt ../../Lab1/corner_info.txt ../../Lab1/traj.txt ../../Lab1/info.txt
..\GlobalOptimizer1.exe ../../Lab1/pointcloud/ ../../Lab1/traj.txt ../../Lab1/info.txt ../../Lab1/pose_opt1.txt ../../Lab1/fail_build_complete_model.txt ../../Lab1/selected_edge_opt1.txt 0.35 0.5 0.35 0.005
if exist ../../Lab1/fail_build_complete_model.txt goto OVER
..\GlobalOptimizer2.exe ../../Lab1/pose_opt1.txt ../../Lab1/pointcloud/ ../../Lab1/traj.txt ../../Lab1/info.txt ../../Lab1/pose_opt2.txt ../../Lab1/traj_remain_opt2.txt ../../Lab1/info_remain_opt2.txt 50.0
:OVER