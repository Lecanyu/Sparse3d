#pragma once

#include <pcl/features/range_image_border_extractor.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/range_image/range_image.h>

class NARF{
	pcl::RangeImage range_image;
	pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_ptr;

	pcl::PointCloud<int> keypoint_indices;

	float support_size = 0.08f;
public:

	pcl::PointCloud<pcl::PointXYZ> keypoints;
	pcl::PointCloud<pcl::Narf36> descriptors;

	NARF(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_pointer){
		point_cloud_ptr = point_cloud_pointer;
		pcl::PointCloud<pcl::PointXYZ>& point_cloud = *point_cloud_pointer;


		// -----------------------------------------------
		// -----Create RangeImage from the PointCloud-----
		// -----------------------------------------------
		/*
		************parameters explaination**************
		1、The angular resolution is supposed to be 1 degree, meaning the beams represented by neighboring pixels differ by one degree.
		2、maxAngleWidth=360 and maxAngleHeight=180 mean that the range sensor we are simulating has a complete 360 degree view of the surrounding. You can always use this setting, since the range image will be cropped to only the areas where something was observed automatically. Yet you can save some computation by reducing the values. E.g. for a laser scanner with a 180 degree view facing forward, where no points behind the sensor can be observed, maxAngleWidth=180 is enough.
		3、sensorPose defines the 6DOF position of the virtual sensor as the origin with roll=pitch=yaw=0.
		4、coordinate_frame=CAMERA_FRAME tells the system that x is facing right, y downwards and the z axis is forward. An alternative would be LASER_FRAME, with x facing forward, y to the left and z upwards.
		5、For noiseLevel=0 the range image is created using a normal z-buffer. Yet if you want to average over points falling in the same cell you can use a higher value. 0.05 would mean, that all point with a maximum distance of 5cm to the closest point are used to calculate the range.
		6、If minRange is greater 0 all points that are closer will be ignored.
		7、borderSize greater 0 will leave a border of unobserved points around the image when cropping it.
		*/
		float angular_resolution = 0.2f;
		angular_resolution = pcl::deg2rad(angular_resolution);

		float maxAngleWidth = (float)(180.0 * (M_PI / 180.0f));  
		float maxAngleHeight = (float)(180.0 * (M_PI / 180.0f));

		Eigen::Affine3f scene_sensor_pose(Eigen::Affine3f::Identity());

		pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;

		float noise_level = 0.0f; 
		float min_range = 0.0f;
		int border_size = 0.05;

		range_image.createFromPointCloud(point_cloud, angular_resolution, maxAngleWidth, maxAngleHeight, scene_sensor_pose, coordinate_frame, noise_level, min_range, border_size);
	}

	// --------------------------------
	// -----Extract NARF keypoints-----
	// --------------------------------
	void narf_keypoints_extraction(){
		pcl::RangeImageBorderExtractor range_image_border_extractor;
		pcl::NarfKeypoint narf_keypoint_detector;
		narf_keypoint_detector.setRangeImageBorderExtractor(&range_image_border_extractor);
		narf_keypoint_detector.setRangeImage(&range_image);
		narf_keypoint_detector.getParameters().support_size = support_size;

		keypoint_indices.clear();
		narf_keypoint_detector.compute(keypoint_indices);
		std::cout << "Found " << keypoint_indices.points.size() << " key points.\n";


		keypoints.points.resize(keypoint_indices.points.size());
		for (size_t i = 0; i < keypoint_indices.points.size(); ++i)
			keypoints.points[i].getVector3fMap() = range_image.points[keypoint_indices.points[i]].getVector3fMap();
	}
};