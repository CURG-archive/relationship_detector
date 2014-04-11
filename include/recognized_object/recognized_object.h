#ifndef RECOGNIZED_OBJECT_H
#define RECOGNIZED_OBJECT_H

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

class RecognizedObject
{

	public:

		int uniqueId;
		pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudPtr;

		RecognizedObject(int recognized_object_id, pcl::PointCloud<pcl::PointXYZ>::Ptr  recognized_object_point_cloud_ptr)
		{
			uniqueId = recognized_object_id;
			pointCloudPtr = recognized_object_point_cloud_ptr;
		}

};

#endif