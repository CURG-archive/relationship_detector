#ifndef SEGMENTED_OBJECT_H
#define SEGMENTED_OBJECT_H

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

class SegmentedObject
{

	public:

		int uniqueId;
		pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudPtr;

		SegmentedObject(int segmented_object_id, pcl::PointCloud<pcl::PointXYZ>::Ptr  segmented_object_point_cloud_ptr)
		{
			uniqueId = segmented_object_id;
			pointCloudPtr = segmented_object_point_cloud_ptr;
		}

};

#endif