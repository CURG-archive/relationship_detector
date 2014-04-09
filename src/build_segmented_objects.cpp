#include "build_segmented_objects.h"

int nextSegmentedObjectId = 0;

SegmentedObject buildSegmentedObject(std::string filename)
{

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ> (filename, *cloud_ptr) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file \n");
  }

  nextSegmentedObjectId +=1;
  SegmentedObject segmentedObject = SegmentedObject(nextSegmentedObjectId, cloud_ptr);

  return segmentedObject;

}