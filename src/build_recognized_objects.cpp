#include "build_recognized_objects.h"

int nextRecognizedObjectId = 0;

RecognizedObject buildRecognizedObject(std::string filename)
{

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ> (filename, *cloud_ptr) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file \n");
  }

  nextRecognizedObjectId +=1;
  RecognizedObject recognizedObject = RecognizedObject(nextRecognizedObjectId, cloud_ptr);

  return recognizedObject;

}