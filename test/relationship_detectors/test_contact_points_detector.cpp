#include <ros/package.h>
#include <string>
#include "gtest/gtest.h"
#include "contact_points_detector.h"
#include <iostream>

//catkin_make run_tests


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

TEST(ContactPointsDetector, TestAppleContactPoints) {

  std::string fileName = ros::package::getPath("object_models") + "/models/rgbd-dataset/apple/apple_1/apple_1_1_100.pcd";
  SegmentedObject segmentedObject1 = buildSegmentedObject(fileName);
  SegmentedObject segmentedObject2 = buildSegmentedObject(fileName);

  ContactPointsDetector contactPointsDetector = ContactPointsDetector();
  contactPointsDetector.setSegmentedObjects(&segmentedObject1,&segmentedObject2);
  contactPointsDetector.calculateRelationshipValue();

  if(contactPointsDetector.detectedRelationship)
  {
    boost::shared_ptr<ContactPointsRelationship> contactPointsRelationship = contactPointsDetector.getContactPointsRelationship();
  }
  
}
