#include <ros/package.h>
#include <string>
#include "gtest/gtest.h"
#include "contact_points_detector.h"
#include <iostream>

//script to run tests : catkin_make run_tests

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

TEST(ContactPointsDetector, TestTwoObjectsTouching) {

  std::string fileName1 = ros::package::getPath("object_models") + "/models/rgbd-dataset/test_data/apple_1_1_1.pcd";
  std::string fileName2 = ros::package::getPath("object_models") + "/models/rgbd-dataset/test_data/apple_trans_x_touch.pcd";
  SegmentedObject segmentedObject1 = buildSegmentedObject(fileName1);
  SegmentedObject segmentedObject2 = buildSegmentedObject(fileName2);

  ContactPointsDetector contactPointsDetector = ContactPointsDetector();
  contactPointsDetector.setSegmentedObjects(&segmentedObject1,&segmentedObject2);
  contactPointsDetector.computeRelationship();

  EXPECT_EQ(contactPointsDetector.detectedRelationship, true);
  
}

TEST(ContactPointsDetector, TestTwoObjectsNotTouching) {

  std::string fileName1 = ros::package::getPath("object_models") + "/models/rgbd-dataset/test_data/apple_1_1_1.pcd";
  std::string fileName2 = ros::package::getPath("object_models") + "/models/rgbd-dataset/test_data/apple_trans_x_not_touching.pcd";
  SegmentedObject segmentedObject1 = buildSegmentedObject(fileName1);
  SegmentedObject segmentedObject2 = buildSegmentedObject(fileName2);

  ContactPointsDetector contactPointsDetector = ContactPointsDetector();
  contactPointsDetector.setSegmentedObjects(&segmentedObject1,&segmentedObject2);
  contactPointsDetector.computeRelationship();

  EXPECT_EQ(contactPointsDetector.detectedRelationship, false);
}
