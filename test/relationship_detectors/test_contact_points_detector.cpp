#include <ros/package.h>
#include <string>
#include "gtest/gtest.h"
#include "contact_points_detector.h"
#include <iostream>
#include "build_recognized_objects.h"
//script to run tests : catkin_make run_tests

// int nextRecognizedObjectId = 0;

// RecognizedObject buildRecognizedObject(std::string filename)
// {

//   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
//   if (pcl::io::loadPCDFile<pcl::PointXYZ> (filename, *cloud_ptr) == -1) //* load the file
//   {
//     PCL_ERROR ("Couldn't read file \n");
//   }

//   nextRecognizedObjectId +=1;
//   RecognizedObject recognizedObject = RecognizedObject(nextRecognizedObjectId, cloud_ptr);

//   return recognizedObject;

// }

TEST(ContactPointsDetector, TestTwoObjectsTouching) {

  std::string fileName1 = ros::package::getPath("object_models") + "/models/rgbd-dataset/test_data/apple_1_1_1.pcd";
  std::string fileName2 = ros::package::getPath("object_models") + "/models/rgbd-dataset/test_data/apple_trans_x_touch.pcd";
  RecognizedObject recognizedObject1 = buildRecognizedObject(fileName1);
  RecognizedObject recognizedObject2 = buildRecognizedObject(fileName2);

  ContactPointsDetector contactPointsDetector = ContactPointsDetector();
  contactPointsDetector.setRecognizedObjects(&recognizedObject1,&recognizedObject2);
  contactPointsDetector.computeRelationship();

  EXPECT_EQ(contactPointsDetector.detectedRelationship, true);
  
}

TEST(ContactPointsDetector, TestTwoObjectsNotTouching) {

  std::string fileName1 = ros::package::getPath("object_models") + "/models/rgbd-dataset/test_data/apple_1_1_1.pcd";
  std::string fileName2 = ros::package::getPath("object_models") + "/models/rgbd-dataset/test_data/apple_trans_x_not_touching.pcd";
  RecognizedObject recognizedObject1 = buildRecognizedObject(fileName1);
  RecognizedObject recognizedObject2 = buildRecognizedObject(fileName2);

  ContactPointsDetector contactPointsDetector = ContactPointsDetector();
  contactPointsDetector.setRecognizedObjects(&recognizedObject1,&recognizedObject2);
  contactPointsDetector.computeRelationship();

  EXPECT_EQ(contactPointsDetector.detectedRelationship, false);
}
