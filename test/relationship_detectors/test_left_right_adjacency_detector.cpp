#include <ros/package.h>
#include <string>
#include <iostream>

#include "gtest/gtest.h"
#include "left_right_adjacency_detector.h"
#include "build_recognized_objects.h"

//script to run tests : catkin_make run_tests

TEST(LeftRightAdjacencyDetector, TestTwoObjectsOnTop) {

  std::string fileName1 = ros::package::getPath("object_models") + "/models/rgbd-dataset/test_data/apple_1_1_1.pcd";
  std::string fileName2 = ros::package::getPath("object_models") + "/models/rgbd-dataset/test_data/apple_trans_x_touch.pcd";
  RecognizedObject recognizedObject1 = buildRecognizedObject(fileName1);
  RecognizedObject recognizedObject2 = buildRecognizedObject(fileName2);

  LeftRightAdjacencyDetector leftRightAdjacencyDetector = LeftRightAdjacencyDetector();
  leftRightAdjacencyDetector.setRecognizedObjects(&recognizedObject1,&recognizedObject2);
  leftRightAdjacencyDetector.computeRelationship();

  EXPECT_EQ(leftRightAdjacencyDetector.detectedRelationship, true); 
}

TEST(LeftRightAdjacencyDetector, TestTwoObjectsNotOnTop) {

  std::string fileName1 = ros::package::getPath("object_models") + "/models/rgbd-dataset/test_data/apple_1_1_1.pcd";
  std::string fileName2 = ros::package::getPath("object_models") + "/models/rgbd-dataset/test_data/apple_trans_x_not_touching.pcd";
  RecognizedObject recognizedObject1 = buildRecognizedObject(fileName1);
  RecognizedObject recognizedObject2 = buildRecognizedObject(fileName2);

  LeftRightAdjacencyDetector leftRightAdjacencyDetector = LeftRightAdjacencyDetector();
  leftRightAdjacencyDetector.setRecognizedObjects(&recognizedObject1,&recognizedObject2);
  leftRightAdjacencyDetector.computeRelationship();

  EXPECT_EQ(leftRightAdjacencyDetector.detectedRelationship, false);
}