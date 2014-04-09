#include <ros/package.h>
#include <string>
#include <iostream>

#include "gtest/gtest.h"
#include "left_right_adjacency_detector.h"
#include "build_segmented_objects.h"

//script to run tests : catkin_make run_tests

TEST(LeftRightAdjacencyDetector, TestTwoObjectsOnTop) {

  std::string fileName1 = ros::package::getPath("object_models") + "/models/rgbd-dataset/test_data/apple_1_1_1.pcd";
  std::string fileName2 = ros::package::getPath("object_models") + "/models/rgbd-dataset/test_data/apple_trans_x_touch.pcd";
  SegmentedObject segmentedObject1 = buildSegmentedObject(fileName1);
  SegmentedObject segmentedObject2 = buildSegmentedObject(fileName2);

  LeftRightAdjacencyDetector leftRightAdjacencyDetector = LeftRightAdjacencyDetector();
  leftRightAdjacencyDetector.setSegmentedObjects(&segmentedObject1,&segmentedObject2);
  leftRightAdjacencyDetector.computeRelationship();

  EXPECT_EQ(leftRightAdjacencyDetector.detectedRelationship, true); 
}

TEST(LeftRightAdjacencyDetector, TestTwoObjectsNotOnTop) {

  std::string fileName1 = ros::package::getPath("object_models") + "/models/rgbd-dataset/test_data/apple_1_1_1.pcd";
  std::string fileName2 = ros::package::getPath("object_models") + "/models/rgbd-dataset/test_data/apple_trans_x_not_touching.pcd";
  SegmentedObject segmentedObject1 = buildSegmentedObject(fileName1);
  SegmentedObject segmentedObject2 = buildSegmentedObject(fileName2);

  LeftRightAdjacencyDetector leftRightAdjacencyDetector = LeftRightAdjacencyDetector();
  leftRightAdjacencyDetector.setSegmentedObjects(&segmentedObject1,&segmentedObject2);
  leftRightAdjacencyDetector.computeRelationship();

  EXPECT_EQ(leftRightAdjacencyDetector.detectedRelationship, false);
}