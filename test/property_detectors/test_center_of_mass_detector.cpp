#include <ros/package.h>
#include <string>
#include "gtest/gtest.h"
#include "center_of_mass_detector.h"
#include <iostream>
#include <boost/shared_ptr.hpp>
#include <memory>
#include <boost/pointer_cast.hpp>

//catkin_make run_tests


TEST(ExampleTestCase, ExampleTestName) {
  EXPECT_EQ(1, 1);
}


TEST(CenterOfMassDetector, TestAppleCenterOfMass) {

  std::string fileName = ros::package::getPath("object_models") + "/models/rgbd-dataset/apple_1/apple_1_1_100.pcd";

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile<pcl::PointXYZ> (fileName, *cloud_ptr) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file \n");
  }
 
  int unique_object_id = 1;
  RecognizedObject recognizedObject = RecognizedObject(unique_object_id, cloud_ptr);

  CenterOfMassDetector centerOfMassDetector = CenterOfMassDetector();

  centerOfMassDetector.setRecognizedObject(&recognizedObject);
  centerOfMassDetector.computeProperty();

  boost::shared_ptr<PCProperty> centerOfMassProperty = centerOfMassDetector.getProperty();
  boost::shared_ptr<CenterOfMassProperty> com;
  com = boost::dynamic_pointer_cast<CenterOfMassProperty>(centerOfMassProperty);

  double absErrorBound = .0001;
  ASSERT_NEAR(com->centerOfMassPoint.x, -0.0127071, absErrorBound);
  ASSERT_NEAR(com->centerOfMassPoint.y, 0.699493, absErrorBound);
  ASSERT_NEAR(com->centerOfMassPoint.z, -0.0152639, absErrorBound);

}






