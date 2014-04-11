#include <ros/ros.h>
#include "perception_msgs/ObjectCenterProperty.h"
#include "perception_msgs/RecognizedObjectList.h"
#include "perception_msgs/RecognizedObject.h"
#include "gtest/gtest.h"
#include "std_msgs/String.h"
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/package.h>

namespace relationship_detector_node_test
{
    class TestRelationshipDetectorNode
    {
        private:
            ros::NodeHandle node_handle;


        public:
            TestRelationshipDetectorNode();

            //will publish recognized objects for the relationshipDetectorNode to look at
            ros::Publisher recognizedObjectsPublisher;

            ros::Subscriber detectedPropertiesSubscriber;
            void relatedObjectsMessageCallback(const perception_msgs::ObjectCenterProperty::ConstPtr &msg);

            bool hasReceivedMessage;
            perception_msgs::ObjectCenterProperty receivedMsg;
    };


    TestRelationshipDetectorNode::TestRelationshipDetectorNode(): node_handle("")
    {
        detectedPropertiesSubscriber = node_handle.subscribe("object_properties", 1000, &TestRelationshipDetectorNode::relatedObjectsMessageCallback, this);

        recognizedObjectsPublisher = node_handle.advertise<perception_msgs::RecognizedObjectList>("recognized_objects",10);

        hasReceivedMessage = false;

        ROS_INFO("test_relationship_detection_node ready");
    }


    void TestRelationshipDetectorNode::relatedObjectsMessageCallback(const perception_msgs::ObjectCenterProperty::ConstPtr &msg)
    {
        ROS_INFO("Received related object list message");
        hasReceivedMessage = true;
        receivedMsg = *msg;
    }

}


//create a node to send messages to the relationship_detector_node so we can validate its responses.
relationship_detector_node_test::TestRelationshipDetectorNode buildTestNode()
{

    relationship_detector_node_test::TestRelationshipDetectorNode node;

    //give test node time to initialize VERY IMPORTANT
    ros::Duration(1).sleep();

    return node;
}


perception_msgs::RecognizedObjectList buildAppleRecognizedObjectsList()
{
    //get point cloud
    std::string fileName = ros::package::getPath("object_models") + "/models/rgbd-dataset/apple_1/apple_1_1_100.pcd";

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ> (fileName, *cloud) == -1) //* load the file
    {
      PCL_ERROR ("Couldn't read file \n");
    }
    sensor_msgs::PointCloud2 sensorMessagePointCloud;
    pcl::toROSMsg(*cloud,sensorMessagePointCloud);

    //build recognized object and add point cloud to it
    perception_msgs::RecognizedObject recognizedObject;
    recognizedObject.recognizedObjectID = 1;
    recognizedObject.recognizedObjectPointCloud = sensorMessagePointCloud;

    //build recognizedObjectList and add recognized object to it.
    perception_msgs::RecognizedObjectList recognizedObjectsList;
    recognizedObjectsList.recognizedObjects.push_back(recognizedObject);
    return recognizedObjectsList;
}


TEST(RELATIONSHIP_DETECTOR_TEST_NODE, TestEmptyRecognizedObjectsList) {

  relationship_detector_node_test::TestRelationshipDetectorNode node = buildTestNode();
  perception_msgs::RecognizedObjectList recognizedObjectsList = buildAppleRecognizedObjectsList();

  node.recognizedObjectsPublisher.publish(recognizedObjectsList);

  double startTimeInSeconds =ros::Time::now().toSec();
  while(!node.hasReceivedMessage)
  {
      ros::spinOnce();
      double currentTimeInSeconds =ros::Time::now().toSec();

      //we should have received a message by now, something is wrong.
      if(currentTimeInSeconds-startTimeInSeconds > 5)
      {
          break;
      }
  }

  EXPECT_EQ(node.hasReceivedMessage, true);

  double absErrorBound = .0001;
  ASSERT_NEAR(node.receivedMsg.objectCenter.x, -0.0127071, absErrorBound);
  ASSERT_NEAR(node.receivedMsg.objectCenter.y, 0.699493, absErrorBound);
  ASSERT_NEAR(node.receivedMsg.objectCenter.z, -0.0152639, absErrorBound);
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_relationship_detector_node");
  ros::NodeHandle nh;

  //give the relationship_detector_node time to start up.
  ros::Duration(1).sleep();

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

