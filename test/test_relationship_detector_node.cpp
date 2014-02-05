#include <ros/ros.h>
#include "relationship_detector/RelatedObjectsList.h"
#include "relationship_detector/SegmentedObjectList.h"
#include "gtest/gtest.h"
#include "std_msgs/String.h"

namespace relationship_detector_node_test
{
    class TestRelationshipDetectorNode
    {
        private:
            ros::NodeHandle node_handle;

        public:
            TestRelationshipDetectorNode();

            //will publish segmented objects for the relationshipDetectorNode to look at
            ros::Publisher segmentedObjectsPublisher;

            ros::Subscriber related_objects_sub;
            void relatedObjectsMessageCallback(const relationship_detector::RelatedObjectsList::ConstPtr &msg);

            bool hasReceivedMessage;
    };


    TestRelationshipDetectorNode::TestRelationshipDetectorNode(): node_handle("")
    {
        related_objects_sub = node_handle.subscribe("related_objects", 1000, &TestRelationshipDetectorNode::relatedObjectsMessageCallback, this);

        segmentedObjectsPublisher = node_handle.advertise<relationship_detector::SegmentedObjectList>("segmented_objects",10);

        hasReceivedMessage = false;

        ROS_INFO("test_relationship_detection_node ready");
    }


    void TestRelationshipDetectorNode::relatedObjectsMessageCallback(const relationship_detector::RelatedObjectsList::ConstPtr &msg)
    {
        ROS_INFO("Received related object list message");
        hasReceivedMessage = true;


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


relationship_detector::SegmentedObjectList buildEmptySegmentedObjectsList()
{
    relationship_detector::SegmentedObjectList segmentedObjectsList;
    segmentedObjectsList.test = "hello world!!!";
    return segmentedObjectsList;
}


TEST(RELATIONSHIP_DETECTOR_TEST_NODE, TestEmptySegmentedObjectsList) {

  relationship_detector_node_test::TestRelationshipDetectorNode node = buildTestNode();
  relationship_detector::SegmentedObjectList segmentedObjectsList = buildEmptySegmentedObjectsList();

  node.segmentedObjectsPublisher.publish(segmentedObjectsList);

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

