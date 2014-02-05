#include <ros/ros.h>
#include "relationship_detector/SegmentedObjectList.h"
#include "relationship_detector/RelatedObjectsList.h"
#include "std_msgs/String.h"

namespace relationship_detector_node
{
    class RelationshipDetectorNode
    {
        private:
            ros::NodeHandle node_handle;

            ros::Subscriber relationship_detector_sub;
            void segmentedObjectListMessageCallback(const relationship_detector::SegmentedObjectList::ConstPtr &msg);

            ros::Publisher detectedRelationshipsPublisher;




        public:
            RelationshipDetectorNode();
    };


    RelationshipDetectorNode::RelationshipDetectorNode(): node_handle("")
    {
        relationship_detector_sub = node_handle.subscribe("segmented_objects", 1000, &RelationshipDetectorNode::segmentedObjectListMessageCallback, this);

        detectedRelationshipsPublisher = node_handle.advertise<relationship_detector::RelatedObjectsList>("related_objects",10);

        ROS_INFO("relationship_detection_node ready");
    }


    void RelationshipDetectorNode::segmentedObjectListMessageCallback(const relationship_detector::SegmentedObjectList::ConstPtr &msg)
    {
        ROS_INFO("Received segmented object list message");
        //takes the list of point clouds and converts it to a list of PropertyObjects
        //convertToPropertyObjects(msg.get()->segmentedObjects());
        //detectProperties(msg);
        //detect
        relationship_detector::RelatedObjectsList relatedObjectsList;
        detectedRelationshipsPublisher.publish(relatedObjectsList);
    }

}



int main(int argc, char **argv) 
{
  ros::init(argc, argv, "relationship_detector_node");
  ros::NodeHandle nh;

  relationship_detector_node::RelationshipDetectorNode node;

  ros::spin();
  return 0;
}
