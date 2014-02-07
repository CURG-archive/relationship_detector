#include <ros/ros.h>
#include "relationship_detector/SegmentedObject.h"
#include "relationship_detector/SegmentedObjectList.h"
#include "relationship_detector/RelatedObjectsList.h"
#include "relationship_detector/ObjectCenterProperty.h"
#include "std_msgs/String.h"
#include "center_of_mass_detector.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <boost/shared_ptr.hpp>
#include <pcl_ros/point_cloud.h>
#include "property_factory.h"
#include "segmented_object.h"

namespace relationship_detector_node
{
    class RelationshipDetectorNode
    {
        private:
            ros::NodeHandle node_handle;

            ros::Subscriber relationship_detector_sub;
            void segmentedObjectListMessageCallback(const relationship_detector::SegmentedObjectList::ConstPtr &msg);

            ros::Publisher detectedPropertyPublisher;

        public:
            RelationshipDetectorNode();
    };


    RelationshipDetectorNode::RelationshipDetectorNode(): node_handle("")
    {

        relationship_detector_sub = node_handle.subscribe("segmented_objects", 1000, &RelationshipDetectorNode::segmentedObjectListMessageCallback, this);

        detectedPropertyPublisher = node_handle.advertise<relationship_detector::ObjectCenterProperty>("object_properties",10);

        ROS_INFO("relationship_detection_node ready");
    }


    void RelationshipDetectorNode::segmentedObjectListMessageCallback(const relationship_detector::SegmentedObjectList::ConstPtr &msg)
    {
        ROS_INFO("Received segmented object list message");

        CenterOfMassDetector centerOfMassDetector;

        int numSegmentedObjects = msg.get()->segmentedObjects.size();

        for(int i = 0; i< numSegmentedObjects; i++)
        {
            //get segmentedObject from message
            relationship_detector::SegmentedObject segmentedObjectMsg = msg.get()->segmentedObjects[i];
            sensor_msgs::PointCloud2 sensorMessagePointCloud = segmentedObjectMsg.segmentedObjectPointCloud;
            pcl::PCLPointCloud2 segmentedObjectPCLPointCloud;
            pcl::PointCloud<pcl::PointXYZ>::Ptr segmentedObjectPCLPointCloudXYZ(new pcl::PointCloud<pcl::PointXYZ>());
            pcl_conversions::toPCL(sensorMessagePointCloud, segmentedObjectPCLPointCloud);
            pcl::fromPCLPointCloud2(segmentedObjectPCLPointCloud, *segmentedObjectPCLPointCloudXYZ);
            SegmentedObject segmentedObject = SegmentedObject(segmentedObjectMsg.segmentedObjectID, segmentedObjectPCLPointCloudXYZ);

            //extract property from pointcloud
            centerOfMassDetector.setSegmentedObject(&segmentedObject);

            centerOfMassDetector.calculatePropertyValue();
            boost::shared_ptr<CenterOfMassProperty> centerOfMassProperty = centerOfMassDetector.getCenterOfMassProperty();

            //build property message from property
            relationship_detector::ObjectCenterProperty objectCenterPropertyMessage;
            objectCenterPropertyMessage.objectCenter = centerOfMassProperty->centerOfMassPoint;
            objectCenterPropertyMessage.segmentedObjectId = centerOfMassProperty->segmentedObjectId;
            objectCenterPropertyMessage.propertyId = centerOfMassProperty->propertyId;

            //send message
            detectedPropertyPublisher.publish(objectCenterPropertyMessage);
        }
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
