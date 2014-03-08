#include <ros/ros.h>
#include "perception_msgs/SegmentedObject.h"
#include "perception_msgs/SegmentedObjectList.h"
#include "perception_msgs/ObjectCenterProperty.h"
#include "std_msgs/String.h"
#include "center_of_mass_detector.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <boost/shared_ptr.hpp>
#include <pcl_ros/point_cloud.h>
#include "segmented_object.h"

namespace relationship_detector_node
{
    class RelationshipDetectorNode
    {
        private:
            ros::NodeHandle node_handle;

            ros::Subscriber relationship_detector_sub;
            void segmentedObjectListMessageCallback(const perception_msgs::SegmentedObjectList::ConstPtr &msg);

            ros::Publisher detectedPropertyPublisher;

        public:
            RelationshipDetectorNode();
    };


    RelationshipDetectorNode::RelationshipDetectorNode(): node_handle("")
    {

        relationship_detector_sub = node_handle.subscribe("segmented_objects", 1000, &RelationshipDetectorNode::segmentedObjectListMessageCallback, this);

        detectedPropertyPublisher = node_handle.advertise<perception_msgs::ObjectCenterProperty>("object_properties",10);

        ROS_INFO("relationship_detection_node ready");
    }


    void RelationshipDetectorNode::segmentedObjectListMessageCallback(const perception_msgs::SegmentedObjectList::ConstPtr &msg)
    {
        ROS_INFO("Received segmented object list message");

        PropertyManager *pm= new PropertyManager()
        RelationshipManager *rm = new RelationshipManager();
        int numSegmentedObjects = msg.get()->segmentedObjects.size();

        for(int i = 0; i< numSegmentedObjects; i++)
        {
            //get segmentedObject from message
            perception_msgs::SegmentedObject segmentedObjectMsg = msg.get()->segmentedObjects[i];
            sensor_msgs::PointCloud2 sensorMessagePointCloud = segmentedObjectMsg.segmentedObjectPointCloud;
            pcl::PCLPointCloud2 segmentedObjectPCLPointCloud;
            pcl::PointCloud<pcl::PointXYZ>::Ptr segmentedObjectPCLPointCloudXYZ(new pcl::PointCloud<pcl::PointXYZ>());
            pcl_conversions::toPCL(sensorMessagePointCloud, segmentedObjectPCLPointCloud);
            pcl::fromPCLPointCloud2(segmentedObjectPCLPointCloud, *segmentedObjectPCLPointCloudXYZ);
            SegmentedObject segmentedObject = SegmentedObject(segmentedObjectMsg.segmentedObjectID, segmentedObjectPCLPointCloudXYZ);
            pm.getAllProperties(segmentedObject);
        }

        for(int i = 0; i< numSegmentedObjects; i+=2)
        {
            //get segmentedObject from message
            perception_msgs::SegmentedObject segmentedObjectMsg1 = msg.get()->segmentedObjects[i];
            sensor_msgs::PointCloud2 sensorMessagePointCloud1 = segmentedObjectMsg1.segmentedObjectPointCloud;
            pcl::PCLPointCloud2 segmentedObjectPCLPointCloud1;
            pcl::PointCloud<pcl::PointXYZ>::Ptr segmentedObjectPCLPointCloudXYZ1(new pcl::PointCloud<pcl::PointXYZ>());
            pcl_conversions::toPCL(sensorMessagePointCloud1, segmentedObjectPCLPointCloud1);
            pcl::fromPCLPointCloud2(segmentedObjectPCLPointCloud1, *segmentedObjectPCLPointCloudXYZ1);
            SegmentedObject segmentedObject1 = SegmentedObject(segmentedObjectMsg1.segmentedObjectID, segmentedObjectPCLPointCloudXYZ1);

            perception_msgs::SegmentedObject segmentedObjectMsg2 = msg.get()->segmentedObjects[i+1];
            sensor_msgs::PointCloud2 sensorMessagePointCloud2 = segmentedObjectMsg2.segmentedObjectPointCloud;
            pcl::PCLPointCloud2 segmentedObjectPCLPointCloud2;
            pcl::PointCloud<pcl::PointXYZ>::Ptr segmentedObjectPCLPointCloudXYZ2(new pcl::PointCloud<pcl::PointXYZ>());
            pcl_conversions::toPCL(sensorMessagePointCloud2, segmentedObjectPCLPointCloud2);
            pcl::fromPCLPointCloud2(segmentedObjectPCLPointCloud2, *segmentedObjectPCLPointCloudXYZ2);
            SegmentedObject segmentedObject2 = SegmentedObject(segmentedObjectMsg2.segmentedObjectID, segmentedObjectPCLPointCloudXYZ2);

            rm.getAllRelationships(segmentedObject1,segmentedObject2);
        }
    }

/*
        for(each segmentedObject )
            pm.detectAllProperties(segmenteObject)

        for (segObject1)
            for (segObject2)
                rm.detectAllRelationships(segObject1,segObject2);

        for all properties in pm.propertiesMap.values:
            publish property
        for all relationships in rm.relationshipsMap.values:
            publish relationship
*/

//____________________________________________________________________
//Old Code Please Delete
        // int numSegmentedObjects = msg.get()->segmentedObjects.size();

        // for(int i = 0; i< numSegmentedObjects; i++)
        // {
        //     //get segmentedObject from message
        //     perception_msgs::SegmentedObject segmentedObjectMsg = msg.get()->segmentedObjects[i];
        //     sensor_msgs::PointCloud2 sensorMessagePointCloud = segmentedObjectMsg.segmentedObjectPointCloud;
        //     pcl::PCLPointCloud2 segmentedObjectPCLPointCloud;
        //     pcl::PointCloud<pcl::PointXYZ>::Ptr segmentedObjectPCLPointCloudXYZ(new pcl::PointCloud<pcl::PointXYZ>());
        //     pcl_conversions::toPCL(sensorMessagePointCloud, segmentedObjectPCLPointCloud);
        //     pcl::fromPCLPointCloud2(segmentedObjectPCLPointCloud, *segmentedObjectPCLPointCloudXYZ);
        //     SegmentedObject segmentedObject = SegmentedObject(segmentedObjectMsg.segmentedObjectID, segmentedObjectPCLPointCloudXYZ);

        //     //extract property from pointcloud
        //     centerOfMassDetector.setSegmentedObject(&segmentedObject);

        //     centerOfMassDetector.computeProperty();
        //     boost::shared_ptr<PCProperty> centerOfMassProperty = centerOfMassDetector.getProperty();

        //     //build property message from property
        //     // perception_msgs::ObjectCenterProperty objectCenterPropertyMessage;
        //     // objectCenterPropertyMessage.objectCenter = centerOfMassProperty->centerOfMassPoint;
        //     // objectCenterPropertyMessage.segmentedObjectId = centerOfMassProperty->segmentedObjectId;
        //     // objectCenterPropertyMessage.propertyId = centerOfMassProperty->uniquePropertyId;

        //     //send message
        //     // detectedPropertyPublisher.publish(objectCenterPropertyMessage);
        // }
    //}
//}



int main(int argc, char **argv) 
{

  ros::init(argc, argv, "relationship_detector_node");
  ros::NodeHandle nh;

  relationship_detector_node::RelationshipDetectorNode node;

  ros::spin();
  return 0;
}
