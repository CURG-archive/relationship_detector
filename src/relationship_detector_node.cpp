#include <ros/ros.h>
#include "perception_msgs/RecognizedObject.h"
#include "perception_msgs/RecognizedObjectList.h"
#include "perception_msgs/ObjectCenterProperty.h"
#include "std_msgs/String.h"
#include "center_of_mass_detector.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <boost/shared_ptr.hpp>
#include <pcl_ros/point_cloud.h>
#include "property_manager.h"
#include "relationship_manager.h"
#include "recognized_object.h"
#include "pc_property.h"

namespace relationship_detector_node
{
    class RelationshipDetectorNode
    {
        private:
            ros::NodeHandle node_handle;
            ros::Publisher detectedPropertyPublisher;
            ros::Subscriber relationship_detector_sub;

            void recognizedObjectListMessageCallback(const perception_msgs::RecognizedObjectList::ConstPtr &msg);

        public:
            RelationshipDetectorNode();
    };


    RelationshipDetectorNode::RelationshipDetectorNode(): node_handle("")
    {
        relationship_detector_sub = node_handle.subscribe("recognizedObjectList", 1000, &RelationshipDetectorNode::recognizedObjectListMessageCallback, this);
        ROS_INFO("recognized objects ready");
        detectedPropertyPublisher = node_handle.advertise<perception_msgs::ObjectCenterProperty>("object_properties",10);
        ROS_INFO("relationship_detection_node ready");
    }

    void RelationshipDetectorNode::recognizedObjectListMessageCallback(const perception_msgs::RecognizedObjectList::ConstPtr &msg)
    {
        ROS_INFO("Received recognized object list message");

        PropertyManager pm;
        RelationshipManager rm;
        int numRecognizedObjects = msg.get()->recognizedObjects.size();

        for(int i = 0; i< numRecognizedObjects; i++)
        {
            //get recognizedObject from message
            perception_msgs::RecognizedObject recognizedObjectMsg = msg.get()->recognizedObjects[i];
            sensor_msgs::PointCloud2 sensorMessagePointCloud = recognizedObjectMsg.recognizedObjectPointCloud;
            pcl::PCLPointCloud2 recognizedObjectPCLPointCloud;
            pcl::PointCloud<pcl::PointXYZ>::Ptr recognizedObjectPCLPointCloudXYZ(new pcl::PointCloud<pcl::PointXYZ>());
            pcl_conversions::toPCL(sensorMessagePointCloud, recognizedObjectPCLPointCloud);
            pcl::fromPCLPointCloud2(recognizedObjectPCLPointCloud, *recognizedObjectPCLPointCloudXYZ);
            RecognizedObject recognizedObject = RecognizedObject(recognizedObjectMsg.recognizedObjectID, recognizedObjectPCLPointCloudXYZ);
            std::vector<boost::shared_ptr<PCProperty>> objProperties = pm.getAllProperties(&recognizedObject);
        }

        for(int i = 0; i< numRecognizedObjects; i+=2)
        {
            //get recognizedObject1 from message
            perception_msgs::RecognizedObject recognizededObjectMsg1 = msg.get()->recognizedObjects[i];
            sensor_msgs::PointCloud2 sensorMessagePointCloud1 = recognizededObjectMsg1.recognizedObjectPointCloud;
            pcl::PCLPointCloud2 recognizedObjectPCLPointCloud1;
            pcl::PointCloud<pcl::PointXYZ>::Ptr recognizedObjectPCLPointCloudXYZ1(new pcl::PointCloud<pcl::PointXYZ>());
            pcl_conversions::toPCL(sensorMessagePointCloud1, recognizedObjectPCLPointCloud1);
            pcl::fromPCLPointCloud2(recognizedObjectPCLPointCloud1, *recognizedObjectPCLPointCloudXYZ1);
            RecognizedObject recognizedObject1 = RecognizedObject(recognizededObjectMsg1.recognizedObjectID, recognizedObjectPCLPointCloudXYZ1);

            //get recognizedObject2 from message
            perception_msgs::RecognizedObject recognizedObjectMsg2 = msg.get()->recognizedObjects[i+1];
            sensor_msgs::PointCloud2 sensorMessagePointCloud2 = recognizedObjectMsg2.recognizedObjectPointCloud;
            pcl::PCLPointCloud2 recognizedObjectPCLPointCloud2;
            pcl::PointCloud<pcl::PointXYZ>::Ptr recognizedObjectPCLPointCloudXYZ2(new pcl::PointCloud<pcl::PointXYZ>());
            pcl_conversions::toPCL(sensorMessagePointCloud2, recognizedObjectPCLPointCloud2);
            pcl::fromPCLPointCloud2(recognizedObjectPCLPointCloud2, *recognizedObjectPCLPointCloudXYZ2);
            RecognizedObject recognizedObject2 = RecognizedObject(recognizedObjectMsg2.recognizedObjectID, recognizedObjectPCLPointCloudXYZ2);

            std::vector<boost::shared_ptr<Relationship>> objRelationships = rm.getAllRelationships(&recognizedObject1,&recognizedObject2);
        }
    }

    // void RelationshipDetectorNode::recognizedObjectListMessageCallback(const perception_msgs::RecognizedObjectList::ConstPtr &msg)
    // {
    //     ROS_INFO("Received recognized object list message");

    //     PropertyManager pm;
    //     RelationshipManager rm;
    //     int numRecognizedObjects = msg.get()->recognizedObjects.size();

    //     for(int i = 0; i< numRecognizedObjects; i++)
    //     {
    //         //get recognizedObject from message
    //         perception_msgs::RecognizedObject recognizedObjectMsg = msg.get()->recognizedObjects[i];
    //         sensor_msgs::PointCloud2 sensorMessagePointCloud = recognizedObjectMsg.recognizedObjectPointCloud;
    //         pcl::PCLPointCloud2 recognizedObjectPCLPointCloud;
    //         pcl::PointCloud<pcl::PointXYZ>::Ptr recognizedObjectPCLPointCloudXYZ(new pcl::PointCloud<pcl::PointXYZ>());
    //         pcl_conversions::toPCL(sensorMessagePointCloud, recognizedObjectPCLPointCloud);
    //         pcl::fromPCLPointCloud2(recognizedObjectPCLPointCloud, *recognizedObjectPCLPointCloudXYZ);
    //         RecognizedObject recognizedObject = RecognizedObject(recognizedObjectMsg.recognizedObjectID, recognizedObjectPCLPointCloudXYZ);
    //         std::vector<boost::shared_ptr<PCProperty>> objProperties = pm.getAllProperties(&recognizedObject);
    //     }

    //     for(int i = 0; i< numRecognizedObjects; i+=2)
    //     {
    //         //get recognizedObject1 from message
    //         perception_msgs::RecognizedObject recognizededObjectMsg1 = msg.get()->recognizedObjects[i];
    //         sensor_msgs::PointCloud2 sensorMessagePointCloud1 = recognizededObjectMsg1.recognizedObjectPointCloud;
    //         pcl::PCLPointCloud2 recognizedObjectPCLPointCloud1;
    //         pcl::PointCloud<pcl::PointXYZ>::Ptr recognizedObjectPCLPointCloudXYZ1(new pcl::PointCloud<pcl::PointXYZ>());
    //         pcl_conversions::toPCL(sensorMessagePointCloud1, recognizedObjectPCLPointCloud1);
    //         pcl::fromPCLPointCloud2(recognizedObjectPCLPointCloud1, *recognizedObjectPCLPointCloudXYZ1);
    //         RecognizedObject recognizedObject1 = RecognizedObject(recognizededObjectMsg1.recognizedObjectID, recognizedObjectPCLPointCloudXYZ1);

    //         //get recognizedObject2 from message
    //         perception_msgs::RecognizedObject recognizedObjectMsg2 = msg.get()->recognizedObjects[i+1];
    //         sensor_msgs::PointCloud2 sensorMessagePointCloud2 = recognizedObjectMsg2.recognizedObjectPointCloud;
    //         pcl::PCLPointCloud2 recognizedObjectPCLPointCloud2;
    //         pcl::PointCloud<pcl::PointXYZ>::Ptr recognizedObjectPCLPointCloudXYZ2(new pcl::PointCloud<pcl::PointXYZ>());
    //         pcl_conversions::toPCL(sensorMessagePointCloud2, recognizedObjectPCLPointCloud2);
    //         pcl::fromPCLPointCloud2(recognizedObjectPCLPointCloud2, *recognizedObjectPCLPointCloudXYZ2);
    //         RecognizedObject recognizedObject2 = RecognizedObject(recognizedObjectMsg2.recognizedObjectID, recognizedObjectPCLPointCloudXYZ2);

    //         std::vector<boost::shared_ptr<Relationship>> objRelationships = rm.getAllRelationships(&recognizedObject1,&recognizedObject2);
    //     }
    // }
}


int main(int argc, char **argv) 
{

  ros::init(argc, argv, "relationship_detector_node");
  ros::NodeHandle nh;
  relationship_detector_node::RelationshipDetectorNode node;
  ros::spin();
  return 0;
}
