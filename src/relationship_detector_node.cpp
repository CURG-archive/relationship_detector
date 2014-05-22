#include <ros/ros.h>
#include <iomanip>
#include <locale>
#include <sstream>
#include <string>

#include "perception_msgs/RecognizedObject.h"
#include "perception_msgs/RecognizedObjectList.h"
#include "perception_msgs/ObjectCenterProperty.h"
#include "std_msgs/String.h"
#include "center_of_mass_detector.h"
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl/ros/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <boost/shared_ptr.hpp>
#include <pcl_ros/point_cloud.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include "property_manager.h"
#include "relationship_manager.h"
#include "recognized_object.h"
#include "pc_property.h"

using namespace std;

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
            static bool RELATIONSHIPS_GENERATED;
            RelationshipDetectorNode();
    };

    bool RelationshipDetectorNode::RELATIONSHIPS_GENERATED = false;
    
    RelationshipDetectorNode::RelationshipDetectorNode(): node_handle("")
    {
        relationship_detector_sub = node_handle.subscribe("recognizedObjectList", 1000, &RelationshipDetectorNode::recognizedObjectListMessageCallback, this);
        ROS_INFO("Recognized objects ready");
        detectedPropertyPublisher = node_handle.advertise<perception_msgs::ObjectCenterProperty>("object_properties",10);
        ROS_INFO("relationship_detection_node ready");
    }

    void RelationshipDetectorNode::recognizedObjectListMessageCallback(const perception_msgs::RecognizedObjectList::ConstPtr &msg)
    {
        ROS_INFO("Received recognized object list message");
        PropertyManager *pm;
        tf::StampedTransform transform;
        std::vector<RecognizedObject> allTransformedObjects;
        pm = PropertyManager::getInstance();
        ROS_INFO("PM instance retrieved!");
        RelationshipManager *rm;
        rm = RelationshipManager::getInstance();
        ROS_INFO("RM instance retrieved!");
        int numRecognizedObjects = msg.get()->recognizedObjects.size();
        if(numRecognizedObjects > 0) ROS_INFO("Getting %i models!",numRecognizedObjects);
        else ROS_INFO("No recognized objects!");

        for(int i = 0; i< numRecognizedObjects; i++)
        {
            //get recognizedObject from message
            perception_msgs::RecognizedObject recognizedObjectMsg = msg.get()->recognizedObjects[i];
            sensor_msgs::PointCloud2 sensorMessagePointCloud = recognizedObjectMsg.recognizedObjectPointCloud;
            
            pcl::PCLPointCloud2 recognizedObjectPCLPointCloud;
            pcl::PointCloud<pcl::PointXYZ>::Ptr recognizedObjectPCLPointCloudXYZ(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::PointCloud<pcl::PointXYZ>::Ptr transformedObjectPCLPointCloudXYZ(new pcl::PointCloud<pcl::PointXYZ>());
            pcl_conversions::toPCL(sensorMessagePointCloud, recognizedObjectPCLPointCloud);
            pcl::fromPCLPointCloud2(recognizedObjectPCLPointCloud, *recognizedObjectPCLPointCloudXYZ);
            tf::TransformListener listener;
            // tranforming point clouds to a single reference frame
            try
            {
                std::cout<<" "<<std::endl;
                ROS_INFO("Calculating Properties of %s!", recognizedObjectMsg.recognizedObjectPointCloud.header.frame_id.c_str());
                listener.waitForTransform("/camera_depth_frame", recognizedObjectMsg.recognizedObjectPointCloud.header.frame_id, ros::Time(0), ros::Duration(10.0) );
                listener.lookupTransform("/camera_depth_frame", recognizedObjectMsg.recognizedObjectPointCloud.header.frame_id, ros::Time(0), transform);
                ROS_INFO("Got Transformation!");
            } 
            catch (tf::TransformException ex) 
            {
                ROS_ERROR("%s",ex.what());
                ROS_INFO("No Transformation Received!");
            }
            ROS_INFO("Transforming Point Clouds!");
            pcl_ros::transformPointCloud(*recognizedObjectPCLPointCloudXYZ, *transformedObjectPCLPointCloudXYZ, transform);
            transformedObjectPCLPointCloudXYZ->header.frame_id = "/camera_depth_frame";
            
            // uncomment to save recognized objects sent from model_rec_manager to raw_pcd folder
            stringstream pcdFile;
            string recognizedObjectNumber = static_cast<ostringstream*>( &(ostringstream() << i) )->str();
            pcdFile << "/home/vmalpani/perception_ws/raw_pcd/recognized_object_" << i <<".pcd";
            pcl::io::savePCDFileASCII (pcdFile.str(), *transformedObjectPCLPointCloudXYZ);
            pcdFile.clear();
            ROS_INFO("Saving Recognized Point Cloud %s at %s", recognizedObjectNumber.c_str(),pcdFile.str().c_str());

            RecognizedObject recognizedObject = RecognizedObject(recognizedObjectMsg.recognizedObjectID, transformedObjectPCLPointCloudXYZ);
            allTransformedObjects.push_back(recognizedObject);
            ROS_INFO("Using Property Manager!");
            std::vector<boost::shared_ptr<PCProperty>> objProperties = pm->getAllProperties(&recognizedObject);
        }
        std::cout<<" "<<std::endl;
        std::cout<<" "<<std::endl;
        ROS_INFO("Properties Calculated!!");

        for(int i=0; i<numRecognizedObjects; i+=1)
        {
            //get recognizedObject1 from allTransformedObjects vector
            RecognizedObject recognizedObject1 = allTransformedObjects.at(i);
            for(int j=i+1; j<numRecognizedObjects; j+=1)
            {
                //get recognizedObject2 from allTransformedObjects vector
                RecognizedObject recognizedObject2 = allTransformedObjects.at(j);
                std::cout<<" "<<std::endl;
                ROS_INFO("Generating Relationships between Objects %d and %d!",recognizedObject1.uniqueId, recognizedObject2.uniqueId);
                std::vector<boost::shared_ptr<Relationship>> objRelationships = rm->getAllRelationships(&recognizedObject1,&recognizedObject2);
            }       
        }
        RelationshipDetectorNode::RELATIONSHIPS_GENERATED = true;
        std::cout<<" "<<std::endl;
        std::cout<<" "<<std::endl;
        ROS_INFO("Relationships Generated!!");
        std::cout<<" "<<std::endl;
        std::cout<<" "<<std::endl;
        ROS_INFO("DONE...");
        ros::shutdown();
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
