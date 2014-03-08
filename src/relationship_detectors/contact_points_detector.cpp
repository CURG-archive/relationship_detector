#include "contact_points_detector.h"

void ContactPointsDetector::calculateRelationshipValue()
{
    std::vector<geometry_msgs::Point> contactPoints;
    
    //create kdtree of segmented object 1
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(segmentedObject1->pointCloudPtr);

    //create a point which will iterate over segmented object 2
    pcl::PointXYZ searchPoint;
    float radius = 0.005;
    int max_nn = 1;

	std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

 for(int i =0; i < segmentedObject2->pointCloudPtr->size() ; i++)
    {
        searchPoint.x = segmentedObject2->pointCloudPtr->points[i].x;
        searchPoint.y = segmentedObject2->pointCloudPtr->points[i].y;
        searchPoint.z = segmentedObject2->pointCloudPtr->points[i].z;

        if(kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance,max_nn) > 0)
        {
            for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
            {
                std::cout << "    "  << segmentedObject1->pointCloudPtr->points[ pointIdxRadiusSearch[i] ].x
                          << " " << segmentedObject1->pointCloudPtr->points[ pointIdxRadiusSearch[i] ].y
                          << " " << segmentedObject1->pointCloudPtr->points[ pointIdxRadiusSearch[i] ].z
                          << " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
            }
            //contactPoints.push_back(searchPoint);
            detectedRelationship = true;
        }
    }
    if(detectedRelationship)
    {
        std::cout << "Relationship Detected";
		//contactPointsRelationship = RelationshipFactory::getInstance().buildContactPointsRelationship(segmentedObject1, segmentedObject2, contactPoints);
    }
}

boost::shared_ptr<ContactPointsRelationship> ContactPointsDetector::getContactPointsRelationship()
{
	return contactPointsRelationship;
}
