#include "contact_points_detector.h"

void ContactPointsDetector::computeRelationship()
{
    std::vector<pcl::PointXYZ> contactPoints;
    
    //create kdtree of recognized object 1
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(recognizedObject1->pointCloudPtr);
    detectedRelationship = false;

    //create a point which will iterate over recognized object 2
    pcl::PointXYZ searchPoint;
    float radius = 0.005;
    int max_nn = 1;

	std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

 for(int i =0; i < recognizedObject2->pointCloudPtr->size() ; i++)
    {
        searchPoint.x = recognizedObject2->pointCloudPtr->points[i].x;
        searchPoint.y = recognizedObject2->pointCloudPtr->points[i].y;
        searchPoint.z = recognizedObject2->pointCloudPtr->points[i].z;

        if(kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance,max_nn) > 0)
        {
            // for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
            // {
            //     std::cout << "    "  << recognizedObject1->pointCloudPtr->points[ pointIdxRadiusSearch[i] ].x
            //               << " " << recognizedObject1->pointCloudPtr->points[ pointIdxRadiusSearch[i] ].y
            //               << " " << recognizedObject1->pointCloudPtr->points[ pointIdxRadiusSearch[i] ].z
            //               << " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
            // }
            contactPoints.push_back(searchPoint);
            detectedRelationship = true;
        }
    }
    if(detectedRelationship)
    {
        std::cout << "Relationship Detected\n";
    }

    Relationship *cp = new ContactPointsRelationship(contactPoints,detectedRelationship);
    computedRelationship = boost::shared_ptr<Relationship>(cp);
}