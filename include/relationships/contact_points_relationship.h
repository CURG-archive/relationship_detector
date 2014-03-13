#ifndef CONTACT_POINTS_RELATIONSHIP_H
#define CONTACT_POINTS_RELATIONSHIP_H

#include "relationship.h"
#include <geometry_msgs/Point.h>
#include <vector>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>



class ContactPointsRelationship: public Relationship
{
	public:
		std::vector<pcl::PointXYZ> contactPoints;

		ContactPointsRelationship(std::vector<pcl::PointXYZ> contact_points)
		{
			uniqueRelationshipType = 1;
			uniqueRelationshipId = getNextRelationshipId();
			contactPoints = contact_points;
		};
}; 

#endif
