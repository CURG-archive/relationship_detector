#ifndef CONTACT_POINTS_RELATIONSHIP_H
#define CONTACT_POINTS_RELATIONSHIP_H

#include "relationship.h"
#include "relationshipEnum.h"
#include <geometry_msgs/Point.h>
#include <vector>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>



class ContactPointsRelationship: public Relationship
{
	public:
		std::vector<pcl::PointXYZ> contactPoints;
		bool isTouching;

		ContactPointsRelationship(std::vector<pcl::PointXYZ> contact_points, bool is_touching)
		{
			uniqueRelationshipType = CONTACT_POINTS;
			uniqueRelationshipId = getNextRelationshipId();
			contactPoints = contact_points;
			isTouching = is_touching;
		};
}; 

#endif
