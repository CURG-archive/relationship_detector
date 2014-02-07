#ifndef CONTACT_POINTS_RELATIONSHIP_H
#define CONTACT_POINTS_RELATIONSHIP_H

#include "relationship.h"
#include <geometry_msgs/Point.h>
#include <vector>



class ContactPointsRelationship: public Relationship
{
	public:
		std::vector<geometry_msgs::Point> contactPoints;

		ContactPointsRelationship(int relationship_id, SegmentedObject *segmented_object_1, SegmentedObject *segmented_object_2, std::vector<geometry_msgs::Point> contact_points)
			:Relationship(relationship_id, segmented_object_1,segmented_object_2 )
		{
			contactPoints = contact_points;
		};	

}; 

#endif