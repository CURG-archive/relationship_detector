#ifndef LEFT_RIGHT_ADJACENCY_RELATIONSHIP_H
#define LEFT_RIGHT_ADJACENCY_RELATIONSHIP_H

#include "relationship.h"
#include <geometry_msgs/Point.h>
#include <vector>


class LeftRightAdjacencyRelationship: public Relationship
{
	public:
		bool isLeftRightAdjacent;
		int leftObjectId, rightObjectId;

		LeftRightAdjacencyRelationship(bool is_left_right_adjacent, int left_object_id, int right_object_id)
		{
			uniqueRelationshipType = 3;
			uniqueRelationshipId = getNextRelationshipId();
			isLeftRightAdjacent = is_left_right_adjacent;
			leftObjectId = left_object_id;
			rightObjectId = right_object_id;
		};

}; 

#endif