#ifndef ON_TOP_RELATIONSHIP_H
#define ON_TOP_RELATIONSHIP_H

#include "relationship.h"
#include <geometry_msgs/Point.h>
#include <vector>


class OnTopRelationship: public Relationship
{
	public:
		bool isOnTop;

		OnTopRelationship(int relationship_id, SegmentedObject *segmented_object_1, SegmentedObject *segmented_object_2, bool is_on_top)
			:Relationship(relationship_id, segmented_object_1,segmented_object_2 )
		{
			isOnTop = is_on_top;
		};	

}; 

#endif
