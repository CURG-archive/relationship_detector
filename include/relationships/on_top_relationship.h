#ifndef ON_TOP_RELATIONSHIP_H
#define ON_TOP_RELATIONSHIP_H

#include "relationship.h"
#include "relationshipEnum.h"
#include <geometry_msgs/Point.h>
#include <vector>


class OnTopRelationship: public Relationship
{
	public:
		bool isOnTop;
		int onTopObjectId;

		OnTopRelationship(bool is_on_top, int on_top_object_id)
		{
			uniqueRelationshipType = ON_TOP;
			uniqueRelationshipId = getNextRelationshipId();
			isOnTop = is_on_top;
			onTopObjectId = on_top_object_id;
		};

}; 

#endif