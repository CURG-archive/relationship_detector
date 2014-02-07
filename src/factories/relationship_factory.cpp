

#include "relationship_factory.h"


boost::shared_ptr<ContactPointsRelationship> RelationshipFactory::buildContactPointsRelationship(SegmentedObject *segmented_object_1, SegmentedObject *segmented_object_2, std::vector<geometry_msgs::Point> contact_points)
{
	int relationshipId = computeNextRelationshipId();

	return boost::shared_ptr<ContactPointsRelationship>(new ContactPointsRelationship(relationshipId,segmented_object_1,segmented_object_2, contact_points));
}
