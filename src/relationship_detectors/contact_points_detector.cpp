#include "contact_points_detector.h"
#include "relationship_factory.h"

void ContactPointsDetector::calculateRelationshipValue()
{
    std::vector<geometry_msgs::Point> contactPoints;
    detectedRelationship = true;
    contactPointsRelationship = RelationshipFactory::getInstance().buildContactPointsRelationship(segmentedObject1, segmentedObject2, contactPoints);
}

boost::shared_ptr<ContactPointsRelationship> ContactPointsDetector::getContactPointsRelationship()
{
	return contactPointsRelationship;
}
