#include "relationship_detector.h"
#include "contact_points_relationship.h"
#include <boost/shared_ptr.hpp>

class ContactPointsDetector: public RelationshipDetector
{
	protected:

		boost::shared_ptr<ContactPointsRelationship> contactPointsRelationship;

	public:
		
		virtual void calculateRelationshipValue();

		boost::shared_ptr<ContactPointsRelationship> getContactPointsRelationship();


}; 