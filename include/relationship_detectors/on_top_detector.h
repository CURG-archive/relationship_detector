#include "relationship_detector.h"
#include "on_top_relationship.h"
#include <boost/shared_ptr.hpp>

class OnTopDetector: public RelationshipDetector
{
	protected:

		boost::shared_ptr<OnTopRelationship> onTopRelationship;

	public:
		
		virtual void calculateRelationshipValue();

		boost::shared_ptr<OnTopRelationship> getContactPointsRelationship();


}; 