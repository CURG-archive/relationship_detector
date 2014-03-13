#ifndef CONTACT_POINTS_DETECTOR_H
#define CONTACT_POINTS_DETECTOR_H

#include "relationship_detector.h"
#include "contact_points_relationship.h"
#include <boost/shared_ptr.hpp>

class ContactPointsDetector: public RelationshipDetector
{
	public:
		
		virtual void computeRelationship();
}; 

#endif