#ifndef LEFT_RIGHT_ADJACENCY_DETECTOR_H
#define LEFT_RIGHT_ADJACENCY_DETECTOR_H

#include "relationship_detector.h"
#include "left_right_adjacency_relationship.h"

#include <boost/shared_ptr.hpp>

class LeftRightAdjacencyDetector: public RelationshipDetector
{
	public:
		
		virtual void computeRelationship();
}; 

#endif