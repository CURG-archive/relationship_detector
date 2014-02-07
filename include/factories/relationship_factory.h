#ifndef RELATIONSHIP_FACTORY_H
#define RELATIONSHIP_FACTORY_H

#include "contact_points_relationship.h"
#include <boost/shared_ptr.hpp>


class RelationshipFactory
{
    public:
        static RelationshipFactory& getInstance()
        {
            static RelationshipFactory instance; // Guaranteed to be destroyed.
                                             // Instantiated on first use.
            return instance;
        }

        int computeNextRelationshipId()
		{
			nextRelationshipId += 1;
			return nextRelationshipId;
		};

		boost::shared_ptr<ContactPointsRelationship> buildContactPointsRelationship(SegmentedObject *segmented_object_1, SegmentedObject *segmented_object_2 ,std::vector<geometry_msgs::Point> contact_points);

    private:

    	int nextRelationshipId;

        RelationshipFactory() {};   // Constructor (the {} brackets) are needed here.

        // Dont forget to declare these two. You want to make sure they
        // are unaccessable otherwise you may accidently get copies of
        // your singleton appearing.
        RelationshipFactory(RelationshipFactory const&);// Don't Implement
        void operator=(RelationshipFactory const&); // Don't implement
};



#endif