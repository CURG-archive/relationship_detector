#ifndef PROPERTY_FACTORY_H
#define PROPERTY_FACTORY_H

#include "center_of_mass_property.h"
#include <boost/shared_ptr.hpp>


class PropertyFactory
{
    public:
        static PropertyFactory& getInstance()
        {
            static PropertyFactory instance; // Guaranteed to be destroyed.
                                             // Instantiated on first use.
            return instance;
        }

        int computeNextPropertyId()
		{
			nextPropertyId += 1;
			return nextPropertyId;
		};

		boost::shared_ptr<CenterOfMassProperty> buildCenterOfMassProperty(int segmentedObjectId, geometry_msgs::Point center_of_mass_point);

    private:

    	int nextPropertyId;

        PropertyFactory() {};   // Constructor (the {} brackets) are needed here.

        // Dont forget to declare these two. You want to make sure they
        // are unaccessable otherwise you may accidently get copies of
        // your singleton appearing.
        PropertyFactory(PropertyFactory const&);// Don't Implement
        void operator=(PropertyFactory const&); // Don't implement
};



#endif