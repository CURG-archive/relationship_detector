#ifndef PC_PROPERTY_H
#define PC_PROPERTY_H

class PCProperty
{
  	public:
  		//unique id for this property
  		int propertyId;

  		//unique id for the object which has this property
    	int segmentedObjectId;

      PCProperty(int property_id, int segmented_object_id)
      {
      	propertyId = property_id;
      	segmentedObjectId = segmented_object_id;
      };

      virtual ~PCProperty(){};
    
};

#endif