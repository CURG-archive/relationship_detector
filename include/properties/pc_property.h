#ifndef PC_PROPERTY_H
#define PC_PROPERTY_H

class PCProperty
{

  protected:
    static int property_id_generator;

  public:
    int uniquePropertyType;

    int uniquePropertyId;

    int segmentedObjectId;

    int getNextPropertyId()
    {
      PCProperty::property_id_generator +=1;
      return property_id_generator;
    }
    virtual ~PCProperty(){};

};

    int PCProperty::property_id_generator = 0;

#endif
