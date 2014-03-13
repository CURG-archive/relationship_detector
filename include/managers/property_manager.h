#ifndef PROPERTY_MANAGER_H
#define PROPERTY_MANAGER_H

#include <ros/package.h>
#include <unordered_map>
#include <map>

#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/functional/factory.hpp>

#include "propertyEnum.h"
#include "segmented_object.h"
#include "center_of_mass_property.h"
#include "center_of_mass_detector.h"
#include "pc_property.h"
#include "property_detector.h"

using namespace std;
namespace boost {
        // Add in a simple wrapper for a factor creating shared pointers
                template<typename T>
        struct shared_factory : public factory<shared_ptr<T> >
        { };
}

class PropertyManager
{
protected:
  typedef boost::function<PropertyDetectorPtr () > PropertyDetectorFactory;
  typedef map<int, PropertyDetectorFactory> PropertyDetectorFactoryMap;
  typedef int PropertyType;

  static const int NUM_PROPERTIES = 1;
  //map of already detected properties
  //key: hash(object_id and property_type)
  //value: property
  unordered_map<int, boost::shared_ptr<PCProperty>> propertyMap;
  PropertyDetectorFactoryMap propertyDetectorFactoryMap;

public:
  //constructor 
  PropertyManager()
  {
    propertyMap.reserve(7000);
    //register the properties to their detectors
    propertyDetectorFactoryMap[PropertyClassIds::CENTER_OF_MASS] = boost::shared_factory<CenterOfMassDetector>();
    // propertyDetectorFactoryMap[PropertyClassIds::COLOR] = boost::shared_factory<ColorPropertyDetector>();    
    // propertyDetectorFactoryMap[PropertyClassIds::ORIENTATION] = boost::shared_factory<OrientationPropertyDetector>();
  }


  boost::shared_ptr<PCProperty> getProperty(SegmentedObject *segmentedObject, PropertyType property_type)
  {
    int property_hash = pow(2,segmentedObject->uniqueId)*pow(3,property_type);
    if(propertyMap.find(property_hash) != propertyMap.end())
    {
      return propertyMap[property_hash];
    }

    boost::shared_ptr<PropertyDetector> detector = propertyDetectorFactoryMap[property_type]();
    detector->setSegmentedObject(segmentedObject);
    detector->computeProperty();
    propertyMap[property_hash] = detector->getProperty();
    return detector->getProperty();
  }

  std::vector<boost::shared_ptr<PCProperty>> getAllProperties(SegmentedObject *segmentedObject)
  {
    std::vector<boost::shared_ptr<PCProperty>> allProperties;
    
    for (int i=0; i<NUM_PROPERTIES; i++)
    {
      allProperties.push_back(this->getProperty(segmentedObject, i));
    }
    return allProperties;
  }

};

#endif