#ifndef PROPERTY_MANAGER_H
#define PROPERTY_MANAGER_H

#include <ros/package.h>
#include <unordered_map>

#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/functional/factory.hpp>

#include "segmented_object.h"
#include "center_of_mass_detector.h"
#include "pc_property.h"
#include "property_detector.h"

class PropertyManager
{

  //map of already detected properties
  //key: hash(object_id and property_type)
  //value: property
  unordered_map<int, PCProperty> propertyMap;

  typedef boost::function<PropertyPtr (SegmentedObject &segmentedObject)> PropertyDetectorFactory;
  typedef map<int, PropertyDetectorFactory> PropertyDetectorFactoryMap;
  typedef int PropertyType;
  PropertyDetectorMap PropertyDetectorFactoryMap;

  //constructor 
  PropertyManager()
  {
    //register the properties to their detectors
    propertyDetectorMap[CenterOfMassProperty.uniquePropertyType] = boost::shared_factory<CenterOfMassPropertyDetector>();
    // propertyDetectorMap[ColorProperty.uniquePropertyType] = boost::shared_factory<ColorPropertyDetector>();
    // propertyDetectorMap[OrientationProperty.uniquePropertyType] = boost::shared_factory<OrientationPropertyDetector>();
  }


  PCProperty * getProperty(SegmentedObject *segmentedObject, PropertyType property_type)
  {
    int property_hash = pow(2,segmentedObject->uniqueId)*pow(3,property_type);
    if(propertyMap.find(property_hash) != propertyMap.end())
    {
      return propertyMap[property_hash];
    }

    detector = propertyDetectorMap[property_type]();
    detector.setSegmentedObject(segmentedObject);
    detector.computeProperty();
    propertyMap[property_hash] = computedProperty;
    return property;
  }

  std::vector<PCProperty> getAllProperty(SegmentedObject *segmentedObject)
  {
    std::vector<PCProperty> allProperties;
    
    allProperties.push_back(this->getProperty(segmentedObject, 1));
    // allProperties.push_back(this->getProperty(segmentedObject, 2));
    // allProperties.push_back(this->getProperty(segmentedObject, 3));
    return allProperties;
  }

};

#endif