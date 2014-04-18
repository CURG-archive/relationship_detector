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
#include "recognized_object.h"
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
  private:
    static PropertyManager *pm;
    static bool instanceFlag;
    PropertyManager();

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
    static PropertyManager* getInstance();

    boost::shared_ptr<PCProperty> getProperty(RecognizedObject *recognizedObject, PropertyType property_type);
    std::vector<boost::shared_ptr<PCProperty>> getAllProperties(RecognizedObject *recognizedObject);

};

#endif