#include "pc_property.h"

int PCProperty::property_id_generator = 0;

int PCProperty::getNextPropertyId()
{
  PCProperty::property_id_generator +=1;
  return property_id_generator;
}