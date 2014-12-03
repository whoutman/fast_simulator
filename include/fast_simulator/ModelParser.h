#ifndef _FAST_SIMULATOR_MODEL_PARSER_H_
#define _FAST_SIMULATOR_MODEL_PARSER_H_

#include "Object.h"

//#include <nav_msgs/OccupancyGrid.h>
#include <tf/tf.h>
#include <string>

#include "fast_simulator/Object.h"
//#include "fast_simulator/Box.h"

#include <geolib/HeightMap.h>

class TiXmlElement;

class ModelParser {

public:

    ModelParser(const std::string& filename, const std::string& model_dir);

    virtual ~ModelParser();

    std::vector<double> parseArray(const TiXmlElement* xml_elem);

    Object* parse(const std::string& model_name, const std::string& id, std::string& error);

    Object* parseHeightMap(const TiXmlElement* xml_elem, std::stringstream& s_error);

    static geo::HeightMap getHeightMapFromImage(const std::string& filename, double height, double resolution);

protected:

    std::string filename_;

    std::string model_dir_;

};

#endif
