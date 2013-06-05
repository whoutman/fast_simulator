#ifndef _FAST_SIMULATOR_MODEL_PARSER_H_
#define _FAST_SIMULATOR_MODEL_PARSER_H_

#include "Object.h"

//#include <nav_msgs/OccupancyGrid.h>
#include <tf/tf.h>
#include <string>

#include "fast_simulator/Object.h"
#include "fast_simulator/Box.h"

class TiXmlElement;

class ModelParser {

public:

    ModelParser(const std::string& filename, const std::string& model_dir);

    virtual ~ModelParser();

    std::vector<double> parseArray(const TiXmlElement* xml_elem);

    bool parse(std::map<std::string, Object> &models);

    std::string getError() const;

    Object* parseHeightMap(const TiXmlElement* xml_elem);

    /*
    void createQuadTree(const nav_msgs::OccupancyGrid& map, unsigned int mx_min, unsigned int my_min,
             unsigned int mx_max, unsigned int my_max, double height, Object* parent, std::string indent = "");
    */
    //void initFromTopic(const std::string& topic);

protected:

    std::string filename_;

    std::string model_dir_;

    std::stringstream error_;

    //nav_msgs::OccupancyGrid world_map_;

    //tf::Transform map_transform_;
    //tf::Transform map_transform_inverse_;

    //void callbackMap(const nav_msgs::OccupancyGrid::ConstPtr& msg);

};

#endif
