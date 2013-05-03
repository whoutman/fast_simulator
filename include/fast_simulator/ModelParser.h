#ifndef _FAST_SIMULATOR_MODEL_PARSER_H_
#define _FAST_SIMULATOR_MODEL_PARSER_H_

#include "Object.h"

class TiXmlElement;

class ModelParser {

public:

    ModelParser(const std::string& filename);

    virtual ~ModelParser();

    std::vector<double> parseArray(const TiXmlElement* xml_elem);

    bool parse(std::map<std::string, Object> &models);

    std::string getError() const;

protected:

    std::string filename_;

    std::stringstream error_;

};

#endif
