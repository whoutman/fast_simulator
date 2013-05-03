#ifndef _FAST_SIMULATOR_MODEL_PARSER_H_
#define _FAST_SIMULATOR_MODEL_PARSER_H_

#include "Object.h"

class ModelParser {

public:

    ModelParser(const std::string& filename);

    virtual ~ModelParser();

    Object* parse();

    std::string getError() const;

protected:

    std::string filename_;

    std::stringstream error_;

};

#endif
