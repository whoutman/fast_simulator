#include "fast_simulator/ModelParser.h"

#include <tinyxml.h>

using namespace std;

ModelParser::ModelParser(const std::string& filename) : filename_(filename) {

}

ModelParser::~ModelParser() {

}

Object* ModelParser::parse() {
    TiXmlDocument doc(filename_);
    doc.LoadFile();

    if (doc.Error()) {
        error_ << "While parsing '" << filename_ << "': " << endl << endl << doc.ErrorDesc() << " at line " << doc.ErrorRow() << ", col " << doc.ErrorCol() << endl;
        return 0;
    }

    const TiXmlElement* model_xml = doc.FirstChildElement("model");

    /* PARSE ALL MODELS */

    while (model_xml) {

        const char* name = model_xml->Attribute("name");
        if (name) {
            cout << "Parsing model '" << name << "'" << endl;

        const TiXmlElement* shape_xml = model_xml->FirstChildElement();
        while(shape_xml) {
            string shape_type = shape_xml->Value();
            if (shape_type == "box") {

            } else {
                error_ << "In definition for model '" << name << "': Unknown shape type: '" << shape_type << "'" << endl;
            }

            shape_xml = shape_xml->NextSiblingElement();
        }


        } else {
            error_ << "Encountered model without 'name' attribute." << endl;
        }

        model_xml = model_xml->NextSiblingElement("model");
    }

    if (error_.str() != "") {
        return 0;
    }

    return new Object();
}

std::string ModelParser::getError() const {
    return error_.str();
}
