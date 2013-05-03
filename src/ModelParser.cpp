#include "fast_simulator/ModelParser.h"

#include <tinyxml.h>

using namespace std;

ModelParser::ModelParser(const std::string& filename) : filename_(filename) {

}

ModelParser::~ModelParser() {

}

vector<double> ModelParser::parseArray(const TiXmlElement* xml_elem) {
    std::string txt = xml_elem->GetText();

    vector<double> v;

    string word;
    stringstream stream(txt);
    while( getline(stream, word, ' ') ) {
        v.push_back(atof(word.c_str()));
    }

    return v;
}

bool ModelParser::parse(map<std::string, Object>& models) {
    TiXmlDocument doc(filename_);
    doc.LoadFile();

    if (doc.Error()) {
        error_ << "While parsing '" << filename_ << "': " << endl << endl << doc.ErrorDesc() << " at line " << doc.ErrorRow() << ", col " << doc.ErrorCol() << endl;
        return false;
    }

    const TiXmlElement* model_xml = doc.FirstChildElement("model");

    /* PARSE ALL MODELS */

    while (model_xml) {

        const char* name = model_xml->Attribute("name");
        if (name) {
            cout << "Parsing model '" << name << "'" << endl;

            vector<double> xyz(3, 0);
            vector<double> rpy;
            vector<double> size;

            Object model(name);

            const TiXmlElement* shape_xml = model_xml->FirstChildElement();
            while(shape_xml) {
                string shape_type = shape_xml->Value();
                if (shape_type == "box") {
                    const TiXmlElement* xyz_xml = shape_xml->FirstChildElement("xyz");
                    if (xyz_xml) {
                        xyz = parseArray(xyz_xml);
                    }

                    const TiXmlElement* rpy_xml = shape_xml->FirstChildElement("rpy");
                    if (rpy_xml) {
                        rpy = parseArray(rpy_xml);
                        if (fabs(rpy[0]) < 0.0001 && fabs(rpy[1]) < 0.0001 && fabs(rpy[2]) < 0.0001) {
                            rpy.clear();
                        }
                    }

                    const TiXmlElement* size_xml = shape_xml->FirstChildElement("size");
                    if (size_xml) {
                        size = parseArray(size_xml);

                        tf::Vector3 v_pos(xyz[0], xyz[1], xyz[2]);
                        tf::Vector3 v_size(size[0], size[1], size[2]);

                        Object* obj = new Object();
                        if (rpy.empty()) {
                            obj->setShape(Box(v_pos - v_size / 2, v_pos + v_size / 2));
                        } else {
                            obj->setShape(Box(-v_size / 2, v_size / 2));
                            tf::Quaternion q;
                            q.setRPY(rpy[0], rpy[1], rpy[2]);
                            obj->setPose(v_pos, q);
                        }
                        model.addChild(obj);
                    } else {
                        error_ << "In definition for model '" << name << "': shape '" << shape_type << "' has no size property" << endl;
                    }

                } else {
                    error_ << "In definition for model '" << name << "': Unknown shape type: '" << shape_type << "'" << endl;
                }

                shape_xml = shape_xml->NextSiblingElement();
            }

            if (error_.str().empty()) {
                cout << "... Parsing successfully ..." << endl;
                models[name] = model;
            }
        } else {
            error_ << "Encountered model without 'name' attribute." << endl;
        }

        model_xml = model_xml->NextSiblingElement("model");
    }

    if (error_.str() != "") {
        return false;
    }

    return true;
}

std::string ModelParser::getError() const {
    return error_.str();
}
