#include "fast_simulator/Kinect.h"
#include "fast_simulator/World.h"
#include "virtual_cam/Loader.h"

using namespace std;

Kinect::Kinect(const string& rgb_topic, const string& depth_topic, const string& info_topic, const string& frame_id) {
    image_loader_ = new ImageLoader(rgb_topic, depth_topic, info_topic, frame_id);
}

Kinect::~Kinect() {
    delete image_loader_;
}

void Kinect::addModel(const std::string& type, const std::string& filename) {
    type_to_filename_[type] = filename;
}

void Kinect::publish() {
    World& world = World::getInstance();
    map<string, Object*> objects = world.getObjects();

    string filename = "";
    for(map<string, Object*>::const_iterator it_obj = objects.begin(); it_obj != objects.end(); ++it_obj) {
        Object& obj = *it_obj->second;

        map<string, string>::const_iterator it_filename = type_to_filename_.find(obj.getType());
        if (it_filename == type_to_filename_.end()) {
            it_filename = type_to_filename_.find(obj.getID());
        }

        if (it_filename != type_to_filename_.end()) {
            // TODO: check if in view
            // ...

            filename = it_filename->second;
        }
    }

    if (filename != "") {
        if (filename != loaded_file_) {
            image_loader_->load(filename);
            loaded_file_ = filename;
        }
        image_loader_->publish();
    }

}
