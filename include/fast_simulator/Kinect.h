#ifndef _FAST_SIMULATOR_KINECT_H_
#define _FAST_SIMULATOR_KINECT_H_

#include "Sensor.h"

class ImageLoader;

class Kinect : public Sensor {

public:

    Kinect(const std::string& rgb_topic, const std::string& depth_topic, const std::string& info_topic, const std::string& frame_id);

    virtual ~Kinect();

    void addModel(const std::string& type, const std::string& filename);

    void publish();

protected:

    ImageLoader* image_loader_;

    std::string loaded_file_;

    std::map<std::string, std::string> type_to_filename_;

};

#endif
