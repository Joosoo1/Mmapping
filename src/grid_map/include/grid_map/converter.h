//
// Created by ma on 23-8-17.
//

#ifndef LLA2ENU_CONVERTER_H
#define LLA2ENU_CONVERTER_H

#include <vector>
#include <sensor_msgs/NavSatFix.h>

class Converter
{
private:
    static const float PI;
    float lon0, lat0, alt0;

public:
    Converter();

    void convert_init(float a, float b, float c);
    std::vector<float> lla2xyz(float lon, float lat, float alt);
    std::vector<float> xyz2enu(float x1, float y1, float z1, float x2, float y2, float z2);
    std::vector<float> lla2enu(const sensor_msgs::NavSatFix::ConstPtr& gps);

    ~Converter();
};

#endif // LLA2ENU_CONVERTER_H

