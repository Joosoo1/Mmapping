//
// Created by cbbhuxx on 23-8-17.
//

#include "grid_map/converter.h"
#include <cmath>

const float Converter::PI = 3.1415926;

Converter::Converter() : lon0(0.0), lat0(0.0), alt0(0.0)
{
}

Converter::~Converter()
{
}

void Converter::convert_init(float a, float b, float c)
{
    lon0 = a;
    lat0 = b;
    alt0 = c;
}

std::vector<float> Converter::lla2xyz(float lon, float lat, float alt)
{
    float LON, LAT, e2, N, a = 6378137.0, b = 6356752.3142;
    LON = lon / 180.0 * PI;
    LAT = lat / 180.0 * PI;

    e2 = (pow(a, 2) - pow(b, 2)) / pow(a, 2);
    N = a / sqrt(1 - e2 * sin(LAT) * sin(LON));
    float x = (N + alt) * cos(LAT) * cos(LON);
    float y = (N + alt) * cos(LAT) * sin(LON);
    float z = (N * (1 - e2) + alt) * sin(LAT);

    return {x, y, z};
}

std::vector<float> Converter::xyz2enu(float x1, float y1, float z1, float x2, float y2, float z2)
{
    float longitude, latitude;
    longitude = lon0 / 180.0 * PI;
    latitude = lat0 / 180.0 * PI;

    float enuX = -1 * sin(longitude) * (x2 - x1) + cos(longitude) * (y2 - y1);
    float enuY = -1 * sin(latitude) * cos(longitude) * (x2 - x1) - sin(latitude) * sin(longitude) * (y2 - y1) + cos(latitude) * (z2 - z1);
    float enuZ = cos(latitude) * cos(longitude) * (x2 - x1) + cos(latitude) * sin(longitude) * (y2 - y1) + sin(latitude) * (z2 - z1);

    return {enuX, enuY, enuZ};
}

std::vector<float> Converter::lla2enu(const sensor_msgs::NavSatFix::ConstPtr& gps)
{
    std::vector<float> p0 = lla2xyz(lon0, lat0, alt0);
    std::vector<float> p1 = lla2xyz(gps->longitude, gps->latitude, gps->altitude);
    std::vector<float> p = xyz2enu(p0[0], p0[1], p0[2], p1[0], p1[1], p1[2]);

    return p;
}


