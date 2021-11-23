#pragma once
#ifndef UTILS_H
#define UTILS_H

#include <vector>
#include <tuple>

struct Point3D{
    float x;
    float y;
    float z;

    uint8_t r;
    uint8_t g;
    uint8_t b;

    Point3D() {
        x = 0;
        y = 0;
        z = 0;
        r = 0;
        g = 0;
        b = 0;
    }
};

#endif