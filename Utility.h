#pragma once
#ifndef _UTILITY_H_
#define _UTILITY_H_

#include <vector>
#include "structs.h"

bool isPointInsideTriangle(double x, double y, double z, const Triangle& t)
{
    double detT = (t.y2 - t.y3) * (t.x1 - t.x3) + (t.x3 - t.x2) * (t.y1 - t.y3);
    double lambda1 = ((t.y2 - t.y3) * (x - t.x3) + (t.x3 - t.x2) * (y - t.y3)) / detT;
    double lambda2 = ((t.y3 - t.y1) * (x - t.x3) + (t.x1 - t.x3) * (y - t.y3)) / detT;
    double lambda3 = 1 - lambda1 - lambda2;
    return lambda1 >= 0 && lambda1 <= 1 && lambda2 >= 0 && lambda2 <= 1 && lambda3 >= 0 && lambda3 <= 1;
}

bool isPointInsidePolyhedron(Point p, const std::vector<Triangle>& triangles) {
    int numIntersections = 0;

    double dx = 0.5;
    double x1 = p.x;
    double y1 = p.y;
    double z1 = p.z;
    double x2 = x1 + dx;
    double y2 = y1 + dx;
    double z2 = z1 + dx;

    for (const Triangle& tri : triangles) {
        double nx = (tri.y2 - tri.y1) * (tri.z3 - tri.z1) - (tri.z2 - tri.z1) * (tri.y3 - tri.y1);
        double ny = (tri.z2 - tri.z1) * (tri.x3 - tri.x1) - (tri.x2 - tri.x1) * (tri.z3 - tri.z1);
        double nz = (tri.x2 - tri.x1) * (tri.y3 - tri.y1) - (tri.y2 - tri.y1) * (tri.x3 - tri.x1);

        double denom = nx * (x2 - x1) + ny * (y2 - y1) + nz * (z2 - z1);
        if (denom != 0) {
            double t = ((tri.x1 - x1) * nx + (tri.y1 - y1) * ny + (tri.z1 - z1) * nz) / denom;
            double x = x1 + t * (x2 - x1);
            double y = y1 + t * (y2 - y1);
            double z = z1 + t * (z2 - z1);

            if (t >= 0 && t <= 1 && isPointInsideTriangle(x, y, z, tri)) {
                numIntersections++;
            }
        }
    }
    return numIntersections % 2 == 1;
}

#endif

