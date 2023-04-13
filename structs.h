#pragma once
#include <vector>

union EndianCheck {
    uint32_t i;
    char c[4];
};

struct Point {

    double x;
    double y;
    double z;
};

struct Triangle {
    float x1, y1, z1;
    float x2, y2, z2;
    float x3, y3, z3;
    float nx, ny, nz; 
};


struct BlockMesh {
    std::vector<double> vertices;
    std::vector<std::vector<int>> blocks;
};