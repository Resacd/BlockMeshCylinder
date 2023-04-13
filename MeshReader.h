#pragma once
#ifndef _MESH_READER_H_
#define _MESH_READER_H_
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include "structs.h"
#include <iomanip>


std::vector<std::vector<double>> readCoordinatesFromOFF(std::string filename) {
    std::vector<std::vector<double>> coordinates;
    std::ifstream inputFile(filename);
    if (inputFile) {
        std::string line;
        int numVertices, numFaces, numEdges;
        getline(inputFile, line);
        if (line != "OFF") {
            std::cout << "Error: Invalid OFF file format." << std::endl;
            return coordinates;
        }
        getline(inputFile, line);
        std::stringstream ss(line);
        ss >> numVertices >> numFaces >> numEdges;
        for (int i = 0; i < numVertices; i++) {
            getline(inputFile, line);
            std::stringstream ss(line);
            double x, y, z;
            ss >> x >> y >> z;
            std::vector<double> row{ x, y, z };
            coordinates.push_back(row);
        }
        inputFile.close();
    }
    else {
        std::cout << "Error opening file " << filename << std::endl;
    }
    return coordinates;
}

std::vector<Triangle> readSTLASCII(const std::string& filename) {
    std::vector<Triangle> triangles;
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: cannot open file " << filename << std::endl;
        return triangles;
    }

    std::string line;
    std::stringstream ss;
    std::string facet, normal, outer, loop, vertex, endloop, endfacet;
    float x, y, z;
    Triangle triangle;
    while (getline(file, line)) {
        ss.clear();
        ss.str(line);
        ss >> facet >> normal >> triangle.nx >> triangle.ny >> triangle.nz;
        if (ss.fail() || facet != "facet" || normal != "normal") {
            continue;
        }

        getline(file, line);
        ss.clear();
        ss.str(line);
        ss >> outer >> loop;
        if (ss.fail() || outer != "outer" || loop != "loop") {
            continue;
        }

        for (int i = 0; i < 3; ++i) {
            getline(file, line);
            ss.clear();
            ss.str(line);
            ss >> vertex >> x >> y >> z;
            if (ss.fail() || vertex != "vertex") {
                break;
            }
            switch (i) {
            case 0:
                triangle.x1 = x;
                triangle.y1 = y;
                triangle.z1 = z;
                break;
            case 1:
                triangle.x2 = x;
                triangle.y2 = y;
                triangle.z2 = z;
                break;
            case 2:
                triangle.x3 = x;
                triangle.y3 = y;
                triangle.z3 = z;
                break;
            }
        }

        getline(file, line);
        ss.clear();
        ss.str(line);
        ss >> endloop;
        if (ss.fail() || endloop != "endloop") {
            continue;
        }

        getline(file, line);
        ss.clear();
        ss.str(line);
        ss >> endfacet;
        if (ss.fail() || endfacet != "endfacet") {
            continue;
        }

        triangles.push_back(triangle);
    }

    file.close();
    return triangles;
}

std::vector<Triangle> readSTLBinary(std::string filename) {
    std::vector<Triangle> triangles;

    std::ifstream file(filename, std::ios::in | std::ios::binary);
    if (!file) {
        throw std::runtime_error("Failed to open file: " + filename);
    }

    char header[80];
    file.read(header, 80);

    uint32_t num_triangles;
    file.read(reinterpret_cast<char*>(&num_triangles), sizeof(num_triangles));

    for (uint32_t i = 0; i < num_triangles; i++) {
        float nx, ny, nz;
        file.read(reinterpret_cast<char*>(&nx), sizeof(float));
        file.read(reinterpret_cast<char*>(&ny), sizeof(float));
        file.read(reinterpret_cast<char*>(&nz), sizeof(float));
        float norm = std::sqrt(nx * nx + ny * ny + nz * nz);
        nx /= norm;
        ny /= norm;
        nz /= norm;
        float v1x, v1y, v1z, v2x, v2y, v2z, v3x, v3y, v3z;
        file.read(reinterpret_cast<char*>(&v1x), sizeof(float));
        file.read(reinterpret_cast<char*>(&v1y), sizeof(float));
        file.read(reinterpret_cast<char*>(&v1z), sizeof(float));
        file.read(reinterpret_cast<char*>(&v2x), sizeof(float));
        file.read(reinterpret_cast<char*>(&v2y), sizeof(float));
        file.read(reinterpret_cast<char*>(&v2z), sizeof(float));
        file.read(reinterpret_cast<char*>(&v3x), sizeof(float));
        file.read(reinterpret_cast<char*>(&v3y), sizeof(float));
        file.read(reinterpret_cast<char*>(&v3z), sizeof(float));


        uint16_t attr_byte_count;
        file.read(reinterpret_cast<char*>(&attr_byte_count), sizeof(attr_byte_count));

        // Create Triangle object
        Triangle t = { v1x, v1y, v1z, v2x, v2y, v2z, v3x, v3y, v3z,nx,ny,nz };
        triangles.push_back(t);
    }

    file.close();
    int i = 0;
    for (const auto& t : triangles) {

        std::cout << "Triangle Number =" << i << std::endl;
        std::cout << t.x1 << ", " << t.y1 << ", " << t.z1 << std::endl;
        std::cout << t.x2 << ", " << t.y2 << ", " << t.z2 << std::endl;
        std::cout << t.x3 << ", " << t.y3 << ", " << t.z3 << std::endl;
        std::cout << t.nx << ", " << t.ny << ", " << t.nz << std::endl;
        std::cout << std::endl;
        i++;

    }
    return triangles;
}

#endif // !_MESH_READER_H_
