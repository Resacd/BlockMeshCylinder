#pragma once
#ifndef _BLOCK_MESH_H_
#define _BLOCK_MESH_H
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include "Utility.h"
#include <cmath>
#include <limits>
#include "structs.h"
#include <iostream>


#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/marching_cubes_hoppe.h>
#include <pcl/surface/marching_cubes.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include <pcl/io/ply_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>


void GenerateBlockMeshPoissonSamplingDisk(std::vector<Triangle>& triangles, const std::string& filename) {
    // Convert the triangles to a PCL point cloud
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointNormal>);
    for (const auto& t : triangles) {
        pcl::PointNormal p1, p2, p3;
        p1.x = t.x1; p1.y = t.y1; p1.z = t.z1;
        p2.x = t.x2; p2.y = t.y2; p2.z = t.z2;
        p3.x = t.x3; p3.y = t.y3; p3.z = t.z3;

        // Calculate the normal for the triangle
        Eigen::Vector3f v1(t.x2 - t.x1, t.y2 - t.y1, t.z2 - t.z1);
        Eigen::Vector3f v2(t.x3 - t.x1, t.y3 - t.y1, t.z3 - t.z1);
        Eigen::Vector3f normal = v1.cross(v2).normalized();

        p1.normal_x = p2.normal_x = p3.normal_x = normal.x();
        p1.normal_y = p2.normal_y = p3.normal_y = normal.y();
        p1.normal_z = p2.normal_z = p3.normal_z = normal.z();

        cloud->push_back(p1);
        cloud->push_back(p2);
        cloud->push_back(p3);
    }

    std::cout << "Number of points in cloud: " << cloud->size() << std::endl;

    // Perform surface reconstruction using GreedyProjectionTriangulation
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    pcl::PolygonMesh mesh;
    gp3.setInputCloud(cloud);
    gp3.setSearchRadius(20);
    gp3.setMu(2.5);
    gp3.setMaximumNearestNeighbors(100);
    gp3.setMaximumSurfaceAngle(M_PI / 4);
    gp3.setMinimumAngle(M_PI / 18);
    gp3.setMaximumAngle(2 * M_PI / 3);
    gp3.setNormalConsistency(false);
    gp3.reconstruct(mesh);

    // Convert the mesh to a PCL point cloud
    pcl::PointCloud<pcl::PointNormal>::Ptr meshCloud(new pcl::PointCloud<pcl::PointNormal>);
    pcl::fromPCLPointCloud2(mesh.cloud, *meshCloud);

    // Create a search tree for the mesh
    pcl::search::KdTree<pcl::PointNormal>::Ptr meshTree(new pcl::search::KdTree<pcl::PointNormal>);
    meshTree->setInputCloud(meshCloud);

    // Define the bounding box of the block mesh
    pcl::PointXYZ minPt(0, -1, -1);
    pcl::PointXYZ maxPt(2, 1, 1);

    // Generate the block mesh using a "Poisson Disk Sampling" algorithm
    pcl::Poisson<pcl::PointNormal> pds;
    pds.setInputCloud(meshCloud);
    pds.setSearchMethod(meshTree);
    pds.setPointWeight(4);
    pds.setSamplesPerNode(100);
    pds.setScale(1.25);
    pds.setSolverDivide(8);
    pds.setIsoDivide(8);
    pcl::PolygonMesh outputMesh;
    pds.reconstruct(outputMesh);
    // Convert the output mesh to triangles
    pcl::fromPCLPointCloud2(outputMesh.cloud, *cloud);
    triangles.clear();
    for (size_t i = 0; i < outputMesh.polygons.size(); ++i) {
        const auto& p = outputMesh.polygons[i];
        const auto& v1 = cloud->points[p.vertices[0]];
        const auto& v2 = cloud->points[p.vertices[1]];
        const auto& v3 = cloud->points[p.vertices[2]];
        triangles.emplace_back(v1.x, v1.y, v1.z, v2.x, v2.y, v2.z, v3.x, v3.y, v3.z);
    }

    // Save the generated mesh to file
    pcl::io::savePLYFileBinary(filename, outputMesh);

    std::cout << "Generated block mesh has " << triangles.size() << " triangles." << std::endl;
}


void GenerateBlockMeshMarchingCube(std::vector<Triangle>& triangles, const std::string& filename) {
    // Convert the triangles to a PCL point cloud
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointNormal>);
    for (const auto& t : triangles) {
        cloud->push_back(pcl::PointNormal(t.x1, t.y1, t.z1, 0, 0, 0));
        cloud->push_back(pcl::PointNormal(t.x2, t.y2, t.z2, 0, 0, 0));
        cloud->push_back(pcl::PointNormal(t.x3, t.y3, t.z3, 0, 0, 0));
    }

    std::cout << "Number of points in cloud: " << cloud->size() << std::endl;

    // Calculate normals for the input point cloud
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointNormal>);
    tree->setInputCloud(cloud);
    pcl::NormalEstimation<pcl::PointNormal, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(20);
    ne.compute(*normals);

    std::cout << "Number of normals: " << normals->size() << std::endl;

    // Concatenate the XYZ and normal fields
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*cloud, *normals, *cloud_normals);

    // Perform Poisson surface reconstruction
    pcl::Poisson<pcl::PointNormal> poisson;
    poisson.setInputCloud(cloud_normals);
    poisson.setDepth(8);
    poisson.setSolverDivide(8);
    pcl::PolygonMesh mesh;
    poisson.reconstruct(mesh);

    // Create a search tree for the mesh
    pcl::search::KdTree<pcl::PointNormal>::Ptr meshTree(new pcl::search::KdTree<pcl::PointNormal>);
    pcl::fromPCLPointCloud2(mesh.cloud, *cloud);
    meshTree->setInputCloud(cloud);

    // Define the bounding box of the block mesh
    pcl::PointXYZ minPt(-1, -1, -1);
    pcl::PointXYZ maxPt(1, 1, 1);

    // Add an offset to the point cloud to move the surface further away
    float offSurfaceDisplacement = 0.1;
    for (int i = 0; i < cloud->size(); i++) {
        pcl::PointNormal& pt = cloud->at(i);
        pt.x += offSurfaceDisplacement;
        pt.y += offSurfaceDisplacement;
        pt.z += offSurfaceDisplacement;
    }

    // Generate the block mesh using marching cubes
    pcl::PolygonMesh blockMesh;
    pcl::MarchingCubesHoppe<pcl::PointNormal> mc;
    mc.setInputCloud(cloud);
    mc.setSearchMethod(meshTree);
    mc.setGridResolution(100, 100, 100);
    mc.setIsoLevel(0.5);
    mc.reconstruct(blockMesh);

    // Save the block mesh to a VTK file
    pcl::io::saveVTKFile(filename, blockMesh);
}


BlockMesh generate_block_mesh(std::vector<Triangle>& triangles, double blockSize) {
    BlockMesh blockmesh;

    double x = 0;
    double y = 0;
    double z = 0;

    float maxX = std::numeric_limits<double>::lowest();
    float maxY = std::numeric_limits<double>::lowest();
    float maxZ = std::numeric_limits<double>::lowest();
    float minX = std::numeric_limits<double>::max();
    float minY = std::numeric_limits<double>::max();
    float minZ = std::numeric_limits<double>::max();

    for (int i = 0; i < triangles.size(); i++) {
        maxX = std::max({ maxX, triangles[i].x1, triangles[i].x2, triangles[i].x3 });
        maxY = std::max({ maxY, triangles[i].y1, triangles[i].y2, triangles[i].y3 });
        maxZ = std::max({ maxZ, triangles[i].z1, triangles[i].z2, triangles[i].z3 });
        minX = std::min({ minX, triangles[i].x1, triangles[i].x2, triangles[i].x3 });
        minY = std::min({ minY, triangles[i].y1, triangles[i].y2, triangles[i].y3 });
        minZ = std::min({ minZ, triangles[i].z1, triangles[i].z2, triangles[i].z3 });
    }
    int numBlocksX = std::ceil((maxX - minX) / blockSize);
    int numBlocksY = std::ceil((maxY - minY) / blockSize);
    int numBlocksZ = std::ceil((maxZ - minZ) / blockSize);


    for (int k = 0; k < numBlocksZ; k++) {
        for (int j = 0; j < numBlocksY; j++) {
            for (int i = 0; i < numBlocksX; i++) {

                x += i * blockSize;
                y += j * blockSize;
                z += k * blockSize;
                Point Point{ x,y,z };
                if (isPointInsidePolyhedron(Point, triangles)) {

                    blockmesh.vertices.push_back(x);
                    blockmesh.vertices.push_back(y);
                    blockmesh.vertices.push_back(z);
                }

            }
        }
    }

    // Generate blocks
    int numVerticesX = numBlocksX + 1;
    int numVerticesY = numBlocksY + 1;
    int numVerticesZ = numBlocksZ + 1;
    for (int k = 0; k < numBlocksZ; k++) {
        for (int j = 0; j < numBlocksY; j++) {
            for (int i = 0; i < numBlocksX; i++) {
                int baseVertexIndex = k * numVerticesY * numVerticesX + j * numVerticesX + i;
                std::vector<int> block;
                block.push_back(baseVertexIndex);
                block.push_back(baseVertexIndex + numVerticesX);
                block.push_back(baseVertexIndex + numVerticesY * numVerticesX + numVerticesX);
                block.push_back(baseVertexIndex + numVerticesY * numVerticesX);
                block.push_back(baseVertexIndex + 1);
                block.push_back(baseVertexIndex + numVerticesX + 1);
                block.push_back(baseVertexIndex + numVerticesY * numVerticesX + numVerticesX + 1);
                block.push_back(baseVertexIndex + numVerticesY * numVerticesX + 1);
                blockmesh.blocks.push_back(block);
            }
        }
    }

    return blockmesh;
}


//void GenerateBlockMeshPoissonSamplingDisk(std::vector<Triangle>& triangles, const std::string& filename) {
//    // Convert the triangles to a PCL point cloud
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//    for (const auto& t : triangles) {
//        cloud->push_back(pcl::PointXYZ(t.x1, t.y1, t.z1));
//        cloud->push_back(pcl::PointXYZ(t.x2, t.y2, t.z2));
//        cloud->push_back(pcl::PointXYZ(t.x3, t.y3, t.z3));
//    }
//
//    std::cout << "Number of points in cloud: " << cloud->size() << std::endl;
//
//    // Calculate normals for the input point cloud
//
//    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
//    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
//    tree->setInputCloud(cloud);
//    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
//    ne.setInputCloud(cloud);
//    ne.setSearchMethod(tree);
//    ne.setRadiusSearch(20);
//    ne.compute(*normals);
//
//    std::cout << "Number of normals: " << normals->size() << std::endl;
//
//    // Concatenate the XYZ and normal fields
//    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals(new pcl::PointCloud<pcl::PointNormal>);
//    pcl::concatenateFields(*cloud, *normals, *cloud_normals);
//
//    // Perform surface reconstruction using GreedyProjectionTriangulation
//    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
//    pcl::PolygonMesh mesh;
//    gp3.setInputCloud(cloud_normals);
//    gp3.setSearchRadius(20);
//    gp3.setMu(2.5);
//    gp3.setMaximumNearestNeighbors(100);
//    gp3.setMaximumSurfaceAngle(M_PI / 4);
//    gp3.setMinimumAngle(M_PI / 18);
//    gp3.setMaximumAngle(2 * M_PI / 3);
//    gp3.setNormalConsistency(false);
//    gp3.reconstruct(mesh);
//
//    // Convert the mesh to a PCL point cloud
//    pcl::PointCloud<pcl::PointXYZ>::Ptr meshCloud(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::fromPCLPointCloud2(mesh.cloud, *meshCloud);
//
//    // Create a search tree for the mesh
//    pcl::search::KdTree<pcl::PointXYZ>::Ptr meshTree(new pcl::search::KdTree<pcl::PointXYZ>);
//    meshTree->setInputCloud(meshCloud);
//
//    // Define the bounding box of the block mesh
//    pcl::PointXYZ minPt(-1, -1, -1);
//    pcl::PointXYZ maxPt(1, 1, 1);
//
//    // Generate the block mesh using a "Poisson Disk Sampling" algorithm
//    pcl::Poisson<pcl::PointXYZ> pds;
//    pds.setInputCloud(meshCloud);
//    pds.setSearchMethod(meshTree);
//    pds.setPointWeight(4);
//    pds.setSamplesPerNode(100);
//    pds.setScale(1.25);
//    pds.setSolverDivide(8);
//    pds.setIsoDivide(8);
//    pds.reconstruct(mesh);
//    
//    // Save the block mesh to a file
//    pcl::io::savePLYFile(filename, mesh);
//}


#endif