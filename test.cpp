#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "structs.h"
#include "MeshReader.h"
#include "BlockMesh.h"

int main(int argc, char** argv)
{

    EndianCheck ec;
    ec.i = 0x01020304;
    if (ec.c[0] == 1 && ec.c[1] == 2 && ec.c[2] == 3 && ec.c[3] == 4) {
        std::cout << "Big-endian" << std::endl;
    }
    else if (ec.c[0] == 4 && ec.c[1] == 3 && ec.c[2] == 2 && ec.c[3] == 1) {
        std::cout << "Little-endian" << std::endl;
    }
    else {
        std::cout << "Unknown byte order" << std::endl;
    }

    std::string filename = "CylinderASCII.stl";
    std::vector<Triangle> Triangles = readSTLASCII(filename);
    std::cout << "PCL version: " << PCL_VERSION << std::endl;

    //GenerateBlockMeshMarchingCube(Triangles, "PCLBM");
    GenerateBlockMeshPoissonSamplingDisk(Triangles, "PCLBM");



    return (0);
}