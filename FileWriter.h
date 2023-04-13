#pragma once
#ifndef  _FILE_WRITER_H_
#define  _FILE_WRITER_H_
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include "structs.h"
#include <vtk-9.2/vtkSmartPointer.h>
#include <vtk-9.2/vtkUnstructuredGrid.h>
#include <vtk-9.2/vtkPoints.h>
#include <vtk-9.2/vtkCellArray.h>
#include <vtk-9.2/vtkXMLUnstructuredGridWriter.h>

void write_block_mesh_vtk(const BlockMesh& bm, const std::string& filename)
{

    vtkSmartPointer<vtkUnstructuredGrid> ug = vtkSmartPointer<vtkUnstructuredGrid>::New();
    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    for (int i = 0; i < bm.vertices.size(); i += 3) {
        points->InsertNextPoint(&bm.vertices[i]);
    }
    ug->SetPoints(points);
    vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New();
    for (auto& block : bm.blocks) {
        vtkIdType cell[8] = { block[0], block[1], block[2], block[3], block[4], block[5], block[6], block[7] };
        cells->InsertNextCell(8, cell);
    }
    ug->SetCells(VTK_HEXAHEDRON, cells);
    vtkSmartPointer<vtkXMLUnstructuredGridWriter> writer = vtkSmartPointer<vtkXMLUnstructuredGridWriter>::New();
    writer->SetFileName(filename.c_str());
    writer->SetInputData(ug);
    writer->Write();
}

void openfoam_writer(const std::string& filename, const BlockMesh& bm) {
    std::ofstream outfile(filename);
    if (!outfile.is_open()) {
        std::cerr << "Error: Unable to open file " << filename << " for writing" << std::endl;
        return;
    }

    outfile << "FoamFile\n{\n    version     2.0;\n    format      ascii;\n    class       dictionary;\n    object      blockMeshDict;\n}\n\n";

    outfile << "convertToMeters 1.0;\n\n";

    outfile << "vertices\n(\n";
    for (size_t i = 0; i < bm.vertices.size(); i += 3) {
        outfile << "    (" << bm.vertices[i] << " " << bm.vertices[i + 1] << " " << bm.vertices[i + 2] << ")\n";
    }
    outfile << ");\n\n";

    outfile << "blocks\n(\n";
    for (size_t i = 0; i < bm.blocks.size(); i++) {
        outfile << "    hex (" << bm.blocks[i][0] << " " << bm.blocks[i][1] << " " << bm.blocks[i][2] << " " << bm.blocks[i][3]
            << " " << bm.blocks[i][4] << " " << bm.blocks[i][5] << " " << bm.blocks[i][6] << " " << bm.blocks[i][7] << ")\n";
    }
    outfile << ");\n\n";

    outfile << "edges\n(\n);\n\n";
    outfile << "boundary\n(\n);\n\n";

    outfile.close();
    std::cout << "Block mesh written to file " << filename << std::endl;
}


void write_vtk_legacy(const std::string& filename, const BlockMesh& bm) {
    std::ofstream outfile(filename);
    if (!outfile.is_open()) {
        std::cerr << "Error: Unable to open file " << filename << " for writing" << std::endl;
        return;
    }

    // Write VTK header
    outfile << "# vtk DataFile Version 2.0\n";
    outfile << "Block Mesh\n";
    outfile << "ASCII\n";
    outfile << "DATASET UNSTRUCTURED_GRID\n";

    // Write vertices
    outfile << "POINTS " << bm.vertices.size() / 3 << " float\n";
    for (size_t i = 0; i < bm.vertices.size(); i += 3) {
        outfile << bm.vertices[i] << " " << bm.vertices[i + 1] << " " << bm.vertices[i + 2] << "\n";
    }

    // Write hexahedral cells
    outfile << "CELLS " << bm.blocks.size() << " " << 9 * bm.blocks.size() << "\n";
    for (size_t i = 0; i < bm.blocks.size(); i++) {
        outfile << "8 ";
        for (size_t j = 0; j < 8; j++) {
            outfile << bm.blocks[i][j] << " ";
        }
        outfile << "\n";
    }

    // Write cell types
    outfile << "CELL_TYPES " << bm.blocks.size() << "\n";
    for (size_t i = 0; i < bm.blocks.size(); i++) {
        outfile << "12\n";  // VTK_HEXAHEDRON
    }

    outfile.close();
    std::cout << "Block mesh written to file " << filename << std::endl;
}

#endif // ! _FILE_WRITER_
