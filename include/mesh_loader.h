#pragma once

#include <fstream>

#define TINYPLY_IMPLEMENTATION
#include "tinyply.h"

#include "utils.h"

// Mesh loader
void readPlyFile(const std::string &mesh_path, std::vector<float3> &vverts, std::vector<uint3> &vfaces, std::vector<float3> &vnormals)
{
    std::cout << "........................................................................\n";
    std::cout << "Reading PLY file: " << mesh_path << std::endl;

    std::unique_ptr<std::istream> file_stream;
    std::vector<uint8_t> byte_buffer;

    manual_timer timer;
    timer.start();

    try
    {
        file_stream.reset(new std::ifstream(mesh_path, std::ios::binary));

        if (!file_stream || file_stream->fail()) throw std::runtime_error("file_stream failed to open " + mesh_path);

        file_stream->seekg(0, std::ios::end);
        file_stream->seekg(0, std::ios::beg);

        tinyply::PlyFile file;
        file.parse_header(*file_stream);

        std::cout << "\t[ply_header] Type: " << (file.is_binary_file() ? "binary" : "ascii") << std::endl;
        for (const auto & c : file.get_comments()) std::cout << "\t[ply_header] Comment: " << c << std::endl;
        for (const auto & c : file.get_info()) std::cout << "\t[ply_header] Info: " << c << std::endl;

        for (const auto & e : file.get_elements())
        {
            std::cout << "\t[ply_header] element: " << e.name << " (" << e.size << ")" << std::endl;
            for (const auto & p : e.properties)
            {
                std::cout << "\t[ply_header] \tproperty: " << p.name << " (type=" << tinyply::PropertyTable[p.propertyType].str << ")";
                if (p.isList) std::cout << " (list_type=" << tinyply::PropertyTable[p.listType].str << ")";
                std::cout << std::endl;
            }
        }

        std::shared_ptr<tinyply::PlyData> pvertices, pfaces, pnormals;

        // The header information can be used to programmatically extract properties on elements
        // known to exist in the header prior to reading the data. For brevity of this sample, properties 
        // like vertex position are hard-coded: 
        try { pvertices = file.request_properties_from_element("vertex", { "x", "y", "z" }); }
        catch (const std::exception & e) { std::cerr << "tinyply exception: " << e.what() << std::endl; }

        try { pfaces = file.request_properties_from_element("face", { "vertex_indices" }, 3); }
        catch (const std::exception & e) { std::cerr << "tinyply exception: " << e.what() << std::endl; }

        try { pnormals = file.request_properties_from_element("vertex", { "nx", "ny", "nz" }); }
        catch (const std::exception & e) { std::cerr << "tinyply exception: " << e.what() << std::endl; }

        file.read(*file_stream);

        if (pvertices)   std::cout << "\tRead " << pvertices->count  << " total vertices "<< std::endl;
        if (pnormals) std::cout << "\tRead " << pnormals->count  << " total normals "<< std::endl;
        if (pfaces)   std::cout << "\tRead " << pfaces->count  << " total faces "<< std::endl;

        const size_t numVerticesBytes = pvertices->buffer.size_bytes();
        vverts.resize(pvertices->count);
        std::memcpy(vverts.data(), pvertices->buffer.get(), numVerticesBytes);
        
        const size_t numFacesBytes = pfaces->buffer.size_bytes();
        vfaces.resize(pfaces->count);
        std::memcpy(vfaces.data(), pfaces->buffer.get(), numFacesBytes);
        
        std::cout << "\n > Number of vertices : " << vverts.size() << std::endl;
        std::cout << "\n > Number of faces : " << vfaces.size() << std::endl;

        if (pnormals)
        {
            if (pvertices->count != pnormals->count)
            {
                std::cerr << "\tSHOULD HAVE AS MANY NORMALS AS VERTICES!!!\n";
                std::cerr << "\tCHECK YOUR PLY FILE...\n";
                exit(-1);
            }

            const size_t numNormalsBytes = pnormals->buffer.size_bytes();
            vnormals.resize(pnormals->count);
            std::memcpy(vnormals.data(), pnormals->buffer.get(), numNormalsBytes);

            std::cout << "\n > Number of normals : " << vnormals.size() << std::endl;
        }

        timer.stop();
        std::cout << "\n >>> PLY file " << mesh_path << " read in " << timer.get() << "ms!\n";
    }
    catch (const std::exception & e)
    {
        std::cerr << "Caught tinyply exception: " << e.what() << std::endl;
    }
}
