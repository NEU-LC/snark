// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2017 The University of Sydney
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
// 3. Neither the name of the University of Sydney nor the
//    names of its contributors may be used to endorse or promote products
//    derived from this software without specific prior written permission.
//
// NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE
// GRANTED BY THIS LICENSE.  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT
// HOLDERS AND CONTRIBUTORS \"AS IS\" AND ANY EXPRESS OR IMPLIED
// WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
// MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
// BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
// OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
// IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

/// @author Navid Pirmarzdashti

#pragma once

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <iostream>

namespace snark { namespace graphics { namespace qopengl {

   
/// import model files using ASSIMP library
/// creates and adds meshes to a mesh_shader
/// the mesh_shader then does the rendering, so all paint related code should be in mesh_shader and mesh classes
/// options: 
///     hold an array of shared ptr to meshes in case they need updating -> this is better for future animation
///     clear shader mesh array and add new ones every time we load model
class model : public mesh
{
public:
    model() : scene(NULL) { }
    /// load model from file
    /// returns true for success, or false if fails to import
//     bool load(const std::string& file_name,mesh_shader* shader)
    bool import(const std::string& file_name)
    {
//         aiSetImportPropertyInteger(props,AI_CONFIG_PP_SBP_REMOVE, aiPrimitiveType_LINE | aiPrimitiveType_POINT);
//         ReadFileEx(... prop)
        
//         aiProcess_PreTransformVertices | aiProcess_GenUVCoords | aiProcess_TransformUVCoords 
        scene = importer.ReadFile(file_name, aiProcess_Triangulate | aiProcess_GenNormals | aiProcess_JoinIdenticalVertices | aiProcess_SortByPType);
//             aiProcess_CalcTangentSpace       |
//             aiProcess_Triangulate            |
//             aiProcess_JoinIdenticalVertices  |
//             aiProcess_SortByPType);
        if(!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE)
        {
            std::cerr<<"failed to import model file: "<<file_name<<std::endl;
            return false;
        }
        debug();
        return true;
    }
    void debug()
    {
        std::cerr<<"import msh "<<scene->mNumMeshes<<", mt "<<scene->mNumMaterials<<", tx "<<scene->mNumTextures<<", cam "<<scene->mNumCameras<<std::endl;
        if(scene->HasMeshes())
        {
            aiMesh* mesh=scene->mMeshes[0];
            std::cerr<<"first mesh v "<<mesh->mNumVertices<<", f "<<mesh->mNumFaces<<", mt "<<mesh->mMaterialIndex<<", uv[0] "<<mesh->mNumUVComponents[0]<<std::endl;
            std::cerr<<"colors ";
            for(unsigned i=0;i<AI_MAX_NUMBER_OF_COLOR_SETS;i++)
                std::cerr<<mesh->mColors[i]<<", ";
            std::cerr<<std::endl;
            if(scene->HasMaterials())
            {
                aiMaterial* mat=scene->mMaterials[mesh->mMaterialIndex];
                std::cerr<<"material "<<mat->mNumProperties<<std::endl;
                for(unsigned i=0;i<mat->mNumProperties;i++)
                {
                    aiMaterialProperty* prop=mat->mProperties[i];
                    std::cerr<<"prop "<<prop->mKey.C_Str()<<" "<<prop->mDataLength<<" ";
                    if(prop->mType==aiPTI_String)
                    {
                        aiString str;
                        mat->Get(prop->mKey.C_Str(),0,0,str);
                        std::cerr<<str.C_Str();
                    }
                    else
                    {
                        for(unsigned j=0;j<prop->mDataLength/4;j++)
                        {
                            switch(prop->mType)
                            {
                                case aiPTI_Float:
                                    if(!j) std::cerr<<"float ";
                                    std::cerr<<((float*)prop->mData)[j]<<" ";
                                    break;
                                case aiPTI_Integer:
                                    if(!j) std::cerr<<"int ";
                                    std::cerr<<((int*)prop->mData)[j]<<" ";
                                    break;
                            }
                        }
                    }
                    std::cerr<<std::endl;
                }
            }
        }
    }
    std::vector<int> vertex;
protected:
    Assimp::Importer importer;
    const aiScene* scene;
};
    
} } } // namespace snark { namespace graphics { namespace qopengl {
    
