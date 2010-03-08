/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redstributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Tim Field */

// urdf_to_collada.cpp

/*
Issues:
  - triangles material hard-coded to "mat0"
*/

#include <dae.h>
#include <dae/daeErrorHandler.h>
#include <dom/domCOLLADA.h>

#include <dae/domAny.h>
#include <dom/domConstants.h>
#include <dom/domTriangles.h>
#include <dae/daeDocument.h>
#include <dom/domTypes.h>
#include <dom/domElements.h>

#include <assimp/assimp.hpp>
#include <assimp/aiScene.h>
#include <assimp/aiPostProcess.h>

#include <resource_retriever/retriever.h>

#include <urdf/model.h>
#include <urdf/pose.h>

#include "STLLoader.h"

using namespace std;

namespace collada_urdf {

class ColladaWriter : public daeErrorHandler
{
public:
    struct SCENE
    {
        domVisual_sceneRef              vscene;
        domKinematics_sceneRef          kscene;
        domPhysics_sceneRef             pscene;
        domInstance_with_extraRef       viscene;
        domInstance_kinematics_sceneRef kiscene;
        domInstance_with_extraRef       piscene;
    };

    struct LINKOUTPUT
    {
        list<int>     listusedlinks;
        daeElementRef plink;
        domNodeRef    pnode;
    };

    urdf::Model* robot_;

    boost::shared_ptr<DAE>            collada_;
    domCOLLADA*                       dom_;
    domCOLLADA::domSceneRef           scene_;

    domLibrary_geometriesRef          geometriesLib_;
    domLibrary_visual_scenesRef       visualScenesLib_;
    domLibrary_kinematics_scenesRef   kinematicsScenesLib_;
    domLibrary_kinematics_modelsRef   kinematicsModelsLib_;
    domLibrary_jointsRef              jointsLib_;
    domLibrary_physics_scenesRef      physicsScenesLib_;
    domLibrary_materialsRef           materialsLib_;
    domLibrary_effectsRef             effectsLib_;
    //domLibrary_articulated_systemsRef articulatedSystemsLib_;

public:
    ColladaWriter(urdf::Model* robot) : robot_(robot) {
        daeErrorHandler::setErrorHandler(this);

        collada_.reset(new DAE());
        collada_->setIOPlugin(NULL);
        collada_->setDatabase(NULL);

        string documentName("mycollada.dae");
        daeDocument* doc = NULL;
        daeInt error = collada_->getDatabase()->insertDocument(documentName.c_str(), &doc); // also creates a collada root
        if (error != DAE_OK || doc == NULL)
        {
            cerr << "Failed to create new document\n";
            throw;
        }

        dom_ = daeSafeCast<domCOLLADA>(doc->getDomRoot());
        dom_->setAttribute("xmlns:math","http://www.w3.org/1998/Math/MathML");

        // Create the required asset tag
        domAssetRef asset = daeSafeCast<domAsset>(dom_->createAndPlace(COLLADA_ELEMENT_ASSET));
        {
            domAsset::domCreatedRef created = daeSafeCast<domAsset::domCreated>(asset->createAndPlace(COLLADA_ELEMENT_CREATED));
            created->setValue("2009-04-06T17:01:00.891550");  // @todo: replace with current date
            domAsset::domModifiedRef modified = daeSafeCast<domAsset::domModified>(asset->createAndPlace(COLLADA_ELEMENT_MODIFIED));
            modified->setValue("2009-04-06T17:01:00.891550"); // @todo: replace with current date

            domAsset::domContributorRef contrib = daeSafeCast<domAsset::domContributor>(asset->createAndPlace(COLLADA_TYPE_CONTRIBUTOR));
            domAsset::domContributor::domAuthoring_toolRef authoringtool = daeSafeCast<domAsset::domContributor::domAuthoring_tool>(contrib->createAndPlace(COLLADA_ELEMENT_AUTHORING_TOOL));
            authoringtool->setValue("URDF Collada Writer");

            domAsset::domUnitRef units = daeSafeCast<domAsset::domUnit>(asset->createAndPlace(COLLADA_ELEMENT_UNIT));
            units->setMeter(1);
            units->setName("meter");

            domAsset::domUp_axisRef zup = daeSafeCast<domAsset::domUp_axis>(asset->createAndPlace(COLLADA_ELEMENT_UP_AXIS));
            zup->setValue(UP_AXIS_Z_UP);
        }

        scene_ = dom_->getScene();
        if (!scene_)
            scene_ = daeSafeCast<domCOLLADA::domScene>(dom_->createAndPlace(COLLADA_ELEMENT_SCENE));

        visualScenesLib_ = daeSafeCast<domLibrary_visual_scenes>(dom_->createAndPlace(COLLADA_ELEMENT_LIBRARY_VISUAL_SCENES));
        visualScenesLib_->setId("vscenes");
        geometriesLib_ = daeSafeCast<domLibrary_geometries>(dom_->createAndPlace(COLLADA_ELEMENT_LIBRARY_GEOMETRIES));
        geometriesLib_->setId("geometries");
        kinematicsScenesLib_ = daeSafeCast<domLibrary_kinematics_scenes>(dom_->createAndPlace(COLLADA_ELEMENT_LIBRARY_KINEMATICS_SCENES));
        kinematicsScenesLib_->setId("kscenes");
        kinematicsModelsLib_ = daeSafeCast<domLibrary_kinematics_models>(dom_->createAndPlace(COLLADA_ELEMENT_LIBRARY_KINEMATICS_MODELS));
        kinematicsModelsLib_->setId("kinematics_models");
        jointsLib_ = daeSafeCast<domLibrary_joints>(dom_->createAndPlace(COLLADA_ELEMENT_LIBRARY_JOINTS));
        jointsLib_->setId("joints");

        physicsScenesLib_ = daeSafeCast<domLibrary_physics_scenes>(dom_->createAndPlace(COLLADA_ELEMENT_LIBRARY_PHYSICS_SCENES));
        physicsScenesLib_->setId("physics_scenes");
        effectsLib_ = daeSafeCast<domLibrary_effects>(dom_->createAndPlace(COLLADA_ELEMENT_LIBRARY_EFFECTS));
        effectsLib_->setId("effects");
        materialsLib_ = daeSafeCast<domLibrary_materials>(dom_->createAndPlace(COLLADA_ELEMENT_LIBRARY_MATERIALS));
        materialsLib_->setId("materials");

        //articulatedSystemsLib_ = daeSafeCast<domLibrary_articulated_systems>(dom_->createAndPlace(COLLADA_ELEMENT_LIBRARY_ARTICULATED_SYSTEMS));
        //articulatedSystemsLib_->setId("articulated_systems");
    }

    virtual ~ColladaWriter() {
        collada_.reset();
        DAE::cleanup();
    }

    SCENE createScene() {
        SCENE s;

        // Create visual scene
        s.vscene = daeSafeCast<domVisual_scene>(visualScenesLib_->createAndPlace(COLLADA_ELEMENT_VISUAL_SCENE));
        s.vscene->setId("vscene");
        s.vscene->setName("URDF Visual Scene");

        // Create instance visual scene
        s.viscene = daeSafeCast<domInstance_with_extra>(scene_->createAndPlace(COLLADA_ELEMENT_INSTANCE_VISUAL_SCENE));
        s.viscene->setUrl((string("#") + string(s.vscene->getID())).c_str());

        // Create kinematics scene
        s.kscene = daeSafeCast<domKinematics_scene>(kinematicsScenesLib_->createAndPlace(COLLADA_ELEMENT_KINEMATICS_SCENE));
        s.kscene->setId("kscene");
        s.kscene->setName("URDF Kinematics Scene");

        // Create instance kinematics scene
        s.kiscene = daeSafeCast<domInstance_kinematics_scene>(scene_->createAndPlace(COLLADA_ELEMENT_INSTANCE_KINEMATICS_SCENE));
        s.kiscene->setUrl((string("#") + string(s.kscene->getID())).c_str());

        // Create physics scene
        s.pscene = daeSafeCast<domPhysics_scene>(physicsScenesLib_->createAndPlace(COLLADA_ELEMENT_PHYSICS_SCENE));
        s.pscene->setId("pscene");
        s.pscene->setName("URDF Physics Scene");

        // Create instance physics scene
        s.piscene = daeSafeCast<domInstance_with_extra>(scene_->createAndPlace(COLLADA_ELEMENT_INSTANCE_PHYSICS_SCENE));
        s.piscene->setUrl((string("#") + string(s.pscene->getID())).c_str());

        return s;
    }

    virtual void handleError(daeString msg) {
        cerr << "COLLADA error: " << msg << "\n";
    }

    virtual void handleWarning(daeString msg) {
        cerr << "COLLADA warning: " << msg << "\n";
    }

    bool writeScene() {
        SCENE scene = createScene();

        setupPhysics(scene);
        addScene(scene);
        addGeometries();

        addJoints();
        addKinematics();

        collada_->writeAll();

        return true;
    }

    void setupPhysics(SCENE scene) {
        // <technique_common>
        domPhysics_scene::domTechnique_commonRef common = daeSafeCast<domPhysics_scene::domTechnique_common>(scene.pscene->createAndPlace(COLLADA_ELEMENT_TECHNIQUE_COMMON));
        {
            // <gravity>0 0 0</gravity>
            domTargetable_float3Ref g = daeSafeCast<domTargetable_float3>(common->createAndPlace(COLLADA_ELEMENT_GRAVITY));
            g->getValue().set3(0.0, 0.0, 0.0);
        }
        // </technique_common>
    }

    void addScene(SCENE scene) {
        boost::shared_ptr<const urdf::Link> urdf_root = robot_->getRoot();

        // <node id="v1" name="pr2">
        domNodeRef root_node = daeSafeCast<domNode>(scene.vscene->createAndPlace(COLLADA_ELEMENT_NODE));
        root_node->setId("v1");
        root_node->setName(urdf_root->name.c_str());

        // <kinematics_model id="k1" name="pr2">
        domKinematics_modelRef kmodel = daeSafeCast<domKinematics_model>(kinematicsModelsLib_->createAndPlace(COLLADA_ELEMENT_KINEMATICS_MODEL));
        kmodel->setId("k1");
        kmodel->setName(urdf_root->name.c_str());

        // <instance_kinematics_model url="#k1" side="inst_k1">
        domInstance_kinematics_modelRef ikm = daeSafeCast<domInstance_kinematics_model>(scene.kscene->createAndPlace(COLLADA_ELEMENT_INSTANCE_KINEMATICS_MODEL));
        string model_id = kmodel->getID();
        string inst_model_sid = string("inst_") + model_id;
        ikm->setUrl((string("#") + model_id).c_str());
        ikm->setSid(inst_model_sid.c_str());
        {            
            // <newparam>
            domKinematics_newparamRef newparam = daeSafeCast<domKinematics_newparam>(ikm->createAndPlace(COLLADA_ELEMENT_NEWPARAM));            
            newparam->setSid("kscene.inst_k1");
            {
                // <SIDREF>kscene/inst_k1</SIDREF>
                daeSafeCast<domKinematics_newparam::domSIDREF>(newparam->createAndPlace(COLLADA_ELEMENT_SIDREF))->setValue("kscene/inst_k1");
            }
            // </newparam>
        }

        /*
        for (map<string, boost::shared_ptr<urdf::Link> >::const_iterator i = robot_->links_.begin(); i != robot_->links_.end(); i++) {
            boost::shared_ptr<urdf::Link> urdf_link = i->second;

            // <node id="v1.node0" name="base_link" side="node0">
            domNodeRef link_node = daeSafeCast<domNode>(scene.vscene->createAndPlace(COLLADA_ELEMENT_NODE));
            root_node->setId("v1");
            root_node->setName(urdf_root->name.c_str());
        }
        */
    }

    void addGeometries() {
        int link_num = 0;

        for (map<string, boost::shared_ptr<urdf::Link> >::const_iterator i = robot_->links_.begin(); i != robot_->links_.end(); i++) {
            boost::shared_ptr<urdf::Link> urdf_link = i->second;

            if (urdf_link->visual == NULL || urdf_link->visual->geometry == NULL)
                continue;

            switch (urdf_link->visual->geometry->type) {
                case urdf::Geometry::MESH: {
                    urdf::Mesh* urdf_mesh = (urdf::Mesh*) urdf_link->visual->geometry.get();

                    string        filename = urdf_mesh->filename;
                    urdf::Vector3 scale    = urdf_mesh->scale;      // @todo use scale

                    // <geometry id="g1.link0.geom0">
                    domGeometryRef geometry = daeSafeCast<domGeometry>(geometriesLib_->createAndPlace(COLLADA_ELEMENT_GEOMETRY));
                    string geometry_id = string("g1.link") + boost::lexical_cast<string>(link_num) + string(".geom0");
                    geometry->setId(geometry_id.c_str());
                    {
                        loadMesh(filename, geometry, geometry_id);
                    }
                    // </geometry>

                    link_num++;
                    break;
                }
                case urdf::Geometry::SPHERE: {
                    cerr << "Geometry type SPHERE of link " << urdf_link->name << " is unsupported" << endl;
                    break;
                }
                case urdf::Geometry::BOX: {
                    cerr << "Geometry type BOX of link " << urdf_link->name << " is unsupported" << endl;
                    break;
                }
                case urdf::Geometry::CYLINDER: {
                    cerr << "Geometry type CYLINDER of link " << urdf_link->name << " is unsupported" << endl;
                    break;
                }
                default: {
                    cerr << "Geometry type " << urdf_link->visual->geometry->type << " of link " << urdf_link->name << " is supported" << endl;
                    break;
                }
            }
        }
    }

    void loadMesh(const string& filename, domGeometryRef geometry, const string& geometry_id) {
    	// Load the mesh
        resource_retriever::MemoryResource resource;
        resource_retriever::Retriever retriever;
        try {
            resource = retriever.get(filename.c_str());
        }
        catch (resource_retriever::Exception& e) {
            cerr << "Unable to load mesh file " << filename << ": " << e.what() << endl;
            return;
        }

        // Try assimp first, then STLLoader
        if (!loadMeshWithAssimp(resource, geometry, geometry_id))
            if (!loadMeshWithSTLLoader(resource, geometry, geometry_id))
                cerr << "*** Can't load " << filename << endl;
    }

    bool loadMeshWithAssimp(const resource_retriever::MemoryResource& resource, domGeometryRef geometry, const string& geometry_id) {
        // Import the mesh using assimp
        Assimp::Importer importer;
        const aiScene* scene = importer.ReadFileFromMemory(resource.data.get(), resource.size, aiProcess_SortByPType /* aiProcess_CalcTangentSpace | aiProcess_Triangulate | aiProcess_JoinIdenticalVertices */);
        if (!scene)
            return false;

        buildMeshFromAssimp(scene, scene->mRootNode, geometry, geometry_id);
        return true;
    }

    bool loadMeshWithSTLLoader(const resource_retriever::MemoryResource& resource, domGeometryRef geometry, const string& geometry_id) {
        // Write the resource to a temporary file
        char tmp_filename[] = "/tmp/collada_urdf_XXXXXX";
        int fd = mkstemp(tmp_filename);
        write(fd, resource.data.get(), resource.size);
        close(fd);

        // Import the mesh using STLLoader
        STLLoader loader;
        Mesh* stl_mesh = loader.load(string(tmp_filename));
        buildMeshFromSTLLoader(stl_mesh, geometry, geometry_id);
        delete stl_mesh;
        
        // Delete the temporary file
        unlink(tmp_filename);

        return true;
    }

    void buildMeshFromSTLLoader(Mesh* stl_mesh, daeElementRef parent, const string& geometry_id) {
        // <mesh>
        domMeshRef mesh = daeSafeCast<domMesh>(parent->createAndPlace(COLLADA_ELEMENT_MESH));
        {
            unsigned int num_vertices = stl_mesh->vertices.size();
            unsigned int num_indices  = stl_mesh->indices.size();
            unsigned int num_faces    = num_indices / 3;
            
            // <source id="g1.link0.geom0.positions">
            domSourceRef positions_source = daeSafeCast<domSource>(mesh->createAndPlace(COLLADA_ELEMENT_SOURCE));
            positions_source->setId((geometry_id + string(".positions")).c_str());
            {
                // <float_array id="g1.link0.geom0.positions-array" count="4533" digits="6">
                domFloat_arrayRef positions_array = daeSafeCast<domFloat_array>(positions_source->createAndPlace(COLLADA_ELEMENT_FLOAT_ARRAY));
                positions_array->setId((geometry_id + string(".positions-array")).c_str());
                positions_array->setCount(num_vertices);
                positions_array->setDigits(6); // 6 decimal places
                positions_array->getValue().setCount(num_vertices);
                
                for (unsigned int j = 0; j < num_vertices; j++) {
                    positions_array->getValue()[j] = stl_mesh->vertices[j].x;
                    positions_array->getValue()[j] = stl_mesh->vertices[j].y;
                    positions_array->getValue()[j] = stl_mesh->vertices[j].z;
                }
                
                // <technique_common>
                domSource::domTechnique_commonRef source_tech = daeSafeCast<domSource::domTechnique_common>(positions_source->createAndPlace(COLLADA_ELEMENT_TECHNIQUE_COMMON));
                
                // <accessor count="4533" source="#g1.link0.geom0.positions-array" stride="3">
                domAccessorRef accessor = daeSafeCast<domAccessor>(source_tech->createAndPlace(COLLADA_ELEMENT_ACCESSOR));
                accessor->setCount(num_vertices);
                accessor->setSource(xsAnyURI(*positions_array, string("#") + geometry_id + string(".positions-array")));
                accessor->setStride(3);
                
                // <param name="X" type="float"/>
                // <param name="Y" type="float"/>
                // <param name="Z" type="float"/>
                domParamRef px = daeSafeCast<domParam>(accessor->createAndPlace(COLLADA_ELEMENT_PARAM)); px->setName("X"); px->setType("float");
                domParamRef py = daeSafeCast<domParam>(accessor->createAndPlace(COLLADA_ELEMENT_PARAM)); py->setName("Y"); py->setType("float");
                domParamRef pz = daeSafeCast<domParam>(accessor->createAndPlace(COLLADA_ELEMENT_PARAM)); pz->setName("Z"); pz->setType("float");
            }
            
            // <vertices id="vertices">
            domVerticesRef vertices = daeSafeCast<domVertices>(mesh->createAndPlace(COLLADA_ELEMENT_VERTICES));
            vertices->setId("vertices");
            {
                // <input semantic="POSITION" source="#g1.link0.geom0.positions"/>
                domInput_localRef vertices_input = daeSafeCast<domInput_local>(vertices->createAndPlace(COLLADA_ELEMENT_INPUT));
                vertices_input->setSemantic("POSITION");
                vertices_input->setSource(domUrifragment(*positions_source, string("#") + geometry_id + string(".positions")));
            }
            // </vertices>
            
            // <triangles count="1511" material="mat0">
            domTrianglesRef triangles = daeSafeCast<domTriangles>(mesh->createAndPlace(COLLADA_ELEMENT_TRIANGLES));
            {
                triangles->setCount(num_faces);
                triangles->setMaterial("mat0");
                
                // <input offset="0" semantic="VERTEX" source="#g1.link0.geom0/vertices" set="0"/>
                domInput_local_offsetRef vertex_offset = daeSafeCast<domInput_local_offset>(triangles->createAndPlace(COLLADA_ELEMENT_INPUT));
                vertex_offset->setSemantic("VERTEX");
                vertex_offset->setOffset(0);
                vertex_offset->setSource(domUrifragment(*positions_source, string("#") + geometry_id + string("/vertices")));
                
                // <p>0 1 2 3 ...
                domPRef indices = daeSafeCast<domP>(triangles->createAndPlace(COLLADA_ELEMENT_P));
                indices->getValue().setCount(num_indices);
                for (unsigned int i = 0; i < num_indices; i++)
                    indices->getValue()[i] = stl_mesh->indices[i];
                // </p>
            }
            // </triangles>
        }
        // </mesh>
    }

    // EXPERIMENTAL: untested
    void buildMeshFromAssimp(const aiScene* scene, aiNode* node, daeElementRef parent, const string& geometry_id) {
        if (node == NULL)
            return;

        aiMatrix4x4 transform = node->mTransformation;

        aiNode* pnode = node->mParent;
        while (pnode) {
            // Don't convert to y-up orientation, which is what the root node in Assimp does
            if (pnode->mParent != NULL)
                transform = pnode->mTransformation * transform;
            pnode = pnode->mParent;
        }

        // <mesh>
        domMeshRef mesh = daeSafeCast<domMesh>(parent->createAndPlace(COLLADA_ELEMENT_MESH));
        {
            for (unsigned int i = 0; i < node->mNumMeshes; i++) {
                aiMesh* aMesh = scene->mMeshes[i];
                
                // Add in the indices for each face
                for (unsigned int j = 0; j < aMesh->mNumFaces; j++) {
                    aiFace* aFace = &(aMesh->mFaces[j]);
                    for (unsigned int k = 0; k < aFace->mNumIndices; k++) {
                        //int index = aFace->mIndices[k];
                        // @todo add index
                        //subMesh->AddIndex(aFace->mIndices[k]);
                    }
                }
                
                domSourceRef positions_source = daeSafeCast<domSource>(mesh->createAndPlace(COLLADA_ELEMENT_SOURCE));
                {
                    positions_source->setId((geometry_id + string(".positions")).c_str());
                    
                    domFloat_arrayRef positions_array = daeSafeCast<domFloat_array>(positions_source->createAndPlace(COLLADA_ELEMENT_FLOAT_ARRAY));
                    positions_array->setId((geometry_id + string(".positions-array")).c_str());
                    positions_array->setCount(aMesh->mNumVertices);
                    positions_array->setDigits(6); // 6 decimal places
                    positions_array->getValue().setCount(3 * aMesh->mNumVertices);
                    
                    for (unsigned int j = 0; j < aMesh->mNumVertices; j++) {
                        aiVector3D p;
                        p.x = aMesh->mVertices[j].x;
                        p.y = aMesh->mVertices[j].y;
                        p.z = aMesh->mVertices[j].z;
                        
                        p *= transform;
                        
                        positions_array->getValue()[j] = p.x;
                        positions_array->getValue()[j] = p.y;
                        positions_array->getValue()[j] = p.z;
                        
                        /*
                          if (aMesh->HasNormals()) {
                          p.x = aMesh->mNormals[j].x;
                          p.y = aMesh->mNormals[j].y;
                          p.z = aMesh->mNormals[j].z;
                          }
                          
                          // @todo add normal
                          //subMesh->AddNormal(p.x, p.y, p.z);
                          
                          // @todo add tex coord
                          //if (aMesh->mNumUVComponents[0])
                          //    subMesh->AddTexCoord(aMesh->mTextureCoords[0][j].x, 1.0 -aMesh->mTextureCoords[0][j].y);
                          //else
                          //    subMesh->AddTexCoord(0,0);
                          */
                    }
                }
                
                domTrianglesRef triangles = daeSafeCast<domTriangles>(mesh->createAndPlace(COLLADA_ELEMENT_TRIANGLES));
                {
                    triangles->setCount(aMesh->mNumFaces / 3);
                }
            }

            for (unsigned int i = 0; i < node->mNumChildren; i++)
                buildMeshFromAssimp(scene, node->mChildren[i], mesh, geometry_id);
        }
    }

    void addJoints() {
        for (map<string, boost::shared_ptr<urdf::Joint> >::const_iterator i = robot_->joints_.begin(); i != robot_->joints_.end(); i++) {
            boost::shared_ptr<urdf::Joint> urdf_joint = i->second;

            // Create COLLADA joint
            domJointRef joint = daeSafeCast<domJoint>(jointsLib_->createAndPlace(COLLADA_ELEMENT_JOINT));

            joint->setId(urdf_joint->name.c_str());
            joint->setName(urdf_joint->name.c_str());

            switch (urdf_joint->type)
            {
                case urdf::Joint::REVOLUTE: {
                    // joint.axis
                    vector<domAxis_constraintRef> axes(1);
                    axes[0] = daeSafeCast<domAxis_constraint>(joint->createAndPlace(COLLADA_ELEMENT_REVOLUTE));
                    axes[0]->setSid("axis0");
                    domAxisRef axis = daeSafeCast<domAxis>(axes[0]->createAndPlace(COLLADA_ELEMENT_AXIS));
                    axis->getValue().setCount(3);
                    axis->getValue()[0] = urdf_joint->axis.x;
                    axis->getValue()[1] = urdf_joint->axis.y;
                    axis->getValue()[2] = urdf_joint->axis.z;

                    // joint.limits
                    domJoint_limitsRef limits = daeSafeCast<domJoint_limits>(axes[0]->createAndPlace(COLLADA_TYPE_LIMITS));
                    daeSafeCast<domMinmax>(limits->createAndPlace(COLLADA_ELEMENT_MIN))->getValue() = urdf_joint->limits->lower;
                    daeSafeCast<domMinmax>(limits->createAndPlace(COLLADA_ELEMENT_MAX))->getValue() = urdf_joint->limits->upper;
                    break;
                }
                case urdf::Joint::CONTINUOUS: {
                    // Model as a REVOLUTE joint without limits

                    // joint.axis
                    vector<domAxis_constraintRef> axes(1);
                    axes[0] = daeSafeCast<domAxis_constraint>(joint->createAndPlace(COLLADA_ELEMENT_REVOLUTE));
                    axes[0]->setSid("axis0");
                    domAxisRef axis = daeSafeCast<domAxis>(axes[0]->createAndPlace(COLLADA_ELEMENT_AXIS));
                    axis->getValue().setCount(3);
                    axis->getValue()[0] = urdf_joint->axis.x;
                    axis->getValue()[1] = urdf_joint->axis.y;
                    axis->getValue()[2] = urdf_joint->axis.z;
                    break;
                }
                case urdf::Joint::PRISMATIC: {
                    // joint.axis
                    vector<domAxis_constraintRef> axes(1);
                    axes[0] = daeSafeCast<domAxis_constraint>(joint->createAndPlace(COLLADA_ELEMENT_PRISMATIC));
                    axes[0]->setSid("axis0");
                    domAxisRef axis = daeSafeCast<domAxis>(axes[0]->createAndPlace(COLLADA_ELEMENT_AXIS));
                    axis->getValue().setCount(3);
                    axis->getValue()[0] = urdf_joint->axis.x;
                    axis->getValue()[1] = urdf_joint->axis.y;
                    axis->getValue()[2] = urdf_joint->axis.z;

                    // joint.limits
                    domJoint_limitsRef limits = daeSafeCast<domJoint_limits>(axes[0]->createAndPlace(COLLADA_TYPE_LIMITS));
                    daeSafeCast<domMinmax>(limits->createAndPlace(COLLADA_ELEMENT_MIN))->getValue() = urdf_joint->limits->lower;
                    daeSafeCast<domMinmax>(limits->createAndPlace(COLLADA_ELEMENT_MAX))->getValue() = urdf_joint->limits->upper;
                    break;
                }
                case urdf::Joint::FIXED: {
                    // Model as a REVOLUTE joint with no leeway

                    // joint.axis
                    vector<domAxis_constraintRef> axes(1);
                    axes[0] = daeSafeCast<domAxis_constraint>(joint->createAndPlace(COLLADA_ELEMENT_REVOLUTE));
                    axes[0]->setSid("axis0");
                    domAxisRef axis = daeSafeCast<domAxis>(axes[0]->createAndPlace(COLLADA_ELEMENT_AXIS));
                    axis->getValue().setCount(3);
                    axis->getValue()[0] = urdf_joint->axis.x;
                    axis->getValue()[1] = urdf_joint->axis.y;
                    axis->getValue()[2] = urdf_joint->axis.z;

                    // joint.limits
                    domJoint_limitsRef limits = daeSafeCast<domJoint_limits>(axes[0]->createAndPlace(COLLADA_TYPE_LIMITS));
                    daeSafeCast<domMinmax>(limits->createAndPlace(COLLADA_ELEMENT_MIN))->getValue() = 0.0;
                    daeSafeCast<domMinmax>(limits->createAndPlace(COLLADA_ELEMENT_MAX))->getValue() = 0.0;
                    break;
                }
                case urdf::Joint::UNKNOWN: {
                    cerr << "Joint type UNKNOWN of joint " << urdf_joint->name << " is unsupported" << endl;
                    break;
                }
                case urdf::Joint::FLOATING: {
                    cerr << "Joint type FLOATING of joint " << urdf_joint->name << " is unsupported" << endl;
                    break;
                }
                case urdf::Joint::PLANAR: {
                    cerr << "Joint type PLANAR of joint " << urdf_joint->name << " is unsupported" << endl;
                    break;
                }
                default: {
                    cerr << "Joint type " << urdf_joint->type << " of joint " << urdf_joint->name << " is unsupported" << endl;
                    break;
                }
            }
        }
    }

    void addKinematics()
    {
        // <kinematics_model id="k1" name="pr2">
        domKinematics_modelRef model = daeSafeCast<domKinematics_model>(kinematicsModelsLib_->createAndPlace(COLLADA_ELEMENT_KINEMATICS_MODEL));
        model->setId("k1");
        {
            // <technique_common>
            domKinematics_model_techniqueRef technique = daeSafeCast<domKinematics_model_technique>(model->createAndPlace(COLLADA_ELEMENT_TECHNIQUE_COMMON));
            for (map<string, boost::shared_ptr<urdf::Link> >::const_iterator i = robot_->links_.begin(); i != robot_->links_.end(); i++) {
                boost::shared_ptr<urdf::Link> urdf_link = i->second;
                
                // <instance_joint>
                domInstance_jointRef instance_joint = daeSafeCast<domInstance_joint>(technique->createAndPlace(COLLADA_ELEMENT_INSTANCE_JOINT));
                instance_joint->setUrl("#joint_1");
            }
            // </technique_common>

            addLink(robot_->getRoot(), technique);
        }
    }

    void addLink(boost::shared_ptr<const urdf::Link> urdf_link, daeElementRef parent) {
        // <link name="base_link">
        domLinkRef link = daeSafeCast<domLink>(parent->createAndPlace(COLLADA_ELEMENT_LINK));
        link->setName(urdf_link->name.c_str());
        // @todo link->setSid
        for (vector<boost::shared_ptr<urdf::Joint> >::const_iterator i = urdf_link->child_joints.begin(); i != urdf_link->child_joints.end(); i++) {
            boost::shared_ptr<urdf::Joint> urdf_joint = *i;

            // <attachemnt_full joint="k1/joint0">
            domLink::domAttachment_fullRef attachment_full = daeSafeCast<domLink::domAttachment_full>(link->createAndPlace(COLLADA_TYPE_ATTACHMENT_FULL));
            attachment_full->setJoint(urdf_joint->name.c_str());
            {
                addTransformation(attachment_full, urdf_joint->parent_to_joint_origin_transform);
                addLink(robot_->getLink(urdf_joint->child_link_name), attachment_full);
            }
            // </attachment_full>
        }
        // </link>
    }

    void addTransformation(daeElementRef parent, const urdf::Pose& pose)
    {
        // <translate>x y z</translate>
        domTranslateRef trans = daeSafeCast<domTranslate>(parent->createAndPlace(COLLADA_ELEMENT_TRANSLATE));
        trans->getValue().setCount(3);
        trans->getValue()[0] = pose.position.x;
        trans->getValue()[1] = pose.position.y;
        trans->getValue()[2] = pose.position.z;

        // <rotate>x y z w</rotate>
        domRotateRef rot = daeSafeCast<domRotate>(parent->createAndPlace(COLLADA_ELEMENT_ROTATE));
        rot->getValue().setCount(4);
        rot->getValue()[0] = pose.rotation.x;
        rot->getValue()[1] = pose.rotation.y;
        rot->getValue()[2] = pose.rotation.z;
        rot->getValue()[3] = pose.rotation.w;
    }
};

}

int main(int argc, char** argv)
{
    if (argc != 2) {
        std::cerr << "Usage: urdf_to_collada input.urdf" << std::endl;
        return -1;
    }

    TiXmlDocument robot_model_xml;
    robot_model_xml.LoadFile(argv[1]);
    TiXmlElement* robot_xml = robot_model_xml.FirstChildElement("robot");
    if (!robot_xml) {
        std::cerr << "ERROR: Could not load the xml into TiXmlElement" << std::endl;
        return -1;
    }

    urdf::Model robot;
    if (!robot.initXml(robot_xml)) {
        std::cerr << "ERROR: Model Parsing the xml failed" << std::endl;
        return -1;
    }

    collada_urdf::ColladaWriter* writer = new collada_urdf::ColladaWriter(&robot);
    writer->writeScene();
    delete writer;

    return 0;
}
