/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2010, Willow Garage, Inc.
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

#include "collada_urdf/ColladaWriter.h"

#include "collada_urdf/STLLoader.h"

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/posix_time_io.hpp>
#include <boost/foreach.hpp>

#define foreach BOOST_FOREACH

using std::string;
using std::map;
using std::vector;
using boost::shared_ptr;

namespace collada_urdf {

ColladaWriter::ColladaWriter(std::string const& filename)
    : source_(filename), dae_(NULL), dom_(NULL)
{
    TiXmlDocument xml;
    if (!xml.LoadFile(filename.c_str()))
        throw ColladaWriterException("Error reading XML file");

    TiXmlElement* robot_xml = xml.FirstChildElement("robot");
    if (!robot_xml)
        throw ColladaWriterException("Error parsing URDF model from XML (robot element not found)");

    robot_ = shared_ptr<urdf::Model>(new urdf::Model);
    if (!robot_->initXml(robot_xml))
        throw ColladaWriterException("Error parsing URDF model from XML");
}

ColladaWriter::ColladaWriter(shared_ptr<urdf::Model> robot, string const& source)
    : robot_(robot), source_(source), dae_(NULL), dom_(NULL)
{
}

void ColladaWriter::writeDocument(string const& documentName) {
    initDocument(documentName);

    SCENE scene = createScene();

    setupPhysics(scene);
    addGeometries();
    addKinematics(scene);
    addVisuals(scene);
    addMaterials();
    addBindings(scene);

    collada_->writeAll();
}

ColladaWriter::~ColladaWriter() {
    collada_.reset();

    DAE::cleanup();
}

// Implementation

void ColladaWriter::handleError(daeString msg) {
    std::cerr << "COLLADA error: " << msg << std::endl;
}

void ColladaWriter::handleWarning(daeString msg) {
    std::cerr << "COLLADA warning: " << msg << std::endl;
}

void ColladaWriter::initDocument(string const& documentName) {
    daeErrorHandler::setErrorHandler(this);

    dae_ = new DAE();

    collada_.reset(dae_);
    collada_->setIOPlugin(NULL);
    collada_->setDatabase(NULL);

    daeDocument* doc = NULL;
    daeInt error = collada_->getDatabase()->insertDocument(documentName.c_str(), &doc); // also creates a collada root
    if (error != DAE_OK || doc == NULL)
        throw ColladaWriterException("Failed to create document");

    dom_ = daeSafeCast<domCOLLADA>(doc->getDomRoot());
    dom_->setAttribute("xmlns:math", "http://www.w3.org/1998/Math/MathML");

    // Create the required asset tag
    domAssetRef asset = daeSafeCast<domAsset>(dom_->createAndPlace(COLLADA_ELEMENT_ASSET));
    {
        string date = getTimeStampString();

        domAsset::domCreatedRef created = daeSafeCast<domAsset::domCreated>(asset->createAndPlace(COLLADA_ELEMENT_CREATED));
        created->setValue(date.c_str());
        domAsset::domModifiedRef modified = daeSafeCast<domAsset::domModified>(asset->createAndPlace(COLLADA_ELEMENT_MODIFIED));
        modified->setValue(date.c_str());

        domAsset::domContributorRef contrib = daeSafeCast<domAsset::domContributor>(asset->createAndPlace(COLLADA_TYPE_CONTRIBUTOR));

        domAsset::domContributor::domAuthoring_toolRef authoringtool = daeSafeCast<domAsset::domContributor::domAuthoring_tool>(contrib->createAndPlace(COLLADA_ELEMENT_AUTHORING_TOOL));
        authoringtool->setValue("URDF Collada Writer");

        domAsset::domContributor::domSource_dataRef sourcedata = daeSafeCast<domAsset::domContributor::domSource_data>(contrib->createAndPlace(COLLADA_ELEMENT_SOURCE_DATA));
        sourcedata->setValue(source_.c_str());

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
    kinematicsModelsLib_->setId("kmodels");
    jointsLib_ = daeSafeCast<domLibrary_joints>(dom_->createAndPlace(COLLADA_ELEMENT_LIBRARY_JOINTS));
    jointsLib_->setId("joints");

    physicsScenesLib_ = daeSafeCast<domLibrary_physics_scenes>(dom_->createAndPlace(COLLADA_ELEMENT_LIBRARY_PHYSICS_SCENES));
    physicsScenesLib_->setId("physics_scenes");
    effectsLib_ = daeSafeCast<domLibrary_effects>(dom_->createAndPlace(COLLADA_ELEMENT_LIBRARY_EFFECTS));
    effectsLib_->setId("effects");
    materialsLib_ = daeSafeCast<domLibrary_materials>(dom_->createAndPlace(COLLADA_ELEMENT_LIBRARY_MATERIALS));
    materialsLib_->setId("materials");
}

ColladaWriter::SCENE ColladaWriter::createScene() {
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

void ColladaWriter::setupPhysics(SCENE const& scene) {
    // <technique_common>
    domPhysics_scene::domTechnique_commonRef common = daeSafeCast<domPhysics_scene::domTechnique_common>(scene.pscene->createAndPlace(COLLADA_ELEMENT_TECHNIQUE_COMMON));
    {
        // <gravity>0 0 0
        domTargetable_float3Ref g = daeSafeCast<domTargetable_float3>(common->createAndPlace(COLLADA_ELEMENT_GRAVITY));
        g->getValue().set3(0.0, 0.0, 0.0);
        // </gravity>
    }
    // </technique_common>
}

void ColladaWriter::addGeometries() {
    int link_num = 0;

    for (map<string, shared_ptr<urdf::Link> >::const_iterator i = robot_->links_.begin(); i != robot_->links_.end(); i++) {
        shared_ptr<urdf::Link> urdf_link = i->second;

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
                geometry_ids_[urdf_link->name] = geometry_id;
                // </geometry>

                link_num++;
                break;
            }
            case urdf::Geometry::SPHERE: {
                std::cerr << "Warning: geometry type SPHERE of link " << urdf_link->name << " not exported" << std::endl;
                break;
            }
            case urdf::Geometry::BOX: {
                std::cerr << "Warning: geometry type BOX of link " << urdf_link->name << " not exported" << std::endl;
                break;
            }
            case urdf::Geometry::CYLINDER: {
                std::cerr << "Warning: geometry type CYLINDER of link " << urdf_link->name << " not exported" << std::endl;
                break;
            }
            default: {
                std::cerr << "Warning: geometry type " << urdf_link->visual->geometry->type << " of link " << urdf_link->name << " not exported" << std::endl;
                break;
            }
        }
    }
}

void ColladaWriter::loadMesh(string const& filename, domGeometryRef geometry, string const& geometry_id) {
    // Load the mesh
    resource_retriever::MemoryResource resource;
    resource_retriever::Retriever retriever;
    try {
        resource = retriever.get(filename.c_str());
    }
    catch (resource_retriever::Exception& e) {
        std::cerr << "Unable to load mesh file " << filename << ": " << e.what() << std::endl;
        return;
    }

    // Try assimp first, then STLLoader
    try {
        loadMeshWithSTLLoader(resource, geometry, geometry_id);
    }
    catch (ColladaWriterException e) {
        std::cerr << "Unable to load mesh file " << filename << ": " << e.what() << std::endl;
    }
}

bool ColladaWriter::loadMeshWithSTLLoader(resource_retriever::MemoryResource const& resource, domGeometryRef geometry, string const& geometry_id) {
    // Write the resource to a temporary file
    char tmp_filename[] = "/tmp/collada_urdf_XXXXXX";
    int fd = mkstemp(tmp_filename);
    if (fd == -1)
        throw ColladaWriterException("Couldn't create temporary file");

    if (write(fd, resource.data.get(), resource.size) != resource.size) {
        close(fd);
        unlink(tmp_filename);
        throw ColladaWriterException("Couldn't write resource to file");
    }
    close(fd);

    // Import the mesh using STLLoader
    STLLoader loader;
    shared_ptr<Mesh> stl_mesh = loader.load(string(tmp_filename));
    if (stl_mesh == shared_ptr<Mesh>()) {
        unlink(tmp_filename);
        throw ColladaWriterException("Couldn't import mesh with STLLoader");
    }

    // Build the COLLADA mesh
    buildMeshFromSTLLoader(stl_mesh, geometry, geometry_id);

    // Delete the temporary file
    unlink(tmp_filename);

    return true;
}

void ColladaWriter::buildMeshFromSTLLoader(shared_ptr<Mesh> stl_mesh, daeElementRef parent, string const& geometry_id) {
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
            positions_array->setCount(num_vertices * 3);
            positions_array->setDigits(6); // 6 decimal places
            positions_array->getValue().setCount(num_vertices * 3);
            for (unsigned int j = 0; j < num_vertices; j++) {
                positions_array->getValue()[j * 3    ] = stl_mesh->vertices[j].x;
                positions_array->getValue()[j * 3 + 1] = stl_mesh->vertices[j].y;
                positions_array->getValue()[j * 3 + 2] = stl_mesh->vertices[j].z;
            }
            // </float_array>

            // <technique_common>
            domSource::domTechnique_commonRef source_tech = daeSafeCast<domSource::domTechnique_common>(positions_source->createAndPlace(COLLADA_ELEMENT_TECHNIQUE_COMMON));
            {
                // <accessor count="4533" source="#g1.link0.geom0.positions-array" stride="3">
                domAccessorRef accessor = daeSafeCast<domAccessor>(source_tech->createAndPlace(COLLADA_ELEMENT_ACCESSOR));
                accessor->setCount(num_vertices / 3);
                accessor->setSource(xsAnyURI(*positions_array, string("#") + geometry_id + string(".positions-array")));
                accessor->setStride(3);
                {                
                    // <param name="X" type="float"/>
                    // <param name="Y" type="float"/>
                    // <param name="Z" type="float"/>
                    domParamRef px = daeSafeCast<domParam>(accessor->createAndPlace(COLLADA_ELEMENT_PARAM)); px->setName("X"); px->setType("float");
                    domParamRef py = daeSafeCast<domParam>(accessor->createAndPlace(COLLADA_ELEMENT_PARAM)); py->setName("Y"); py->setType("float");
                    domParamRef pz = daeSafeCast<domParam>(accessor->createAndPlace(COLLADA_ELEMENT_PARAM)); pz->setName("Z"); pz->setType("float");
                }
                // </accessor>
            }
            // </technique_common>
        }

        // <vertices id="vertices">
        domVerticesRef vertices = daeSafeCast<domVertices>(mesh->createAndPlace(COLLADA_ELEMENT_VERTICES));
        string vertices_id = geometry_id + string(".vertices");
        vertices->setId(vertices_id.c_str());
        {
            // <input semantic="POSITION" source="#g1.link0.geom0.positions"/>
            domInput_localRef vertices_input = daeSafeCast<domInput_local>(vertices->createAndPlace(COLLADA_ELEMENT_INPUT));
            vertices_input->setSemantic("POSITION");
            vertices_input->setSource(domUrifragment(*positions_source, string("#") + string(positions_source->getId())));
        }
        // </vertices>

        // <triangles count="1511" material="mat0">
        domTrianglesRef triangles = daeSafeCast<domTriangles>(mesh->createAndPlace(COLLADA_ELEMENT_TRIANGLES));
        triangles->setCount(num_faces);
        triangles->setMaterial("mat0");
        {
            // <input offset="0" semantic="VERTEX" source="#g1.link0.geom0/vertices" set="0"/>
            domInput_local_offsetRef vertex_offset = daeSafeCast<domInput_local_offset>(triangles->createAndPlace(COLLADA_ELEMENT_INPUT));
            vertex_offset->setSemantic("VERTEX");
            vertex_offset->setOffset(0);
            vertex_offset->setSource(domUrifragment(*positions_source, string("#") + vertices_id));
            {
                // <p>0 1 2 3 ...
                domPRef indices = daeSafeCast<domP>(triangles->createAndPlace(COLLADA_ELEMENT_P));
                indices->getValue().setCount(num_indices);
                for (unsigned int i = 0; i < num_indices; i++)
                    indices->getValue()[i] = stl_mesh->indices[i];
                // </p>
            }
        }
        // </triangles>
    }
    // </mesh>
}

void ColladaWriter::addJoints(daeElementRef parent) {
    int joint_num = 0;
    for (map<string, shared_ptr<urdf::Joint> >::const_iterator i = robot_->joints_.begin(); i != robot_->joints_.end(); i++) {
        shared_ptr<urdf::Joint> urdf_joint = i->second;

        // <joint name="base_laser_joint" sid="joint0">
        domJointRef joint = daeSafeCast<domJoint>(parent->createAndPlace(COLLADA_ELEMENT_JOINT));
        string joint_sid = string("joint") + boost::lexical_cast<string>(joint_num);
        joint_num++;
        joint->setName(urdf_joint->name.c_str());
        joint->setSid(joint_sid.c_str());
        joint_sids_[urdf_joint->name] = joint_sid;

        double axis_x = urdf_joint->axis.x;
        double axis_y = urdf_joint->axis.y;
        double axis_z = urdf_joint->axis.z;
        if (axis_x == 0.0 && axis_y == 0.0 && axis_z == 0.0) {
            axis_x = 1.0;
            axis_y = 0.0;
            axis_z = 0.0;
        }

        // @hack: OpenRAVE appears to flip joint axes
        axis_x *= -1.0;
        axis_y *= -1.0;
        axis_z *= -1.0;

        switch (urdf_joint->type)
        {
            case urdf::Joint::REVOLUTE: {
                // <revolute sid="axis0">
                domAxis_constraintRef revolute = daeSafeCast<domAxis_constraint>(joint->createAndPlace(COLLADA_ELEMENT_REVOLUTE));
                revolute->setSid("axis0");
                {
                    // <axis>
                    domAxisRef axis = daeSafeCast<domAxis>(revolute->createAndPlace(COLLADA_ELEMENT_AXIS));
                    {
                        axis->getValue().setCount(3);
                        axis->getValue()[0] = axis_x;
                        axis->getValue()[1] = axis_y;
                        axis->getValue()[2] = axis_z;
                    }
                    // </axis>

                    // <limits>
                    domJoint_limitsRef limits = daeSafeCast<domJoint_limits>(revolute->createAndPlace(COLLADA_TYPE_LIMITS));
                    {
                        daeSafeCast<domMinmax>(limits->createAndPlace(COLLADA_ELEMENT_MIN))->getValue() = angles::to_degrees(urdf_joint->limits->lower);
                        daeSafeCast<domMinmax>(limits->createAndPlace(COLLADA_ELEMENT_MAX))->getValue() = angles::to_degrees(urdf_joint->limits->upper);
                    }
                    // </limits>
                }
                // </revolute>
                break;
            }
            case urdf::Joint::CONTINUOUS: {
                // Model as a REVOLUTE joint without limits

                // <revolute sid="axis0">
                domAxis_constraintRef revolute = daeSafeCast<domAxis_constraint>(joint->createAndPlace(COLLADA_ELEMENT_REVOLUTE));
                revolute->setSid("axis0");
                {
                    // <axis>
                    domAxisRef axis = daeSafeCast<domAxis>(revolute->createAndPlace(COLLADA_ELEMENT_AXIS));
                    {
                        axis->getValue().setCount(3);
                        axis->getValue()[0] = axis_x;
                        axis->getValue()[1] = axis_y;
                        axis->getValue()[2] = axis_z;
                    }
                    // </axis>
                }
                // </revolute>
                break;
            }
            case urdf::Joint::PRISMATIC: {
                // <prismatic sid="axis0">
                domAxis_constraintRef prismatic = daeSafeCast<domAxis_constraint>(joint->createAndPlace(COLLADA_ELEMENT_PRISMATIC));
                prismatic->setSid("axis0");
                {
                    // <axis>
                    domAxisRef axis = daeSafeCast<domAxis>(prismatic->createAndPlace(COLLADA_ELEMENT_AXIS));
                    {
                        axis->getValue().setCount(3);
                        axis->getValue()[0] = axis_x;
                        axis->getValue()[1] = axis_y;
                        axis->getValue()[2] = axis_z;
                    }
                    // </axis>

                    // <limits>
                    domJoint_limitsRef limits = daeSafeCast<domJoint_limits>(prismatic->createAndPlace(COLLADA_TYPE_LIMITS));
                    {
                        daeSafeCast<domMinmax>(limits->createAndPlace(COLLADA_ELEMENT_MIN))->getValue() = urdf_joint->limits->lower;
                        daeSafeCast<domMinmax>(limits->createAndPlace(COLLADA_ELEMENT_MAX))->getValue() = urdf_joint->limits->upper;
                    }
                    // </limits>
                }
                // </prismatic>
                break;
            }
            case urdf::Joint::FIXED: {
                // Model as a REVOLUTE joint with no leeway

                domAxis_constraintRef revolute = daeSafeCast<domAxis_constraint>(joint->createAndPlace(COLLADA_ELEMENT_REVOLUTE));
                revolute->setSid("axis0");
                {
                    // <axis>
                    domAxisRef axis = daeSafeCast<domAxis>(revolute->createAndPlace(COLLADA_ELEMENT_AXIS));
                    {
                        axis->getValue().setCount(3);
                        axis->getValue()[0] = axis_x;
                        axis->getValue()[1] = axis_y;
                        axis->getValue()[2] = axis_z;
                    }
                    // </axis>

                    // <limits>
                    domJoint_limitsRef limits = daeSafeCast<domJoint_limits>(revolute->createAndPlace(COLLADA_TYPE_LIMITS));
                    {
                        daeSafeCast<domMinmax>(limits->createAndPlace(COLLADA_ELEMENT_MIN))->getValue() = 0.0;
                        daeSafeCast<domMinmax>(limits->createAndPlace(COLLADA_ELEMENT_MAX))->getValue() = 0.0;
                    }
                    // </limits>
                }
                // </revolute>
                break;
            }
            case urdf::Joint::UNKNOWN: {
                std::cerr << "Joint type UNKNOWN of joint " << urdf_joint->name << " is unsupported" << std::endl;
                break;
            }
            case urdf::Joint::FLOATING: {
                std::cerr << "Joint type FLOATING of joint " << urdf_joint->name << " is unsupported" << std::endl;
                break;
            }
            case urdf::Joint::PLANAR: {
                std::cerr << "Joint type PLANAR of joint " << urdf_joint->name << " is unsupported" << std::endl;
                break;
            }
            default: {
                std::cerr << "Joint type " << urdf_joint->type << " of joint " << urdf_joint->name << " is unsupported" << std::endl;
                break;
            }
        }
    }
}

void ColladaWriter::addBindings(SCENE const& scene) {
    string model_id = kmodel_->getID();
    string inst_model_sid = string("inst_") + model_id;

    // <bind_kinematics_scene>
    //   <bind_kinematics_model node="node0">
    domBind_kinematics_modelRef kmodel_bind = daeSafeCast<domBind_kinematics_model>(scene.kiscene->createAndPlace(COLLADA_ELEMENT_BIND_KINEMATICS_MODEL));
    kmodel_bind->setNode("v1.node0");  // @todo
    daeSafeCast<domCommon_param>(kmodel_bind->createAndPlace(COLLADA_ELEMENT_PARAM))->setValue(string(string(scene.kscene->getID()) + string(".") + inst_model_sid).c_str());

    for (map<string, shared_ptr<urdf::Joint> >::const_iterator i = robot_->joints_.begin(); i != robot_->joints_.end(); i++) {
        shared_ptr<urdf::Joint> urdf_joint = i->second;

        int    idof                 = 0;   // @todo assuming 1 dof joints
        string joint_sid            = joint_sids_[urdf_joint->name];
        string axis_name            = string("axis") + boost::lexical_cast<string>(idof);
        string joint_axis_sid       = string("kscene.inst_") + model_id + string(".") + joint_sid + string(".") + axis_name;
        string joint_axis_value_sid = joint_axis_sid + string("_value");

        // <bind_joint_axis target="node0/joint_1_axis0">
        domBind_joint_axisRef joint_bind = daeSafeCast<domBind_joint_axis>(scene.kiscene->createAndPlace(COLLADA_ELEMENT_BIND_JOINT_AXIS));
        string node_name = node_ids_[urdf_joint->name];
        joint_bind->setTarget((node_name + string("/node_") + joint_sid + string("_") + axis_name).c_str());
        {
            // <axis>
            domCommon_sidref_or_paramRef axis_bind = daeSafeCast<domCommon_sidref_or_param>(joint_bind->createAndPlace(COLLADA_ELEMENT_AXIS));
            {
                daeSafeCast<domCommon_param>(axis_bind->createAndPlace(COLLADA_TYPE_PARAM))->setValue(joint_axis_sid.c_str());
            }
            // </axis>
            // <value>
            domCommon_float_or_paramRef value_bind = daeSafeCast<domCommon_float_or_param>(joint_bind->createAndPlace(COLLADA_ELEMENT_VALUE));
            {
                daeSafeCast<domCommon_param>(value_bind->createAndPlace(COLLADA_TYPE_PARAM))->setValue(joint_axis_value_sid.c_str());
            }
        }
        // </bind_joint_axis>
    }
}

void ColladaWriter::addKinematics(SCENE const& scene) {
    // <kinematics_model id="k1" name="pr2">
    domKinematics_modelRef kmodel = daeSafeCast<domKinematics_model>(kinematicsModelsLib_->createAndPlace(COLLADA_ELEMENT_KINEMATICS_MODEL));
    kmodel->setId("k1");
    kmodel->setName(robot_->getName().c_str());
    {
        // <technique_common>
        domKinematics_model_techniqueRef technique = daeSafeCast<domKinematics_model_technique>(kmodel->createAndPlace(COLLADA_ELEMENT_TECHNIQUE_COMMON));
        addJoints(technique);
        // </technique_common>

        // <link ...>
        int link_num = 0;
        addKinematicLink(robot_->getRoot(), technique, link_num);
        // </link>
    }
    kmodel_ = kmodel;
    // </kinematics_model>

    string model_id = kmodel->getID();
    string inst_model_sid = string("inst_") + model_id;

    // <instance_kinematics_model url="#k1" sid="inst_k1">
    domInstance_kinematics_modelRef ikm = daeSafeCast<domInstance_kinematics_model>(scene.kscene->createAndPlace(COLLADA_ELEMENT_INSTANCE_KINEMATICS_MODEL));
    ikm->setUrl((string("#") + model_id).c_str());
    ikm->setSid(inst_model_sid.c_str());
    {
        // <newparam sid="kscene.inst_k1">
        domKinematics_newparamRef newparam_model = daeSafeCast<domKinematics_newparam>(ikm->createAndPlace(COLLADA_ELEMENT_NEWPARAM));
        string newparam_model_sid = string("kscene.inst_") + model_id;
        newparam_model->setSid(newparam_model_sid.c_str());
        {
            // <SIDREF>kscene/inst_k1</SIDREF>
            string model_sidref = string("kscene/inst_") + model_id;
            daeSafeCast<domKinematics_newparam::domSIDREF>(newparam_model->createAndPlace(COLLADA_ELEMENT_SIDREF))->setValue(model_sidref.c_str());
        }
        // </newparam>

        for (map<string, shared_ptr<urdf::Joint> >::const_iterator i = robot_->joints_.begin(); i != robot_->joints_.end(); i++) {
            shared_ptr<urdf::Joint> urdf_joint = i->second;

            int idof = 0;   // @todo assuming 1 dof joints

            string joint_sid = joint_sids_[urdf_joint->name];

            string axis_name = string("axis") + boost::lexical_cast<string>(idof);

            // <newparam sid="kscene.inst_k1.joint0.axis0">
            domKinematics_newparamRef newparam = daeSafeCast<domKinematics_newparam>(ikm->createAndPlace(COLLADA_ELEMENT_NEWPARAM));
            string newparam_sid = string("kscene.inst_") + model_id + string(".") + joint_sid + string(".") + axis_name;
            newparam->setSid(newparam_sid.c_str());
            {
                // <SIDREF>kscene/inst_k1/joint0/axis0</SIDREF>
                string sidref = string("kscene/inst_") + model_id + string("/") + joint_sid + string("/") + axis_name;
                daeSafeCast<domKinematics_newparam::domSIDREF>(newparam->createAndPlace(COLLADA_ELEMENT_SIDREF))->setValue(sidref.c_str());
            }
            // </newparam>

            // <newparam sid="kscene.inst_k1.joint0.axis0_value">
            domKinematics_newparamRef newparam_value = daeSafeCast<domKinematics_newparam>(ikm->createAndPlace(COLLADA_ELEMENT_NEWPARAM));
            string newparam_value_sid = string("kscene.inst_") + model_id + string(".") + joint_sid + string(".") + axis_name + string("_value");
            newparam_value->setSid(newparam_value_sid.c_str());
            {
                // <float>0</float>
                daeSafeCast<domKinematics_newparam::domFloat>(newparam_value->createAndPlace(COLLADA_ELEMENT_FLOAT))->setValue(0.0f);
            }
            // </newparam>
        }
    }
    // </instance_kinematics_model>
}

void ColladaWriter::addKinematicLink(shared_ptr<const urdf::Link> urdf_link, daeElementRef parent, int& link_num) {
    // <link sid="link0" name="base_link">
    domLinkRef link = daeSafeCast<domLink>(parent->createAndPlace(COLLADA_ELEMENT_LINK));
    string link_sid = string("link") + boost::lexical_cast<string>(link_num);
    link->setName(urdf_link->name.c_str());
    link->setSid(link_sid.c_str());
    link_num++;
    foreach(shared_ptr<urdf::Joint> urdf_joint, urdf_link->child_joints) {
        // <attachment_full joint="k1/joint0">
        domLink::domAttachment_fullRef attachment_full = daeSafeCast<domLink::domAttachment_full>(link->createAndPlace(COLLADA_TYPE_ATTACHMENT_FULL));
        string attachment_joint = string("k1/") + joint_sids_[urdf_joint->name];
        attachment_full->setJoint(attachment_joint.c_str());
        {
            addRotate(attachment_full, urdf_joint->parent_to_joint_origin_transform.rotation);
            addTranslate(attachment_full, urdf_joint->parent_to_joint_origin_transform.position);
            addKinematicLink(robot_->getLink(urdf_joint->child_link_name), attachment_full, link_num);
        }
        // </attachment_full>
    }
    // </link>
}

void ColladaWriter::addVisuals(SCENE const& scene) {
    // <node id="v1" name="pr2">
    domNodeRef root_node = daeSafeCast<domNode>(scene.vscene->createAndPlace(COLLADA_ELEMENT_NODE));
    root_node->setId("v1");
    root_node->setName(robot_->getName().c_str());
    {
        int link_num = 0;
        addVisualLink(robot_->getRoot(), root_node, link_num);
    }
}

void ColladaWriter::addMaterials() {
    urdf::Color ambient, diffuse;
    ambient.init("1 1 1 0");
    diffuse.init("1 1 1 0");

    for (map<string, shared_ptr<urdf::Link> >::const_iterator i = robot_->links_.begin(); i != robot_->links_.end(); i++) {
        shared_ptr<urdf::Link> urdf_link = i->second;

        map<string, string>::const_iterator j = geometry_ids_.find(urdf_link->name);
        if (j == geometry_ids_.end())
            continue;

        string geometry_id = j->second;

        domEffectRef effect = addEffect(geometry_id, ambient, diffuse);

        // <material id="g1.link0.geom0.eff">
        domMaterialRef material = daeSafeCast<domMaterial>(materialsLib_->createAndPlace(COLLADA_ELEMENT_MATERIAL));
        string material_id = geometry_id + string(".mat");
        material->setId(material_id.c_str());
        {
            // <instance_effect url="#g1.link0.geom0.eff"/>
            domInstance_effectRef instance_effect = daeSafeCast<domInstance_effect>(material->createAndPlace(COLLADA_ELEMENT_INSTANCE_EFFECT));
            string effect_id(effect->getId());
            instance_effect->setUrl((string("#") + effect_id).c_str());
        }
        // </material>
    }
}

domEffectRef ColladaWriter::addEffect(string const& geometry_id, urdf::Color const& color_ambient, urdf::Color const& color_diffuse)
{
    // <effect id="g1.link0.geom0.eff">
    domEffectRef effect = daeSafeCast<domEffect>(effectsLib_->createAndPlace(COLLADA_ELEMENT_EFFECT));
    string effect_id = geometry_id + string(".eff");
    effect->setId(effect_id.c_str());
    {
        // <profile_COMMON>
        domProfile_commonRef profile = daeSafeCast<domProfile_common>(effect->createAndPlace(COLLADA_ELEMENT_PROFILE_COMMON));
        {
            // <technique sid="">
            domProfile_common::domTechniqueRef technique = daeSafeCast<domProfile_common::domTechnique>(profile->createAndPlace(COLLADA_ELEMENT_TECHNIQUE));
            {
                // <phong>
                domProfile_common::domTechnique::domPhongRef phong = daeSafeCast<domProfile_common::domTechnique::domPhong>(technique->createAndPlace(COLLADA_ELEMENT_PHONG));
                {
                    // <ambient>
                    domFx_common_color_or_textureRef ambient = daeSafeCast<domFx_common_color_or_texture>(phong->createAndPlace(COLLADA_ELEMENT_AMBIENT));
                    {
                        // <color>r g b a
                        domFx_common_color_or_texture::domColorRef ambient_color = daeSafeCast<domFx_common_color_or_texture::domColor>(ambient->createAndPlace(COLLADA_ELEMENT_COLOR));
                        ambient_color->getValue().setCount(4);
                        ambient_color->getValue()[0] = color_ambient.r;
                        ambient_color->getValue()[1] = color_ambient.g;
                        ambient_color->getValue()[2] = color_ambient.b;
                        ambient_color->getValue()[3] = color_ambient.a;
                        // </color>
                    }
                    // </ambient>

                    // <diffuse>
                    domFx_common_color_or_textureRef diffuse = daeSafeCast<domFx_common_color_or_texture>(phong->createAndPlace(COLLADA_ELEMENT_DIFFUSE));
                    {
                        // <color>r g b a
                        domFx_common_color_or_texture::domColorRef diffuse_color = daeSafeCast<domFx_common_color_or_texture::domColor>(diffuse->createAndPlace(COLLADA_ELEMENT_COLOR));
                        diffuse_color->getValue().setCount(4);
                        diffuse_color->getValue()[0] = color_diffuse.r;
                        diffuse_color->getValue()[1] = color_diffuse.g;
                        diffuse_color->getValue()[2] = color_diffuse.b;
                        diffuse_color->getValue()[3] = color_diffuse.a;
                        // </color>
                    }
                    // </diffuse>
                }
                // </phong>
            }
            // </technique>
        }
        // </profile_COMMON>
    }
    // </effect>

    return effect;
}

void ColladaWriter::addVisualLink(shared_ptr<urdf::Link const> urdf_link, daeElementRef parent, int& link_num) {
    // <node id="v1.node0" name="base_link" sid="node0">
    domNodeRef node = daeSafeCast<domNode>(parent->createAndPlace(COLLADA_ELEMENT_NODE));
    string node_sid = string("node") + boost::lexical_cast<string>(link_num);
    string node_id = string("v1.") + node_sid;
    node->setName(urdf_link->name.c_str());
    node->setSid(node_sid.c_str());
    node->setId(node_id.c_str());
    link_num++;
    {
        if (urdf_link->parent_joint != NULL) {
            // <rotate>x y z w</rotate>
            addRotate(node, urdf_link->parent_joint->parent_to_joint_origin_transform.rotation);
            // <translate>x y z</translate>
            addTranslate(node, urdf_link->parent_joint->parent_to_joint_origin_transform.position);

            // <rotate sid="node_joint0_axis0">x y z angle</rotate>
            domRotateRef joint_rotate = addRotate(node, urdf_link->parent_joint->parent_to_joint_origin_transform.rotation);
            string joint_sid = joint_sids_[urdf_link->parent_joint->name];
            string joint_rotate_sid = string("node_") + joint_sid + string("_axis0");
            joint_rotate->setSid(joint_rotate_sid.c_str());

            node_ids_[urdf_link->parent_joint->name] = node_id;
        }

        // <instance_geometry url="#g1.link0.geom">
        map<string, string>::const_iterator i = geometry_ids_.find(urdf_link->name);
        if (i != geometry_ids_.end()) {
            domInstance_geometryRef instance_geometry = daeSafeCast<domInstance_geometry>(node->createAndPlace(COLLADA_ELEMENT_INSTANCE_GEOMETRY));
            string geometry_id = i->second;
            string instance_geometry_url = string("#") + geometry_id;
            instance_geometry->setUrl(instance_geometry_url.c_str());
            {
                // <bind_material>
                domBind_materialRef bind_material = daeSafeCast<domBind_material>(instance_geometry->createAndPlace(COLLADA_ELEMENT_BIND_MATERIAL));
                {
                    // <technique_common>
                    domBind_material::domTechnique_commonRef technique_common = daeSafeCast<domBind_material::domTechnique_common>(bind_material->createAndPlace(COLLADA_ELEMENT_TECHNIQUE_COMMON));
                    {
                        // <instance_material>
                        domInstance_materialRef instance_material = daeSafeCast<domInstance_material>(technique_common->createAndPlace(COLLADA_ELEMENT_INSTANCE_MATERIAL));
                        instance_material->setTarget((instance_geometry_url + string(".mat")).c_str());
                        instance_material->setSymbol("mat0");
                        // </instance_material>
                    }
                    // </technique_common>
                }
                // </bind_material>
            }
        }
        // </instance_geometry>

        // <node ...>
        foreach(shared_ptr<urdf::Link> link2, urdf_link->child_links)
            addVisualLink(link2, node, link_num);
        // </node>
    }
    // </node>
}

domTranslateRef ColladaWriter::addTranslate(daeElementRef parent, urdf::Vector3 const& position) {
    // <translate>x y z</translate>
    domTranslateRef trans = daeSafeCast<domTranslate>(parent->createAndPlace(COLLADA_ELEMENT_TRANSLATE));
    trans->getValue().setCount(3);
    trans->getValue()[0] = position.x;
    trans->getValue()[1] = position.y;
    trans->getValue()[2] = position.z;
    return trans;
}

domRotateRef ColladaWriter::addRotate(daeElementRef parent, urdf::Rotation const& r) {
    double ax, ay, az, aa;

    // Convert from quaternion to axis-angle
    double sqr_len = r.x * r.x + r.y * r.y + r.z * r.z;
    if (sqr_len > 0) {
        aa = 2 * acos(r.w);

        double inv_len = 1.0 / sqrt(sqr_len);
        ax = r.x * inv_len;
        ay = r.y * inv_len;
        az = r.z * inv_len;
    }
    else {
        // Angle is 0 (mod 2*pi), so any axis will do
        aa = 0.0;
        ax = 1.0;
        ay = 0.0;
        az = 0.0;
    }

    // <rotate>x y z w</rotate>
    domRotateRef rot = daeSafeCast<domRotate>(parent->createAndPlace(COLLADA_ELEMENT_ROTATE));
    rot->getValue().setCount(4);
    rot->getValue()[0] = ax;
    rot->getValue()[1] = ay;
    rot->getValue()[2] = az;
    rot->getValue()[3] = angles::to_degrees(aa);

    return rot;
}

string ColladaWriter::getTimeStampString() const {
    //"2009-04-06T17:01:00.891550"

    // facet becomes owned by locale, so no need to explicitly delete
    boost::posix_time::time_facet* facet = new boost::posix_time::time_facet("%Y-%m-%dT%H:%M:%s");

    std::stringstream ss(std::stringstream::in | std::stringstream::out);
    ss.imbue(std::locale(ss.getloc(), facet));
    ss << boost::posix_time::second_clock::local_time();

    string date;
    ss >> date;

    return date;
}

}
