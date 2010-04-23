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

#ifndef COLLADA_URDF_COLLADA_WRITER_H
#define COLLADA_URDF_COLLADA_WRITER_H

#include <map>

#include <dae.h>
#include <dae/daeDocument.h>
#include <dae/daeErrorHandler.h>
#include <dae/domAny.h>
#include <dom/domCOLLADA.h>
#include <dom/domConstants.h>
#include <dom/domElements.h>
#include <dom/domTriangles.h>
#include <dom/domTypes.h>
#include <resource_retriever/retriever.h>
#include <urdf/model.h>
#include <urdf/pose.h>
#include <angles/angles.h>

namespace collada_urdf {

class ColladaWriterException : public std::runtime_error
{
public:
    ColladaWriterException(std::string const& what) : std::runtime_error(what) { }
};

class Mesh;

class ColladaWriter : public daeErrorHandler
{
private:
    struct SCENE
    {
        domVisual_sceneRef              vscene;
        domKinematics_sceneRef          kscene;
        domPhysics_sceneRef             pscene;
        domInstance_with_extraRef       viscene;
        domInstance_kinematics_sceneRef kiscene;
        domInstance_with_extraRef       piscene;
    };

public:
    /** \brief Create a ColladaWriter using the specified URDF filename.
     *
     * \param filename The name of the URDF file to convert
     * \throws ColladaWriterException if file could not be opened, or URDF could not be parsed
     */
    ColladaWriter(std::string const& filename);

    /** \brief Create a ColladaWriter using the specified URDF robot model and source.
     *
     * \param robot The URDF model to write
     * \param source The source of the model, e.g. the URL the URDF was read from
     */
    ColladaWriter(boost::shared_ptr<urdf::Model> robot, std::string const& source);

    virtual ~ColladaWriter();

    /** \brief Write the model to a COLLADA file.
     *
     * \param documentName The filename of the document to write to
     * \return True if the file was successfully written
     */
    bool writeDocument(std::string const& documentName);

protected:
    virtual void handleError(daeString msg);
    virtual void handleWarning(daeString msg);

private:
    void  initDocument(std::string const& documentName);
    SCENE createScene();

    void setupPhysics(SCENE const& scene);

    void addGeometries();
    void loadMesh(std::string const& filename, domGeometryRef geometry, std::string const& geometry_id);
    bool loadMeshWithSTLLoader(resource_retriever::MemoryResource const& resource, domGeometryRef geometry, std::string const& geometry_id);
    void buildMeshFromSTLLoader(boost::shared_ptr<Mesh> stl_mesh, daeElementRef parent, std::string const& geometry_id);

    void addKinematics(SCENE const& scene);
    void addJoints(daeElementRef parent);
    void addKinematicLink(boost::shared_ptr<urdf::Link const> urdf_link, daeElementRef parent, int& link_num);

    void         addVisuals(SCENE const& scene);
    void         addVisualLink(boost::shared_ptr<urdf::Link const> urdf_link, daeElementRef parent, int& link_num);

    void         addMaterials();
    domEffectRef addEffect(std::string const& geometry_id, urdf::Color const& color_ambient, urdf::Color const& color_diffuse);

    void addBindings(SCENE const& scene);

    domTranslateRef addTranslate(daeElementRef parent, urdf::Vector3 const& position);
    domRotateRef    addRotate(daeElementRef parent, urdf::Rotation const& r);

    std::string getTimeStampString() const;

private:
    boost::shared_ptr<urdf::Model> robot_;
    std::string                    source_;

    DAE*                            dae_;
    boost::shared_ptr<DAE>          collada_;
    domCOLLADA*                     dom_;
    domCOLLADA::domSceneRef         scene_;

    domLibrary_geometriesRef        geometriesLib_;
    domLibrary_visual_scenesRef     visualScenesLib_;
    domLibrary_kinematics_scenesRef kinematicsScenesLib_;
    domLibrary_kinematics_modelsRef kinematicsModelsLib_;
    domLibrary_jointsRef            jointsLib_;
    domLibrary_physics_scenesRef    physicsScenesLib_;
    domLibrary_materialsRef         materialsLib_;
    domLibrary_effectsRef           effectsLib_;

    domKinematics_modelRef kmodel_;

    std::map<std::string, std::string> geometry_ids_;   //!< link.name -> geometry.id
    std::map<std::string, std::string> joint_sids_;     //!< joint.name -> joint.sid
    std::map<std::string, std::string> node_ids_;       //!< joint.name -> node.id
};

}

#endif
