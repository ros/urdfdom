/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2010, Willow Garage, Inc., University of Tokyo
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

/* Authors: Rosen Diankov, Tim Field */

#include "collada_urdf/collada_urdf.h"
#include <map>
#include <vector>
#include <list>

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

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/posix_time_io.hpp>
#include <boost/format.hpp>

#include <assimp/assimp.hpp>
#include <assimp/aiScene.h>
#include <assimp/aiPostProcess.h>
#include <assimp/IOStream.h>
#include <assimp/IOSystem.h>

#define FOREACH(it, v) for(typeof((v).begin()) it = (v).begin(); it != (v).end(); (it)++)
#define FOREACHC FOREACH

using namespace std;

namespace collada_urdf {

/// ResourceIOStream is copied from rviz (BSD, Willow Garage)
class ResourceIOStream : public Assimp::IOStream
{
public:
    ResourceIOStream(const resource_retriever::MemoryResource& res)
        : res_(res)
        , pos_(res.data.get())
    {}

    ~ResourceIOStream()
    {}

    size_t Read(void* buffer, size_t size, size_t count)
    {
        size_t to_read = size * count;
        if (pos_ + to_read > res_.data.get() + res_.size)
            {
                to_read = res_.size - (pos_ - res_.data.get());
            }

        memcpy(buffer, pos_, to_read);
        pos_ += to_read;

        return to_read;
    }

    size_t Write( const void* buffer, size_t size, size_t count) { ROS_BREAK(); return 0; }

    aiReturn Seek( size_t offset, aiOrigin origin)
    {
        uint8_t* new_pos = 0;
        switch (origin)
            {
            case aiOrigin_SET:
                new_pos = res_.data.get() + offset;
                break;
            case aiOrigin_CUR:
                new_pos = pos_ + offset; // TODO is this right?  can offset really not be negative
                break;
            case aiOrigin_END:
                new_pos = res_.data.get() + res_.size - offset; // TODO is this right?
                break;
            default:
                ROS_BREAK();
            }

        if (new_pos < res_.data.get() || new_pos > res_.data.get() + res_.size)
            {
                return aiReturn_FAILURE;
            }

        pos_ = new_pos;
        return aiReturn_SUCCESS;
    }

    size_t Tell() const
    {
        return pos_ - res_.data.get();
    }

    size_t FileSize() const
    {
        return res_.size;
    }

    void Flush() {}

private:
    resource_retriever::MemoryResource res_;
    uint8_t* pos_;
};

/// ResourceIOSystem is copied from rviz (BSD, Willow Garage)
class ResourceIOSystem : public Assimp::IOSystem
{
public:
    ResourceIOSystem()
    {
    }

    ~ResourceIOSystem()
    {
    }

    // Check whether a specific file exists
    bool Exists(const char* file) const
    {
        // Ugly -- two retrievals where there should be one (Exists + Open)
        // resource_retriever needs a way of checking for existence
        // TODO: cache this
        resource_retriever::MemoryResource res;
        try {
            res = retriever_.get(file);
        }
        catch (resource_retriever::Exception& e) {
            return false;
        }

        return true;
    }

    // Get the path delimiter character we'd like to see
    char getOsSeparator() const
    {
        return '/';
    }

    // ... and finally a method to open a custom stream
    Assimp::IOStream* Open(const char* file, const char* mode)
    {
        ROS_ASSERT(mode == std::string("r") || mode == std::string("rb"));

        // Ugly -- two retrievals where there should be one (Exists + Open)
        // resource_retriever needs a way of checking for existence
        resource_retriever::MemoryResource res;
        try {
            res = retriever_.get(file);
        }
        catch (resource_retriever::Exception& e) {
            return 0;
        }

        return new ResourceIOStream(res);
    }

    void Close(Assimp::IOStream* stream) { delete stream; }

private:
    mutable resource_retriever::Retriever retriever_;
};

/** \brief Implements writing urdf::Model objects to a COLLADA DOM.
*/
class ColladaWriter : public daeErrorHandler
{
private:
    struct SCENE
    {
        domVisual_sceneRef vscene;
        domKinematics_sceneRef kscene;
        domPhysics_sceneRef pscene;
        domInstance_with_extraRef viscene;
        domInstance_kinematics_sceneRef kiscene;
        domInstance_with_extraRef piscene;
    };

    struct LINKOUTPUT
    {
        list<pair<int,string> > listusedlinks;
        list<pair<int,string> > listprocesseddofs;
        daeElementRef plink;
        domNodeRef pnode;
    };

    struct kinematics_model_output
    {
        struct axis_output
        {
            //axis_output(const string& sid, KinBody::JointConstPtr pjoint, int iaxis) : sid(sid), pjoint(pjoint), iaxis(iaxis) {}
        axis_output() : iaxis(0) {}
            string sid, nodesid;
            boost::shared_ptr<const urdf::Joint> pjoint;
            int iaxis;
            string jointnodesid;
        };
        domKinematics_modelRef kmodel;
        std::vector<axis_output> vaxissids;
        std::vector<std::string > vlinksids;
    };

    struct axis_sids
    {
    axis_sids(const string& axissid, const string& valuesid, const string& jointnodesid) : axissid(axissid), valuesid(valuesid), jointnodesid(jointnodesid) {}
        string axissid, valuesid, jointnodesid;
    };

    struct instance_kinematics_model_output
    {
        domInstance_kinematics_modelRef ikm;
        std::vector<axis_sids> vaxissids;
        boost::shared_ptr<kinematics_model_output> kmout;
        std::vector<std::pair<std::string,std::string> > vkinematicsbindings;
    };

    struct instance_articulated_system_output
    {
        domInstance_articulated_systemRef ias;
        std::vector<axis_sids> vaxissids;
        std::vector<std::string > vlinksids;
        std::vector<std::pair<std::string,std::string> > vkinematicsbindings;
    };

public:
    ColladaWriter(const urdf::Model& robot, int writeoptions) : _writeoptions(writeoptions), _robot(robot), _dom(NULL) {
        daeErrorHandler::setErrorHandler(this);        
        _collada.reset(new DAE);
        _collada->setIOPlugin(NULL);
        _collada->setDatabase(NULL);
        _importer.SetIOHandler(new ResourceIOSystem());
    }
    virtual ~ColladaWriter() {}

    boost::shared_ptr<DAE> convert()
    {
        try {
            const char* documentName = "urdf_snapshot";
            daeDocument *doc = NULL;
            daeInt error = _collada->getDatabase()->insertDocument(documentName, &doc ); // also creates a collada root
            if (error != DAE_OK || doc == NULL) {
                throw ColladaUrdfException("Failed to create document");
            }
            _dom = daeSafeCast<domCOLLADA>(doc->getDomRoot());
            _dom->setAttribute("xmlns:math","http://www.w3.org/1998/Math/MathML");

            //create the required asset tag
            domAssetRef asset = daeSafeCast<domAsset>( _dom->add( COLLADA_ELEMENT_ASSET ) );
            {
                // facet becomes owned by locale, so no need to explicitly delete
                boost::posix_time::time_facet* facet = new boost::posix_time::time_facet("%Y-%m-%dT%H:%M:%s");
                std::stringstream ss;
                ss.imbue(std::locale(ss.getloc(), facet));
                ss << boost::posix_time::second_clock::local_time();

                domAsset::domCreatedRef created = daeSafeCast<domAsset::domCreated>( asset->add( COLLADA_ELEMENT_CREATED ) );
                created->setValue(ss.str().c_str());
                domAsset::domModifiedRef modified = daeSafeCast<domAsset::domModified>( asset->add( COLLADA_ELEMENT_MODIFIED ) );
                modified->setValue(ss.str().c_str());

                domAsset::domContributorRef contrib = daeSafeCast<domAsset::domContributor>( asset->add( COLLADA_TYPE_CONTRIBUTOR ) );
                domAsset::domContributor::domAuthoring_toolRef authoringtool = daeSafeCast<domAsset::domContributor::domAuthoring_tool>( contrib->add( COLLADA_ELEMENT_AUTHORING_TOOL ) );
                authoringtool->setValue("URDF Collada Writer");

                domAsset::domUnitRef units = daeSafeCast<domAsset::domUnit>( asset->add( COLLADA_ELEMENT_UNIT ) );
                units->setMeter(1);
                units->setName("meter");

                domAsset::domUp_axisRef zup = daeSafeCast<domAsset::domUp_axis>( asset->add( COLLADA_ELEMENT_UP_AXIS ) );
                zup->setValue(UP_AXIS_Z_UP);
            }

            _globalscene = _dom->getScene();
            if( !_globalscene ) {
                _globalscene = daeSafeCast<domCOLLADA::domScene>( _dom->add( COLLADA_ELEMENT_SCENE ) );
            }

            _visualScenesLib = daeSafeCast<domLibrary_visual_scenes>(_dom->add(COLLADA_ELEMENT_LIBRARY_VISUAL_SCENES));
            _visualScenesLib->setId("vscenes");
            _geometriesLib = daeSafeCast<domLibrary_geometries>(_dom->add(COLLADA_ELEMENT_LIBRARY_GEOMETRIES));
            _geometriesLib->setId("geometries");
            _effectsLib = daeSafeCast<domLibrary_effects>(_dom->add(COLLADA_ELEMENT_LIBRARY_EFFECTS));
            _effectsLib->setId("effects");
            _materialsLib = daeSafeCast<domLibrary_materials>(_dom->add(COLLADA_ELEMENT_LIBRARY_MATERIALS));
            _materialsLib->setId("materials");
            _kinematicsModelsLib = daeSafeCast<domLibrary_kinematics_models>(_dom->add(COLLADA_ELEMENT_LIBRARY_KINEMATICS_MODELS));
            _kinematicsModelsLib->setId("kmodels");
            _articulatedSystemsLib = daeSafeCast<domLibrary_articulated_systems>(_dom->add(COLLADA_ELEMENT_LIBRARY_ARTICULATED_SYSTEMS));
            _articulatedSystemsLib->setId("asystems");
            _kinematicsScenesLib = daeSafeCast<domLibrary_kinematics_scenes>(_dom->add(COLLADA_ELEMENT_LIBRARY_KINEMATICS_SCENES));
            _kinematicsScenesLib->setId("kscenes");
            _physicsScenesLib = daeSafeCast<domLibrary_physics_scenes>(_dom->add(COLLADA_ELEMENT_LIBRARY_PHYSICS_SCENES));
            _physicsScenesLib->setId("pscenes");
            domExtraRef pextra_library_sensors = daeSafeCast<domExtra>(_dom->add(COLLADA_ELEMENT_EXTRA));
            pextra_library_sensors->setId("sensors");
            pextra_library_sensors->setType("library_sensors");
            _sensorsLib = daeSafeCast<domTechnique>(pextra_library_sensors->add(COLLADA_ELEMENT_TECHNIQUE));
            _sensorsLib->setProfile("OpenRAVE"); ///< documented profile on robot extensions

            _CreateScene();
            _WritePhysics();
            _WriteRobot();
            _WriteBindingsInstance_kinematics_scene();
            return _collada;
        }
        catch (ColladaUrdfException ex) {
            ROS_ERROR("Error converting: %s", ex.what());
            return boost::shared_ptr<DAE>();
        }
    }

protected:
    virtual void handleError(daeString msg) { throw ColladaUrdfException(msg); }
    virtual void handleWarning(daeString msg) { std::cerr << "COLLADA DOM warning: " << msg << std::endl; }

    void _CreateScene()
    {
        // Create visual scene
        _scene.vscene = daeSafeCast<domVisual_scene>(_visualScenesLib->add(COLLADA_ELEMENT_VISUAL_SCENE));
        _scene.vscene->setId("vscene");
        _scene.vscene->setName("URDF Visual Scene");

        // Create kinematics scene
        _scene.kscene = daeSafeCast<domKinematics_scene>(_kinematicsScenesLib->add(COLLADA_ELEMENT_KINEMATICS_SCENE));
        _scene.kscene->setId("kscene");
        _scene.kscene->setName("URDF Kinematics Scene");

        // Create physic scene
        _scene.pscene = daeSafeCast<domPhysics_scene>(_physicsScenesLib->add(COLLADA_ELEMENT_PHYSICS_SCENE));
        _scene.pscene->setId("pscene");
        _scene.pscene->setName("URDF Physics Scene");

        // Create instance visual scene
        _scene.viscene = daeSafeCast<domInstance_with_extra>(_globalscene->add( COLLADA_ELEMENT_INSTANCE_VISUAL_SCENE ));
        _scene.viscene->setUrl( (string("#") + string(_scene.vscene->getID())).c_str() );

        // Create instance kinematics scene
        _scene.kiscene = daeSafeCast<domInstance_kinematics_scene>(_globalscene->add( COLLADA_ELEMENT_INSTANCE_KINEMATICS_SCENE ));
        _scene.kiscene->setUrl( (string("#") + string(_scene.kscene->getID())).c_str() );

        // Create instance physics scene
        _scene.piscene = daeSafeCast<domInstance_with_extra>(_globalscene->add( COLLADA_ELEMENT_INSTANCE_PHYSICS_SCENE ));
        _scene.piscene->setUrl( (string("#") + string(_scene.pscene->getID())).c_str() );
    }

    void _WritePhysics() {
        domPhysics_scene::domTechnique_commonRef common = daeSafeCast<domPhysics_scene::domTechnique_common>(_scene.pscene->add(COLLADA_ELEMENT_TECHNIQUE_COMMON));
        //  Create gravity
        domTargetable_float3Ref g = daeSafeCast<domTargetable_float3>(common->add(COLLADA_ELEMENT_GRAVITY));
        g->getValue().set3 (0,0,0);
    }

    /// \brief Write kinematic body in a given scene
    void _WriteRobot(int id = 0)
    {
        ROS_DEBUG_STREAM(str(boost::format("writing robot as instance_articulated_system (%d) %s\n")%id%_robot.getName()));
        string asid = str(boost::format("robot%d")%id);
        string askid = str(boost::format("%s_kinematics")%asid);
        string asmid = str(boost::format("%s_motion")%asid);
        string iassid = str(boost::format("%s_inst")%asmid);

        domInstance_articulated_systemRef ias = daeSafeCast<domInstance_articulated_system>(_scene.kscene->add(COLLADA_ELEMENT_INSTANCE_ARTICULATED_SYSTEM));
        ias->setSid(iassid.c_str());
        ias->setUrl((string("#")+asmid).c_str());
        ias->setName(_robot.getName().c_str());

        _iasout.reset(new instance_articulated_system_output());
        _iasout->ias = ias;

        // motion info
        domArticulated_systemRef articulated_system_motion = daeSafeCast<domArticulated_system>(_articulatedSystemsLib->add(COLLADA_ELEMENT_ARTICULATED_SYSTEM));
        articulated_system_motion->setId(asmid.c_str());
        domMotionRef motion = daeSafeCast<domMotion>(articulated_system_motion->add(COLLADA_ELEMENT_MOTION));
        domMotion_techniqueRef mt = daeSafeCast<domMotion_technique>(motion->add(COLLADA_ELEMENT_TECHNIQUE_COMMON));
        domInstance_articulated_systemRef ias_motion = daeSafeCast<domInstance_articulated_system>(motion->add(COLLADA_ELEMENT_INSTANCE_ARTICULATED_SYSTEM));
        ias_motion->setUrl(str(boost::format("#%s")%askid).c_str());

        // kinematics info
        domArticulated_systemRef articulated_system_kinematics = daeSafeCast<domArticulated_system>(_articulatedSystemsLib->add(COLLADA_ELEMENT_ARTICULATED_SYSTEM));
        articulated_system_kinematics->setId(askid.c_str());
        domKinematicsRef kinematics = daeSafeCast<domKinematics>(articulated_system_kinematics->add(COLLADA_ELEMENT_KINEMATICS));
        domKinematics_techniqueRef kt = daeSafeCast<domKinematics_technique>(kinematics->add(COLLADA_ELEMENT_TECHNIQUE_COMMON));

        _WriteInstance_kinematics_model(kinematics,askid,id);

        for(size_t idof = 0; idof < _ikmout->vaxissids.size(); ++idof) {
            string axis_infosid = str(boost::format("axis_info_inst%d")%idof);
            boost::shared_ptr<const urdf::Joint> pjoint = _ikmout->kmout->vaxissids.at(idof).pjoint;
            BOOST_ASSERT(_mapjointindices[pjoint] == (int)idof);
            //int iaxis = _ikmout->kmout->vaxissids.at(idof).iaxis;

            //  Kinematics axis info
            domKinematics_axis_infoRef kai = daeSafeCast<domKinematics_axis_info>(kt->add(COLLADA_ELEMENT_AXIS_INFO));
            kai->setAxis(str(boost::format("%s/%s")%_ikmout->kmout->kmodel->getID()%_ikmout->kmout->vaxissids.at(idof).sid).c_str());
            kai->setSid(axis_infosid.c_str());
            bool bactive = !pjoint->mimic;
            double flower=0, fupper=0;
            if( pjoint->type != urdf::Joint::CONTINUOUS ) {
                if( !!pjoint->limits ) {
                    flower = pjoint->limits->lower;
                    fupper = pjoint->limits->upper;
                }
                if( flower == fupper ) {
                    bactive = false;
                }
                double fmult = 1.0;
                if( pjoint->type != urdf::Joint::PRISMATIC ) {
                    fmult = 180.0/M_PI;
                }
                domKinematics_limitsRef plimits = daeSafeCast<domKinematics_limits>(kai->add(COLLADA_ELEMENT_LIMITS));
                daeSafeCast<domCommon_float_or_param::domFloat>(plimits->add(COLLADA_ELEMENT_MIN)->add(COLLADA_ELEMENT_FLOAT))->setValue(flower*fmult);
                daeSafeCast<domCommon_float_or_param::domFloat>(plimits->add(COLLADA_ELEMENT_MAX)->add(COLLADA_ELEMENT_FLOAT))->setValue(fupper*fmult);
            }

            domCommon_bool_or_paramRef active = daeSafeCast<domCommon_bool_or_param>(kai->add(COLLADA_ELEMENT_ACTIVE));
            daeSafeCast<domCommon_bool_or_param::domBool>(active->add(COLLADA_ELEMENT_BOOL))->setValue(bactive);
            domCommon_bool_or_paramRef locked = daeSafeCast<domCommon_bool_or_param>(kai->add(COLLADA_ELEMENT_LOCKED));
            daeSafeCast<domCommon_bool_or_param::domBool>(locked->add(COLLADA_ELEMENT_BOOL))->setValue(false);
        
            //  Motion axis info
            domMotion_axis_infoRef mai = daeSafeCast<domMotion_axis_info>(mt->add(COLLADA_ELEMENT_AXIS_INFO));
            mai->setAxis(str(boost::format("%s/%s")%askid%axis_infosid).c_str());
            if( !!pjoint->limits ) {
                domCommon_float_or_paramRef speed = daeSafeCast<domCommon_float_or_param>(mai->add(COLLADA_ELEMENT_SPEED));
                daeSafeCast<domCommon_float_or_param::domFloat>(speed->add(COLLADA_ELEMENT_FLOAT))->setValue(pjoint->limits->velocity);
                domCommon_float_or_paramRef accel = daeSafeCast<domCommon_float_or_param>(mai->add(COLLADA_ELEMENT_ACCELERATION));
                daeSafeCast<domCommon_float_or_param::domFloat>(accel->add(COLLADA_ELEMENT_FLOAT))->setValue(pjoint->limits->effort);
            }
        }

        // write the bindings
        string asmsym = str(boost::format("%s.%s")%asmid%_ikmout->ikm->getSid());
        string assym = str(boost::format("%s.%s")%_scene.kscene->getID()%_ikmout->ikm->getSid());
        FOREACH(it, _ikmout->vkinematicsbindings) {
            domKinematics_newparamRef abm = daeSafeCast<domKinematics_newparam>(ias_motion->add(COLLADA_ELEMENT_NEWPARAM));
            abm->setSid(asmsym.c_str());
            daeSafeCast<domKinematics_newparam::domSIDREF>(abm->add(COLLADA_ELEMENT_SIDREF))->setValue(it->first.c_str());
            domKinematics_bindRef ab = daeSafeCast<domKinematics_bind>(ias->add(COLLADA_ELEMENT_BIND));
            ab->setSymbol(assym.c_str());
            daeSafeCast<domKinematics_param>(ab->add(COLLADA_ELEMENT_PARAM))->setRef(asmsym.c_str());
            _iasout->vkinematicsbindings.push_back(make_pair(string(ab->getSymbol()), it->second));
        }
        for(size_t idof = 0; idof < _ikmout->vaxissids.size(); ++idof) {
            const axis_sids& kas = _ikmout->vaxissids.at(idof);
            domKinematics_newparamRef abm = daeSafeCast<domKinematics_newparam>(ias_motion->add(COLLADA_ELEMENT_NEWPARAM));
            abm->setSid(str(boost::format("%s.%s")%asmid%kas.axissid).c_str());
            daeSafeCast<domKinematics_newparam::domSIDREF>(abm->add(COLLADA_ELEMENT_SIDREF))->setValue(kas.axissid.c_str());
            domKinematics_bindRef ab = daeSafeCast<domKinematics_bind>(ias->add(COLLADA_ELEMENT_BIND));
            ab->setSymbol(str(boost::format("%s.%s")%assym%kas.axissid).c_str());
            daeSafeCast<domKinematics_param>(ab->add(COLLADA_ELEMENT_PARAM))->setRef(str(boost::format("%s.%s")%asmid%kas.axissid).c_str());
            string valuesid;
            if( kas.valuesid.size() > 0 ) {
                domKinematics_newparamRef abmvalue = daeSafeCast<domKinematics_newparam>(ias_motion->add(COLLADA_ELEMENT_NEWPARAM));
                abmvalue->setSid(str(boost::format("%s.%s")%asmid%kas.valuesid).c_str());
                daeSafeCast<domKinematics_newparam::domSIDREF>(abmvalue->add(COLLADA_ELEMENT_SIDREF))->setValue(kas.valuesid.c_str());
                domKinematics_bindRef abvalue = daeSafeCast<domKinematics_bind>(ias->add(COLLADA_ELEMENT_BIND));
                valuesid = str(boost::format("%s.%s")%assym%kas.valuesid);
                abvalue->setSymbol(valuesid.c_str());
                daeSafeCast<domKinematics_param>(abvalue->add(COLLADA_ELEMENT_PARAM))->setRef(str(boost::format("%s.%s")%asmid%kas.valuesid).c_str());
            }
            _iasout->vaxissids.push_back(axis_sids(ab->getSymbol(),valuesid,kas.jointnodesid));
        }
    }

    /// \brief Write kinematic body in a given scene
    virtual void _WriteInstance_kinematics_model(daeElementRef parent, const string& sidscope, int id)
    {
        ROS_DEBUG_STREAM(str(boost::format("writing instance_kinematics_model %s\n")%_robot.getName()));
        boost::shared_ptr<kinematics_model_output> kmout = WriteKinematics_model(id);

        _ikmout.reset(new instance_kinematics_model_output());
        _ikmout->kmout = kmout;
        _ikmout->ikm = daeSafeCast<domInstance_kinematics_model>(parent->add(COLLADA_ELEMENT_INSTANCE_KINEMATICS_MODEL));

        string symscope, refscope;
        if( sidscope.size() > 0 ) {
            symscope = sidscope+string(".");
            refscope = sidscope+string("/");
        }
        string ikmsid = str(boost::format("%s_inst")%kmout->kmodel->getID());
        _ikmout->ikm->setUrl(str(boost::format("#%s")%kmout->kmodel->getID()).c_str());
        _ikmout->ikm->setSid(ikmsid.c_str());

        domKinematics_newparamRef kbind = daeSafeCast<domKinematics_newparam>(_ikmout->ikm->add(COLLADA_ELEMENT_NEWPARAM));
        kbind->setSid((symscope+ikmsid).c_str());
        daeSafeCast<domKinematics_newparam::domSIDREF>(kbind->add(COLLADA_ELEMENT_SIDREF))->setValue((refscope+ikmsid).c_str());
        _ikmout->vkinematicsbindings.push_back(make_pair(string(kbind->getSid()), str(boost::format("visual%d/node0")%id)));

        _ikmout->vaxissids.reserve(kmout->vaxissids.size());
        int i = 0;
        FOREACH(it,kmout->vaxissids) {
            domKinematics_newparamRef kbind = daeSafeCast<domKinematics_newparam>(_ikmout->ikm->add(COLLADA_ELEMENT_NEWPARAM));
            string ref = it->sid;
            size_t index = ref.find("/");
            while(index != string::npos) {
                ref[index] = '.';
                index = ref.find("/",index+1);
            }
            string sid = symscope+ikmsid+"."+ref;
            kbind->setSid(sid.c_str());
            daeSafeCast<domKinematics_newparam::domSIDREF>(kbind->add(COLLADA_ELEMENT_SIDREF))->setValue((refscope+ikmsid+"/"+it->sid).c_str());
            double value=0;
            double flower=0, fupper=0;
            if( !!it->pjoint->limits ) {
                flower = it->pjoint->limits->lower;
                fupper = it->pjoint->limits->upper;
                if( flower > 0 || fupper < 0 ) {
                    value = 0.5*(flower+fupper);
                }
            }

            domKinematics_newparamRef pvalueparam = daeSafeCast<domKinematics_newparam>(_ikmout->ikm->add(COLLADA_ELEMENT_NEWPARAM));
            pvalueparam->setSid((sid+string("_value")).c_str());
            daeSafeCast<domKinematics_newparam::domFloat>(pvalueparam->add(COLLADA_ELEMENT_FLOAT))->setValue(value);
            _ikmout->vaxissids.push_back(axis_sids(sid,pvalueparam->getSid(),kmout->vaxissids.at(i).jointnodesid));
            ++i;
        }
    }

    virtual boost::shared_ptr<kinematics_model_output> WriteKinematics_model(int id)
    {
        domKinematics_modelRef kmodel = daeSafeCast<domKinematics_model>(_kinematicsModelsLib->add(COLLADA_ELEMENT_KINEMATICS_MODEL));
        string kmodelid = str(boost::format("kmodel%d")%id);
        kmodel->setId(kmodelid.c_str());
        kmodel->setName(_robot.getName().c_str());

        domKinematics_model_techniqueRef ktec = daeSafeCast<domKinematics_model_technique>(kmodel->add(COLLADA_ELEMENT_TECHNIQUE_COMMON));

        //  Create root node for the visual scene
        domNodeRef pnoderoot = daeSafeCast<domNode>(_scene.vscene->add(COLLADA_ELEMENT_NODE));
        string bodyid = str(boost::format("visual%d")%id);
        pnoderoot->setId(bodyid.c_str());
        pnoderoot->setSid(bodyid.c_str());
        pnoderoot->setName(_robot.getName().c_str());

        //  Declare all the joints
        _mapjointindices.clear();
        int index=0;
        FOREACHC(itj, _robot.joints_) {
            _mapjointindices[itj->second] = index++;
        }
        _maplinkindices.clear();
        index=0;
        FOREACHC(itj, _robot.links_) {
            _maplinkindices[itj->second] = index++;
        }
        _mapmaterialindices.clear();
        index=0;
        FOREACHC(itj, _robot.materials_) {
            _mapmaterialindices[itj->second] = index++;
        }

        double lmin, lmax;
        vector<domJointRef> vdomjoints(_robot.joints_.size());
        boost::shared_ptr<kinematics_model_output> kmout(new kinematics_model_output());
        kmout->kmodel = kmodel;
        kmout->vaxissids.resize(_robot.joints_.size());
        kmout->vlinksids.resize(_robot.links_.size());

        FOREACHC(itjoint, _robot.joints_) {
            boost::shared_ptr<urdf::Joint> pjoint = itjoint->second;
            int index = _mapjointindices[itjoint->second];
            domJointRef pdomjoint = daeSafeCast<domJoint>(ktec->add(COLLADA_ELEMENT_JOINT));
            string jointid = pjoint->name;//str(boost::format("joint%d")%index);
            pdomjoint->setSid( jointid.c_str() );
            pdomjoint->setName(pjoint->name.c_str());
            domAxis_constraintRef axis;
            if( !!pjoint->limits ) {
                lmin=pjoint->limits->lower;
                lmax=pjoint->limits->upper;
            }
            else {
                lmin = lmax = 0;
            }

            double fmult = 1.0;
            switch(pjoint->type) {
            case urdf::Joint::REVOLUTE:
            case urdf::Joint::CONTINUOUS:
                axis = daeSafeCast<domAxis_constraint>(pdomjoint->add(COLLADA_ELEMENT_REVOLUTE));
                fmult = 180.0f/M_PI;
                lmin*=fmult;
                lmax*=fmult;
                break;
            case urdf::Joint::PRISMATIC:
                axis = daeSafeCast<domAxis_constraint>(pdomjoint->add(COLLADA_ELEMENT_PRISMATIC));
                break;
            case urdf::Joint::FIXED:
                axis = daeSafeCast<domAxis_constraint>(pdomjoint->add(COLLADA_ELEMENT_REVOLUTE));
                lmin = 0;
                lmax = 0;
                fmult = 0;
                break;
            default:
                ROS_WARN_STREAM(str(boost::format("unsupported joint type specified %d")%(int)pjoint->type));
                break;
            }

            if( !axis ) {
                continue;
            }

            int ia = 0;
            string axisid = str(boost::format("axis%d")%ia);
            axis->setSid(axisid.c_str());
            kmout->vaxissids.at(index).pjoint = pjoint;
            kmout->vaxissids.at(index).sid = jointid+string("/")+axisid;
            kmout->vaxissids.at(index).iaxis = ia;
            domAxisRef paxis = daeSafeCast<domAxis>(axis->add(COLLADA_ELEMENT_AXIS));
            paxis->getValue().setCount(3);
            paxis->getValue()[0] = pjoint->axis.x;
            paxis->getValue()[1] = pjoint->axis.y;
            paxis->getValue()[2] = pjoint->axis.z;
            if( pjoint->type != urdf::Joint::CONTINUOUS ) {
                domJoint_limitsRef plimits = daeSafeCast<domJoint_limits>(axis->add(COLLADA_TYPE_LIMITS));
                daeSafeCast<domMinmax>(plimits->add(COLLADA_ELEMENT_MIN))->getValue() = lmin;
                daeSafeCast<domMinmax>(plimits->add(COLLADA_ELEMENT_MAX))->getValue() = lmax;
            }
            vdomjoints.at(index) = pdomjoint;
        }

        LINKOUTPUT childinfo = _WriteLink(_robot.getRoot(), ktec, pnoderoot, kmodel->getID());
        FOREACHC(itused, childinfo.listusedlinks) {
            kmout->vlinksids.at(itused->first) = itused->second;
        }
        FOREACH(itprocessed,childinfo.listprocesseddofs) {
            kmout->vaxissids.at(itprocessed->first).jointnodesid = itprocessed->second;
        }

        // create the formulas for all mimic joints
        FOREACHC(itjoint, _robot.joints_) {
            string jointsid = itjoint->second->name;
            boost::shared_ptr<urdf::Joint> pjoint = itjoint->second;
            if( !pjoint->mimic ) {
                continue;
            }

            domFormulaRef pf = daeSafeCast<domFormula>(ktec->add(COLLADA_ELEMENT_FORMULA));
            string formulaid = str(boost::format("%s_formula")%jointsid);
            pf->setSid(formulaid.c_str());
            domCommon_float_or_paramRef ptarget = daeSafeCast<domCommon_float_or_param>(pf->add(COLLADA_ELEMENT_TARGET));
            string targetjointid = str(boost::format("%s/%s")%kmodel->getID()%jointsid);
            daeSafeCast<domCommon_param>(ptarget->add(COLLADA_TYPE_PARAM))->setValue(targetjointid.c_str());

            domFormula_techniqueRef pftec = daeSafeCast<domFormula_technique>(pf->add(COLLADA_ELEMENT_TECHNIQUE_COMMON));
            // create a const0*joint+const1 formula
            // <apply> <plus/> <apply> <times/> <cn>a</cn> x </apply> <cn>b</cn> </apply>
            daeElementRef pmath_math = pftec->add("math");
            daeElementRef pmath_apply = pmath_math->add("apply");
            {
                daeElementRef pmath_plus = pmath_apply->add("plus");
                daeElementRef pmath_apply1 = pmath_apply->add("apply");
                {
                    daeElementRef pmath_times = pmath_apply1->add("times");
                    daeElementRef pmath_const0 = pmath_apply1->add("cn");
                    pmath_const0->setCharData(str(boost::format("%f")%pjoint->mimic->multiplier));
                    daeElementRef pmath_symb = pmath_apply1->add("csymbol");
                    pmath_symb->setAttribute("encoding","COLLADA");
                    pmath_symb->setCharData(str(boost::format("%s/%s")%kmodel->getID()%pjoint->mimic->joint_name));
                }
                daeElementRef pmath_const1 = pmath_apply->add("cn");
                pmath_const1->setCharData(str(boost::format("%f")%pjoint->mimic->offset));
            }
        }

        return kmout;
    }

    /// \brief Write link of a kinematic body
    /// \param link Link to write
    /// \param pkinparent Kinbody parent
    /// \param pnodeparent Node parent
    /// \param strModelUri
    virtual LINKOUTPUT _WriteLink(boost::shared_ptr<const urdf::Link> plink, daeElementRef pkinparent, domNodeRef pnodeparent, const string& strModelUri)
    {
        LINKOUTPUT out;
        int linkindex = _maplinkindices[plink];
        string linksid = plink->name;
        domLinkRef pdomlink = daeSafeCast<domLink>(pkinparent->add(COLLADA_ELEMENT_LINK));
        pdomlink->setName(plink->name.c_str());
        pdomlink->setSid(linksid.c_str());

        domNodeRef pnode = daeSafeCast<domNode>(pnodeparent->add(COLLADA_ELEMENT_NODE));
        string nodeid = str(boost::format("v%s.node%d")%strModelUri%linkindex);
        pnode->setId( nodeid.c_str() );
        string nodesid = str(boost::format("node%d")%linkindex);
        pnode->setSid(nodesid.c_str());
        pnode->setName(plink->name.c_str());

        boost::shared_ptr<urdf::Geometry> geometry;
        boost::shared_ptr<urdf::Material> material;
        urdf::Pose geometry_origin;
        if( !!plink->visual ) {
            geometry = plink->visual->geometry;
            material = plink->visual->material;
            geometry_origin = plink->visual->origin;
        }
        else if( !!plink->collision ) {
            geometry = plink->collision->geometry;
            geometry_origin = plink->collision->origin;
        }

        if( !!geometry ) {
            int igeom = 0;
            string geomid = str(boost::format("g%s.%s.geom%d")%strModelUri%linksid%igeom);
            domGeometryRef pdomgeom = _WriteGeometry(geometry, geomid);
            domInstance_geometryRef pinstgeom = daeSafeCast<domInstance_geometry>(pnode->add(COLLADA_ELEMENT_INSTANCE_GEOMETRY));
            pinstgeom->setUrl((string("#")+geomid).c_str());

            // material
            _WriteMaterial(pdomgeom->getID(), material);
            domBind_materialRef pmat = daeSafeCast<domBind_material>(pinstgeom->add(COLLADA_ELEMENT_BIND_MATERIAL));
            domBind_material::domTechnique_commonRef pmattec = daeSafeCast<domBind_material::domTechnique_common>(pmat->add(COLLADA_ELEMENT_TECHNIQUE_COMMON));
            domInstance_materialRef pinstmat = daeSafeCast<domInstance_material>(pmattec->add(COLLADA_ELEMENT_INSTANCE_MATERIAL));
            pinstmat->setTarget(xsAnyURI(*pdomgeom, string("#")+geomid+string(".mat")));
            pinstmat->setSymbol("mat0");
        }

        _WriteTransformation(pnode, geometry_origin);
        urdf::Pose geometry_origin_inv = _poseInverse(geometry_origin);
        
        // process all children
        FOREACHC(itjoint, plink->child_joints) {
            boost::shared_ptr<urdf::Joint> pjoint = *itjoint;
            int index = _mapjointindices[pjoint];

            // <attachment_full joint="k1/joint0">
            domLink::domAttachment_fullRef attachment_full = daeSafeCast<domLink::domAttachment_full>(pdomlink->add(COLLADA_ELEMENT_ATTACHMENT_FULL));
            string jointid = str(boost::format("%s/%s")%strModelUri%pjoint->name);
            attachment_full->setJoint(jointid.c_str());

            LINKOUTPUT childinfo = _WriteLink(_robot.getLink(pjoint->child_link_name), attachment_full, pnode, strModelUri);
            FOREACH(itused,childinfo.listusedlinks) {
                out.listusedlinks.push_back(make_pair(itused->first,linksid+string("/")+itused->second));
            }
            FOREACH(itprocessed,childinfo.listprocesseddofs) {
                out.listprocesseddofs.push_back(make_pair(itprocessed->first,nodesid+string("/")+itprocessed->second));
            }

            // rotate/translate elements
            string jointnodesid = str(boost::format("node_%s_axis0")%pjoint->name);
            switch(pjoint->type) {
            case urdf::Joint::REVOLUTE:
            case urdf::Joint::CONTINUOUS:
            case urdf::Joint::FIXED: {
                domRotateRef protate = daeSafeCast<domRotate>(childinfo.pnode->add(COLLADA_ELEMENT_ROTATE,0));
                protate->setSid(jointnodesid.c_str());
                protate->getValue().setCount(4);
                protate->getValue()[0] = pjoint->axis.x;
                protate->getValue()[1] = pjoint->axis.y;
                protate->getValue()[2] = pjoint->axis.z;
                protate->getValue()[3] = 0;
                break;
            }
            case urdf::Joint::PRISMATIC: {
                domTranslateRef ptrans = daeSafeCast<domTranslate>(childinfo.pnode->add(COLLADA_ELEMENT_TRANSLATE,0));
                ptrans->setSid(jointnodesid.c_str());
                ptrans->getValue().setCount(3);
                ptrans->getValue()[0] = 0;
                ptrans->getValue()[1] = 0;
                ptrans->getValue()[2] = 0;
                break;
            }
            default:
                ROS_WARN_STREAM(str(boost::format("unsupported joint type specified %d")%(int)pjoint->type));
                break;
            }
            
            _WriteTransformation(attachment_full, pjoint->parent_to_joint_origin_transform);
            _WriteTransformation(childinfo.pnode, pjoint->parent_to_joint_origin_transform);
            _WriteTransformation(childinfo.pnode, geometry_origin_inv); // have to do multiply by inverse since geometry transformation is not part of hierarchy
            out.listprocesseddofs.push_back(make_pair(index,string(childinfo.pnode->getSid())+string("/")+jointnodesid));
            // </attachment_full>
        }

        out.listusedlinks.push_back(make_pair(linkindex,linksid));
        out.plink = pdomlink;
        out.pnode = pnode;
        return  out;
    }

    domGeometryRef _WriteGeometry(boost::shared_ptr<urdf::Geometry> geometry, const std::string& geometry_id)
    {
        domGeometryRef cgeometry = daeSafeCast<domGeometry>(_geometriesLib->add(COLLADA_ELEMENT_GEOMETRY));
        cgeometry->setId(geometry_id.c_str());
        switch (geometry->type) {
        case urdf::Geometry::MESH: {
            urdf::Mesh* urdf_mesh = (urdf::Mesh*) geometry.get();
            _loadMesh(urdf_mesh->filename, cgeometry, urdf_mesh->scale);
            break;
        }
        case urdf::Geometry::SPHERE: {
            ROS_WARN_STREAM(str(boost::format("cannot export sphere geometries yet! %s")%geometry_id));
            break;
        }
        case urdf::Geometry::BOX: {
            ROS_WARN_STREAM(str(boost::format("cannot export box geometries yet! %s")%geometry_id));
            break;
        }
        case urdf::Geometry::CYLINDER: {
            ROS_WARN_STREAM(str(boost::format("cannot export cylinder geometries yet! %s")%geometry_id));
            break;
        }
        default: {
            throw ColladaUrdfException(str(boost::format("undefined geometry type %d, name %s")%(int)geometry->type%geometry_id));
        }
        }
        return cgeometry;
    }

    void _WriteMaterial(const string& geometry_id, boost::shared_ptr<urdf::Material> material)
    {
        string effid = geometry_id+string(".eff");
        string matid = geometry_id+string(".mat");
        domMaterialRef pdommat = daeSafeCast<domMaterial>(_materialsLib->add(COLLADA_ELEMENT_MATERIAL));
        pdommat->setId(matid.c_str());
        domInstance_effectRef pdominsteff = daeSafeCast<domInstance_effect>(pdommat->add(COLLADA_ELEMENT_INSTANCE_EFFECT));
        pdominsteff->setUrl((string("#")+effid).c_str());

        urdf::Color ambient, diffuse;
        ambient.init("0.1 0.1 0.1 0");
        diffuse.init("1 1 1 0");

        if( !!material ) {
            ambient.r = diffuse.r = material->color.r;
            ambient.g = diffuse.g = material->color.g;
            ambient.b = diffuse.b = material->color.b;
            ambient.a = diffuse.a = material->color.a;
        }

        domEffectRef effect = _WriteEffect(geometry_id, ambient, diffuse);

        // <material id="g1.link0.geom0.eff">
        domMaterialRef dommaterial = daeSafeCast<domMaterial>(_materialsLib->add(COLLADA_ELEMENT_MATERIAL));
        string material_id = geometry_id + string(".mat");
        dommaterial->setId(material_id.c_str());
        {
            // <instance_effect url="#g1.link0.geom0.eff"/>
            domInstance_effectRef instance_effect = daeSafeCast<domInstance_effect>(dommaterial->add(COLLADA_ELEMENT_INSTANCE_EFFECT));
            string effect_id(effect->getId());
            instance_effect->setUrl((string("#") + effect_id).c_str());
        }
        // </material>

        domEffectRef pdomeff = _WriteEffect(effid, ambient, diffuse);
    }

    void _loadMesh(std::string const& filename, domGeometryRef pdomgeom, const urdf::Vector3& scale)
    {        
        const aiScene* scene = _importer.ReadFile(filename, aiProcess_SortByPType|aiProcess_Triangulate);//|aiProcess_GenNormals|aiProcess_GenUVCoords|aiProcess_FlipUVs);
        if( !scene ) {
            ROS_WARN("failed to load resource %s",filename.c_str());
            return;
        }
        if( !scene->mRootNode ) {
            ROS_WARN("resource %s has no data",filename.c_str());
            return;
        }
        if (!scene->HasMeshes()) {
            ROS_WARN_STREAM(str(boost::format("No meshes found in file %s")%filename));
            return;
        }
        domMeshRef pdommesh = daeSafeCast<domMesh>(pdomgeom->add(COLLADA_ELEMENT_MESH));
        domSourceRef pvertsource = daeSafeCast<domSource>(pdommesh->add(COLLADA_ELEMENT_SOURCE));
        domAccessorRef pacc;
        domFloat_arrayRef parray;
        {
            pvertsource->setId(str(boost::format("%s.positions")%pdomgeom->getID()).c_str());

            parray = daeSafeCast<domFloat_array>(pvertsource->add(COLLADA_ELEMENT_FLOAT_ARRAY));
            parray->setId(str(boost::format("%s.positions-array")%pdomgeom->getID()).c_str());
            parray->setDigits(6); // 6 decimal places

            domSource::domTechnique_commonRef psourcetec = daeSafeCast<domSource::domTechnique_common>(pvertsource->add(COLLADA_ELEMENT_TECHNIQUE_COMMON));
            pacc = daeSafeCast<domAccessor>(psourcetec->add(COLLADA_ELEMENT_ACCESSOR));
            pacc->setSource(xsAnyURI(*parray, string("#")+string(parray->getID())));
            pacc->setStride(3);

            domParamRef px = daeSafeCast<domParam>(pacc->add(COLLADA_ELEMENT_PARAM));
            px->setName("X"); px->setType("float");
            domParamRef py = daeSafeCast<domParam>(pacc->add(COLLADA_ELEMENT_PARAM));
            py->setName("Y"); py->setType("float");
            domParamRef pz = daeSafeCast<domParam>(pacc->add(COLLADA_ELEMENT_PARAM));
            pz->setName("Z"); pz->setType("float");
        }
        domVerticesRef pverts = daeSafeCast<domVertices>(pdommesh->add(COLLADA_ELEMENT_VERTICES));
        {
            pverts->setId("vertices");
            domInput_localRef pvertinput = daeSafeCast<domInput_local>(pverts->add(COLLADA_ELEMENT_INPUT));
            pvertinput->setSemantic("POSITION");
            pvertinput->setSource(domUrifragment(*pvertsource, string("#")+string(pvertsource->getID())));
        }
        _buildAiMesh(scene,scene->mRootNode,pdommesh,parray, pdomgeom->getID(),scale);
        pacc->setCount(parray->getCount());
    }

    void _buildAiMesh(const aiScene* scene, aiNode* node, domMeshRef pdommesh, domFloat_arrayRef parray, const string& geomid, const urdf::Vector3& scale)
    {
        if( !node ) {
            return;
        }
        aiMatrix4x4 transform = node->mTransformation;
        aiNode *pnode = node->mParent;
        while (pnode) {
            // Don't convert to y-up orientation, which is what the root node in
            // Assimp does
            if (pnode->mParent != NULL) {
                transform = pnode->mTransformation * transform;
            }
            pnode = pnode->mParent;
        }

        {
            uint32_t vertexOffset = parray->getCount();
            uint32_t nTotalVertices=0;
            for (uint32_t i = 0; i < node->mNumMeshes; i++) {
                aiMesh* input_mesh = scene->mMeshes[node->mMeshes[i]];
                nTotalVertices += input_mesh->mNumVertices;
            }

            parray->getValue().grow(parray->getCount()+nTotalVertices*3);
            parray->setCount(parray->getCount()+nTotalVertices);

            for (uint32_t i = 0; i < node->mNumMeshes; i++) {
                aiMesh* input_mesh = scene->mMeshes[node->mMeshes[i]];
                for (uint32_t j = 0; j < input_mesh->mNumVertices; j++) {
                    aiVector3D p = input_mesh->mVertices[j];
                    p *= transform;
                    parray->getValue().append(p.x*scale.x);
                    parray->getValue().append(p.y*scale.y);
                    parray->getValue().append(p.z*scale.z);
                }
            }

            // in order to save space, separate triangles from poly lists
            
            vector<int> triangleindices, otherindices;
            int nNumOtherPrimitives = 0;
            for (uint32_t i = 0; i < node->mNumMeshes; i++) {
                aiMesh* input_mesh = scene->mMeshes[node->mMeshes[i]];
                uint32_t indexCount = 0, otherIndexCount = 0;
                for (uint32_t j = 0; j < input_mesh->mNumFaces; j++) {
                    aiFace& face = input_mesh->mFaces[j];
                    if( face.mNumIndices == 3 ) {
                        indexCount += face.mNumIndices;
                    }
                    else {
                        otherIndexCount += face.mNumIndices;
                    }
                }
                triangleindices.reserve(triangleindices.size()+indexCount);
                otherindices.reserve(otherindices.size()+otherIndexCount);
                for (uint32_t j = 0; j < input_mesh->mNumFaces; j++) {
                    aiFace& face = input_mesh->mFaces[j];
                    if( face.mNumIndices == 3 ) {
                        triangleindices.push_back(vertexOffset+face.mIndices[0]);
                        triangleindices.push_back(vertexOffset+face.mIndices[1]);
                        triangleindices.push_back(vertexOffset+face.mIndices[2]);
                    }
                    else {
                        for (uint32_t k = 0; k < face.mNumIndices; ++k) {
                            otherindices.push_back(face.mIndices[k]+vertexOffset);
                        }
                        nNumOtherPrimitives++;
                    }
                }
                vertexOffset += input_mesh->mNumVertices;
            }

            if( triangleindices.size() > 0 ) {
                domTrianglesRef ptris = daeSafeCast<domTriangles>(pdommesh->add(COLLADA_ELEMENT_TRIANGLES));
                ptris->setCount(triangleindices.size()/3);
                ptris->setMaterial("mat0");
                domInput_local_offsetRef pvertoffset = daeSafeCast<domInput_local_offset>(ptris->add(COLLADA_ELEMENT_INPUT));
                pvertoffset->setSemantic("VERTEX");
                pvertoffset->setOffset(0);
                pvertoffset->setSource(domUrifragment(*pdommesh->getVertices(), str(boost::format("#%s/vertices")%geomid)));
                domPRef pindices = daeSafeCast<domP>(ptris->add(COLLADA_ELEMENT_P));
                pindices->getValue().setCount(triangleindices.size());
                for(size_t ind = 0; ind < triangleindices.size(); ++ind) {
                    pindices->getValue()[ind] = triangleindices[ind];
                }
            }

            if( nNumOtherPrimitives > 0 ) {
                domPolylistRef ptris = daeSafeCast<domPolylist>(pdommesh->add(COLLADA_ELEMENT_POLYLIST));
                ptris->setCount(nNumOtherPrimitives);
                ptris->setMaterial("mat0");
                domInput_local_offsetRef pvertoffset = daeSafeCast<domInput_local_offset>(ptris->add(COLLADA_ELEMENT_INPUT));
                pvertoffset->setSemantic("VERTEX");
                pvertoffset->setSource(domUrifragment(*pdommesh->getVertices(), str(boost::format("#%s/vertices")%geomid)));
                domPRef pindices = daeSafeCast<domP>(ptris->add(COLLADA_ELEMENT_P));
                pindices->getValue().setCount(otherindices.size());
                for(size_t ind = 0; ind < otherindices.size(); ++ind) {
                    pindices->getValue()[ind] = otherindices[ind];
                }
                
                domPolylist::domVcountRef pcount = daeSafeCast<domPolylist::domVcount>(ptris->add(COLLADA_ELEMENT_VCOUNT));
                pcount->getValue().setCount(nNumOtherPrimitives);
                uint32_t offset = 0;
                for (uint32_t i = 0; i < node->mNumMeshes; i++) {
                    aiMesh* input_mesh = scene->mMeshes[node->mMeshes[i]];
                    for (uint32_t j = 0; j < input_mesh->mNumFaces; j++) {
                        aiFace& face = input_mesh->mFaces[j];
                        if( face.mNumIndices > 3 ) {
                            pcount->getValue()[offset++] = face.mNumIndices;
                        }
                    }
                }
            }
        }

        for (uint32_t i=0; i < node->mNumChildren; ++i) {
            _buildAiMesh(scene, node->mChildren[i], pdommesh,parray,geomid,scale);
        }
    }


    domEffectRef _WriteEffect(std::string const& effect_id, urdf::Color const& color_ambient, urdf::Color const& color_diffuse)
    {
        // <effect id="g1.link0.geom0.eff">
        domEffectRef effect = daeSafeCast<domEffect>(_effectsLib->add(COLLADA_ELEMENT_EFFECT));
        effect->setId(effect_id.c_str());
        {
            // <profile_COMMON>
            domProfile_commonRef profile = daeSafeCast<domProfile_common>(effect->add(COLLADA_ELEMENT_PROFILE_COMMON));
            {
                // <technique sid="">
                domProfile_common::domTechniqueRef technique = daeSafeCast<domProfile_common::domTechnique>(profile->add(COLLADA_ELEMENT_TECHNIQUE));
                {
                    // <phong>
                    domProfile_common::domTechnique::domPhongRef phong = daeSafeCast<domProfile_common::domTechnique::domPhong>(technique->add(COLLADA_ELEMENT_PHONG));
                    {
                        // <ambient>
                        domFx_common_color_or_textureRef ambient = daeSafeCast<domFx_common_color_or_texture>(phong->add(COLLADA_ELEMENT_AMBIENT));
                        {
                            // <color>r g b a
                            domFx_common_color_or_texture::domColorRef ambient_color = daeSafeCast<domFx_common_color_or_texture::domColor>(ambient->add(COLLADA_ELEMENT_COLOR));
                            ambient_color->getValue().setCount(4);
                            ambient_color->getValue()[0] = color_ambient.r;
                            ambient_color->getValue()[1] = color_ambient.g;
                            ambient_color->getValue()[2] = color_ambient.b;
                            ambient_color->getValue()[3] = color_ambient.a;
                            // </color>
                        }
                        // </ambient>

                        // <diffuse>
                        domFx_common_color_or_textureRef diffuse = daeSafeCast<domFx_common_color_or_texture>(phong->add(COLLADA_ELEMENT_DIFFUSE));
                        {
                            // <color>r g b a
                            domFx_common_color_or_texture::domColorRef diffuse_color = daeSafeCast<domFx_common_color_or_texture::domColor>(diffuse->add(COLLADA_ELEMENT_COLOR));
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

    /// \brief Write transformation
    /// \param pelt Element to transform
    /// \param t Transform to write
    void _WriteTransformation(daeElementRef pelt, const urdf::Pose& t)
    {
        domRotateRef prot = daeSafeCast<domRotate>(pelt->add(COLLADA_ELEMENT_ROTATE,0));
        domTranslateRef ptrans = daeSafeCast<domTranslate>(pelt->add(COLLADA_ELEMENT_TRANSLATE,0));
        ptrans->getValue().setCount(3);
        ptrans->getValue()[0] = t.position.x;
        ptrans->getValue()[1] = t.position.y;
        ptrans->getValue()[2] = t.position.z;

        prot->getValue().setCount(4);
        // extract axis from quaternion
        double sinang = t.rotation.x*t.rotation.x+t.rotation.y*t.rotation.y+t.rotation.z*t.rotation.z;
        if( std::fabs(sinang) < 1e-10 ) {
            prot->getValue()[0] = 1;
            prot->getValue()[1] = 0;
            prot->getValue()[2] = 0;
            prot->getValue()[3] = 0;
        }
        else {
            urdf::Rotation quat;
            if( t.rotation.w < 0 ) {
                quat.x = -t.rotation.x;
                quat.y = -t.rotation.y;
                quat.z = -t.rotation.z;
                quat.w = -t.rotation.w;
            }
            else {
                quat = t.rotation;
            }
            sinang = std::sqrt(sinang);
            prot->getValue()[0] = quat.x/sinang;
            prot->getValue()[1] = quat.y/sinang;
            prot->getValue()[2] = quat.z/sinang;
            prot->getValue()[3] = angles::to_degrees(2.0*std::atan2(sinang,quat.w));
        }
    }

    // binding in instance_kinematics_scene
    void _WriteBindingsInstance_kinematics_scene()
    {
        FOREACHC(it, _iasout->vkinematicsbindings) {
            domBind_kinematics_modelRef pmodelbind = daeSafeCast<domBind_kinematics_model>(_scene.kiscene->add(COLLADA_ELEMENT_BIND_KINEMATICS_MODEL));
            pmodelbind->setNode(it->second.c_str());
            daeSafeCast<domCommon_param>(pmodelbind->add(COLLADA_ELEMENT_PARAM))->setValue(it->first.c_str());
        }
        FOREACHC(it, _iasout->vaxissids) {
            domBind_joint_axisRef pjointbind = daeSafeCast<domBind_joint_axis>(_scene.kiscene->add(COLLADA_ELEMENT_BIND_JOINT_AXIS));
            pjointbind->setTarget(it->jointnodesid.c_str());
            daeSafeCast<domCommon_param>(pjointbind->add(COLLADA_ELEMENT_AXIS)->add(COLLADA_TYPE_PARAM))->setValue(it->axissid.c_str());
            daeSafeCast<domCommon_param>(pjointbind->add(COLLADA_ELEMENT_VALUE)->add(COLLADA_TYPE_PARAM))->setValue(it->valuesid.c_str());
        }
    }

private:
    static urdf::Vector3 _poseMult(const urdf::Pose& p, const urdf::Vector3& v)
    {
        double ww = 2 * p.rotation.x * p.rotation.x;
        double wx = 2 * p.rotation.x * p.rotation.y;
        double wy = 2 * p.rotation.x * p.rotation.z;
        double wz = 2 * p.rotation.x * p.rotation.w;
        double xx = 2 * p.rotation.y * p.rotation.y;
        double xy = 2 * p.rotation.y * p.rotation.z;
        double xz = 2 * p.rotation.y * p.rotation.w;
        double yy = 2 * p.rotation.z * p.rotation.z;
        double yz = 2 * p.rotation.z * p.rotation.w;
        urdf::Vector3 vnew;
        vnew.x = (1-xx-yy) * v.x + (wx-yz) * v.y + (wy+xz)*v.z + p.position.x;
        vnew.y = (wx+yz) * v.x + (1-ww-yy) * v.y + (xy-wz)*v.z + p.position.y;
        vnew.z = (wy-xz) * v.x + (xy+wz) * v.y + (1-ww-xx)*v.z + p.position.z;
        return vnew;
    }

    static urdf::Pose _poseInverse(const urdf::Pose& p)
    {
        urdf::Pose pinv;
        pinv.rotation.x = -p.rotation.x;
        pinv.rotation.y = -p.rotation.y;
        pinv.rotation.z = -p.rotation.z;
        pinv.rotation.w = p.rotation.w;
        urdf::Vector3 t = _poseMult(pinv,p.position);
        pinv.position.x = -t.x;
        pinv.position.y = -t.y;
        pinv.position.z = -t.z;
        return pinv;
    }

    int _writeoptions;

    const urdf::Model&              _robot;
    boost::shared_ptr<DAE>          _collada;
    domCOLLADA*                     _dom;
    domCOLLADA::domSceneRef         _globalscene;

    domLibrary_visual_scenesRef _visualScenesLib;
    domLibrary_kinematics_scenesRef _kinematicsScenesLib;
    domLibrary_kinematics_modelsRef _kinematicsModelsLib;
    domLibrary_articulated_systemsRef _articulatedSystemsLib;
    domLibrary_physics_scenesRef _physicsScenesLib;
    domLibrary_materialsRef _materialsLib;
    domLibrary_effectsRef _effectsLib;
    domLibrary_geometriesRef _geometriesLib;
    domTechniqueRef _sensorsLib;///< custom sensors library
    SCENE _scene;

    boost::shared_ptr<instance_kinematics_model_output> _ikmout;
    boost::shared_ptr<instance_articulated_system_output> _iasout;
    std::map< boost::shared_ptr<const urdf::Joint>, int > _mapjointindices;
    std::map< boost::shared_ptr<const urdf::Link>, int > _maplinkindices;
    std::map< boost::shared_ptr<const urdf::Material>, int > _mapmaterialindices;
    Assimp::Importer _importer;
};

ColladaUrdfException::ColladaUrdfException(std::string const& what)
    : std::runtime_error(what)
{
}

bool colladaFromUrdfModel(const urdf::Model& robot_model, boost::shared_ptr<DAE>& dom, int writeoptions)
{
    ColladaWriter writer(robot_model,writeoptions);
    dom = writer.convert();
    return dom != boost::shared_ptr<DAE>();
}

bool colladaToFile(boost::shared_ptr<DAE> dom, string const& file) {
	daeString uri = dom->getDoc(0)->getDocumentURI()->getURI();
	return dom->writeTo(uri, file);
}

}
