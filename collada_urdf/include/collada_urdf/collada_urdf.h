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

/* Authors: Tim Field, Rosen Diankov */

#ifndef COLLADA_URDF_COLLADA_URDF_H
#define COLLADA_URDF_COLLADA_URDF_H

#include <string>
#include <boost/shared_ptr.hpp>
#include <dae.h>
#include "urdf/model.h"

namespace collada_urdf {

class ColladaUrdfException : public std::runtime_error
{
public:
    ColladaUrdfException(std::string const& what);
};

enum WriteOptions
{
    WO_IgnoreCollisionGeometry = 1, ///< if set, will use only the visual geometry
};

/** Construct a COLLADA DOM from an URDF model
    
    \param robot_model The initialized robot model
    \param dom The resulting COLLADA DOM
    \param writeoptions A combination of \ref WriteOptions 
    
    \return true on success, false on failure
 */
bool colladaFromUrdfModel(const urdf::Model& robot_model, boost::shared_ptr<DAE>& dom, int writeoptions=0);

/** Write a COLLADA DOM to a file
 * \param dom COLLADA DOM to write
 * \param file The filename to write the document to
 * \return true on success, false on failure
 */
bool colladaToFile(boost::shared_ptr<DAE> dom, std::string const& file);

}

#endif
