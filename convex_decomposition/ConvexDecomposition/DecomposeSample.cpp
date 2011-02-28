#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <float.h>

#include <vector>

#include "./ConvexDecomposition/ConvexDecomposition.h"
#include "./ConvexDecomposition/cd_wavefront.h"

using namespace ConvexDecomposition;

// Test application to demonstrate how to use the ConvexDecomposition system.

typedef std::vector< ConvexResult * > ConvexResultVector;

static const char * fstring(float v)
{
	static char data[64*16];
	static int  index=0;

	char *ret = &data[index*64];
	index++;
	if (index == 16 ) index = 0;

	if ( v == FLT_MIN ) return "-INF";  // collada notation for FLT_MIN and FLT_MAX
	if ( v == FLT_MAX ) return "INF";

	if ( v == 1 )
	{
		strcpy(ret,"1");
	}
	else if ( v == 0 )
	{
		strcpy(ret,"0");
	}
	else if ( v == - 1 )
	{
		strcpy(ret,"-1");
	}
	else
	{
		sprintf(ret,"%.9f", v );
		const char *dot = strstr(ret,".");
		if ( dot )
		{
			int len = (int)strlen(ret);
		  char *foo = &ret[len-1];
			while ( *foo == '0' ) foo--;
			if ( *foo == '.' )
				*foo = 0;
			else
				foo[1] = 0;
		}
	}

	return ret;
}

class CBuilder : public ConvexDecompInterface
{
public:

  CBuilder(const char *fname,const DecompDesc &d)
  {

  	strcpy(mBaseName,fname);
  	char *dot = strstr(mBaseName,".");
  	if ( dot ) *dot = 0;

		sprintf(mObjName,"%s_convex.obj", mBaseName );

  	mBaseCount = 0;
  	mHullCount = 0;
  	mSkinWidth = (float)d.mSkinWidth;

  	mFph = fopen(mObjName,"wb");


		printf("########################################################################\r\n");
  	printf("# Perfomring approximate convex decomposition for triangle mesh %s\r\n", fname );
  	printf("# Input Mesh has %d vertices and %d triangles.\r\n", d.mVcount, d.mTcount );
  	printf("# Recursion depth: %d\r\n", d.mDepth );
  	printf("# Concavity Threshold Percentage: %0.2f\r\n", d.mCpercent );
  	printf("# Hull Merge Volume Percentage: %0.2f\r\n", d.mPpercent );
  	printf("# Maximum output vertices per hull: %d\r\n", d.mMaxVertices );
  	printf("# Hull Skin Width: %0.2f\r\n", d.mSkinWidth );
		printf("########################################################################\r\n");


  	if ( mFph )
  	{
			fprintf(mFph,"########################################################################\r\n");
  		fprintf(mFph,"# Approximate convex decomposition for triangle mesh %s\r\n", fname );
  		fprintf(mFph,"# Input Mesh has %d vertices and %d triangles.\r\n", d.mVcount, d.mTcount );
  		fprintf(mFph,"# Recursion depth: %d\r\n", d.mDepth );
  		fprintf(mFph,"# Concavity Threshold Percentage: %0.2f\r\n", d.mCpercent );
  		fprintf(mFph,"# Hull Merge Volume Percentage: %0.2f\r\n", d.mPpercent );
  		fprintf(mFph,"# Maximum output vertices per hull: %d\r\n", d.mMaxVertices );
  		fprintf(mFph,"# Hull Skin Width: %0.2f\r\n", d.mSkinWidth );
			fprintf(mFph,"########################################################################\r\n");

		  printf("Opened Wavefront file %s for output.\r\n", mObjName );
  	}
  	else
  	{
  		printf("Failed to open output file %s\r\n", mObjName );
  	}
  }

	~CBuilder(void)
	{
		if ( mFph )
		{
			fclose(mFph);
		}
    for (unsigned int i=0; i<mHulls.size(); i++)
    {
    	ConvexResult *cr = mHulls[i];
    	delete cr;
    }
	}

  virtual void ConvexDecompResult(ConvexResult &result)
  {
  	// we have a hull...
  	mHullCount++;

  	printf("Received hull %d  HullVolume(%0.3f)\r\n", mHullCount, result.mHullVolume );

		if ( mFph )
		{

			ConvexResult *cr = new ConvexResult(result);
			mHulls.push_back(cr);

			fprintf(mFph,"########################################################################\r\n");
			fprintf(mFph,"## Hull Piece %d with %d vertices and %d triangles.\r\n", mHullCount, result.mHullVcount, result.mHullTcount );
			fprintf(mFph,"########################################################################\r\n");

			for (unsigned int i=0; i<result.mHullVcount; i++)
			{
				const double *p = &result.mHullVertices[i*3];
				fprintf(mFph,"v %0.9f %0.9f %0.9f\r\n", (float)p[0], (float)p[1], (float)p[2] );
			}

			if ( 1 )
			{
				const unsigned int *src = result.mHullIndices;
				for (unsigned int i=0; i<result.mHullTcount; i++)
				{
					unsigned int i1 = *src++;
					unsigned int i2 = *src++;
					unsigned int i3 = *src++;

          i1+=mBaseCount;
          i2+=mBaseCount;
          i3+=mBaseCount;

				 fprintf(mFph,"f %d %d %d\r\n", i1+1, i2+1, i3+1 );
				}
			}
			mBaseCount+=result.mHullVcount; // advance the 'base index' counter.
		}

  }

  void saveCOLLADA(FILE *fph,unsigned int index,ConvexResult *cr)
  {

    fprintf(fph,"    <geometry id=\"%s_%d-Mesh\" name=\"%s_%d-Mesh\">\r\n", mBaseName, index, mBaseName, index);
    fprintf(fph,"      <convex_mesh>\r\n");
    fprintf(fph,"        <source id=\"%s_%d-Position\">\r\n", mBaseName, index);
    fprintf(fph,"          <float_array count=\"%d\" id=\"%s_%d-Position-array\">\r\n", cr->mHullVcount*3, mBaseName, index);
    fprintf(fph,"            ");
    for (unsigned int i=0; i<cr->mHullVcount; i++)
    {
    	const double *p = &cr->mHullVertices[i*3];
    	fprintf(fph,"%s %s %s  ", fstring((float)p[0]), fstring((float)p[1]), fstring((float)p[2]) );
			if ( ((i+1)&3) == 0 && (i+1) < cr->mHullVcount )
    	{
    		fprintf(fph,"\r\n");
    		fprintf(fph,"            ");
    	}
    }
    fprintf(fph,"\r\n");
    fprintf(fph,"          </float_array>\r\n");
    fprintf(fph,"          <technique_common>\r\n");
    fprintf(fph,"            <accessor count=\"%d\" source=\"#%s_%d-Position-array\" stride=\"3\">\r\n", cr->mHullVcount, mBaseName, index );
    fprintf(fph,"              <param name=\"X\" type=\"float\"/>\r\n");
    fprintf(fph,"              <param name=\"Y\" type=\"float\"/>\r\n");
    fprintf(fph,"              <param name=\"Z\" type=\"float\"/>\r\n");
    fprintf(fph,"            </accessor>\r\n");
    fprintf(fph,"    		   </technique_common>\r\n");
    fprintf(fph,"				 </source>\r\n");
    fprintf(fph,"   		 <vertices id=\"%s_%d-Vertex\">\r\n", mBaseName, index);
    fprintf(fph,"					<input semantic=\"POSITION\" source=\"#%s_%d-Position\"/>\r\n", mBaseName, index);
    fprintf(fph,"				</vertices>\r\n");
    fprintf(fph,"				<triangles material=\"Material\" count=\"%d\">\r\n", cr->mHullTcount );
    fprintf(fph,"					<input offset=\"0\" semantic=\"VERTEX\" source=\"#%s_%d-Vertex\"/>\r\n", mBaseName, index);
    fprintf(fph,"           <p>\r\n");
    fprintf(fph,"             ");
    for (unsigned int i=0; i<cr->mHullTcount; i++)
    {
    	unsigned int *tri = &cr->mHullIndices[i*3];
    	fprintf(fph,"%d %d %d  ", tri[0], tri[1], tri[2] );
			if ( ((i+1)&3) == 0 && (i+1) < cr->mHullTcount )
    	{
    		fprintf(fph,"\r\n");
    		fprintf(fph,"             ");
    	}
    }
    fprintf(fph,"\r\n");
    fprintf(fph,"           </p>\r\n");
    fprintf(fph,"				</triangles>\r\n");
    fprintf(fph,"      </convex_mesh>\r\n");
    fprintf(fph,"		</geometry>\r\n");

  }

	void saveCOLLADA(void)
	{
  	char scratch[512];
		sprintf(scratch,"%s.dae", mBaseName );
		FILE *fph = fopen(scratch,"wb");

  	if ( fph )
  	{
  		printf("Saving convex decomposition of %d hulls to COLLADA file '%s'\r\n", (int)mHulls.size(), scratch );

      fprintf(fph,"<?xml version=\"1.0\" encoding=\"utf-8\"?>\r\n");
      fprintf(fph,"<COLLADA version=\"1.4.0\" xmlns=\"http://www.collada.org/2005/11/COLLADASchema\">\r\n");
      fprintf(fph,"	<asset>\r\n");
      fprintf(fph,"		<contributor>\r\n");
      fprintf(fph,"			<author>NxuStream2 converter - http://www.ageia.com</author>\r\n");
      fprintf(fph,"			<authoring_tool>PhysX Rocket, PhysX Viewer, or CreateDynamics</authoring_tool>\r\n");
      fprintf(fph,"			<comments>questions to: jratcliff@ageia.com</comments>\r\n");
      fprintf(fph,"			<copyright></copyright>\r\n");
      fprintf(fph,"			<source_data>chair_gothic_tilted</source_data>\r\n");
      fprintf(fph,"		</contributor>\r\n");
      fprintf(fph,"		<unit meter=\"1\" name=\"meter\"/>\r\n");
      fprintf(fph,"		<up_axis>Y_UP</up_axis>\r\n");
      fprintf(fph,"	</asset>\r\n");
      fprintf(fph,"   <library_materials>\r\n");
      fprintf(fph,"      <material id=\"Material\" name=\"Material\">\r\n");
      fprintf(fph,"         <instance_effect url=\"#Material-fx\"></instance_effect>\r\n");
      fprintf(fph,"      </material>\r\n");
      fprintf(fph,"   </library_materials>\r\n");
      fprintf(fph,"   <library_effects>\r\n");
      fprintf(fph,"      <effect id=\"Material-fx\" name=\"Material\">\r\n");
      fprintf(fph,"         <profile_COMMON>\r\n");
      fprintf(fph,"            <technique id=\"Material-fx-COMMON\" sid=\"COMMON\">\r\n");
      fprintf(fph,"               <phong>\r\n");
      fprintf(fph,"                  <ambient>\r\n");
      fprintf(fph,"                     <color>0.803922 0.588235 0.92549 1</color>\r\n");
      fprintf(fph,"                  </ambient>\r\n");
      fprintf(fph,"                  <diffuse>\r\n");
      fprintf(fph,"                     <color>0.803922 0.588235 0.92549 1</color>\r\n");
      fprintf(fph,"                  </diffuse>\r\n");
      fprintf(fph,"                  <specular>\r\n");
      fprintf(fph,"                     <color>0.631373 0.631373 0.631373 1</color>\r\n");
      fprintf(fph,"                  </specular>\r\n");
      fprintf(fph,"                  <shininess>\r\n");
      fprintf(fph,"                     <float>1</float>\r\n");
      fprintf(fph,"                  </shininess>\r\n");
      fprintf(fph,"                  <reflective>\r\n");
      fprintf(fph,"                     <color>0 0 0 1</color>\r\n");
      fprintf(fph,"                  </reflective>\r\n");
      fprintf(fph,"                  <transparent>\r\n");
      fprintf(fph,"                     <color>1 1 1 1</color>\r\n");
      fprintf(fph,"                  </transparent>\r\n");
      fprintf(fph,"                  <transparency>\r\n");
      fprintf(fph,"                     <float>0</float>\r\n");
      fprintf(fph,"                  </transparency>\r\n");
      fprintf(fph,"               </phong>\r\n");
      fprintf(fph,"           </technique>\r\n");
      fprintf(fph,"         </profile_COMMON>\r\n");
      fprintf(fph,"      </effect>\r\n");
      fprintf(fph,"   </library_effects>\r\n");
      fprintf(fph,"  <library_geometries>\r\n");

      for (unsigned int i=0; i<mHulls.size(); i++)
      {
      	ConvexResult *cr = mHulls[i];
      	saveCOLLADA(fph,i,cr);
      }


      fprintf(fph,"  </library_geometries>\r\n");
      fprintf(fph,"  <library_visual_scenes>\r\n");
      fprintf(fph,"	  <visual_scene id=\"Scene0-Visual\" name=\"Scene0-Visual\">\r\n");
      fprintf(fph,"      <node id=\"%s-Node\" name=\"%s\" type=\"NODE\">\r\n", mBaseName, mBaseName );
      fprintf(fph,"        <translate>0 0 0</translate>\r\n");
      fprintf(fph,"        <rotate>1 0 0  0</rotate>\r\n");
      fprintf(fph,"      </node>\r\n");
      fprintf(fph,"  	  </visual_scene>\r\n");
      fprintf(fph,"  </library_visual_scenes>\r\n");
      fprintf(fph,"  <library_physics_materials>\r\n");
      fprintf(fph,"    <physics_material id=\"pmat0_0-PhysicsMaterial\" name=\"pmat0_0-PhysicsMaterial\">\r\n");
      fprintf(fph,"      <technique_common>\r\n");
      fprintf(fph,"        <dynamic_friction>0.5</dynamic_friction>\r\n");
      fprintf(fph,"        <restitution>0</restitution>\r\n");
      fprintf(fph,"        <static_friction>0.5</static_friction>\r\n");
      fprintf(fph,"      </technique_common>\r\n");
      fprintf(fph,"    </physics_material>\r\n");
      fprintf(fph,"  </library_physics_materials>\r\n");
      fprintf(fph,"  <library_physics_models>\r\n");
      fprintf(fph,"    <physics_model id=\"Scene0-PhysicsModel\">\r\n");
      fprintf(fph,"      <rigid_body sid=\"%s-RigidBody\" name=\"%s\">\r\n", mBaseName, mBaseName);
      fprintf(fph,"        <technique_common>\r\n");
      fprintf(fph,"          <instance_physics_material url=\"#pmat0_0-PhysicsMaterial\"/>\r\n");

      for (unsigned int i=0; i<mHulls.size(); i++)
      {
      	ConvexResult *ch = mHulls[i];
        fprintf(fph,"         <shape>\r\n");
        fprintf(fph,"            <translate>0 0 0</translate>\r\n");
        fprintf(fph,"            <rotate>1 0 0  0</rotate>\r\n");
        fprintf(fph,"            <instance_physics_material url=\"#pmat0_0-PhysicsMaterial\"/>\r\n");
        fprintf(fph,"            <density>1</density>\r\n");
        fprintf(fph,"            <extra>\r\n");
        fprintf(fph,"              <technique profile=\"PhysX\">\r\n");
        fprintf(fph,"               <skinWidth>%s</skinWidth>\r\n", fstring( mSkinWidth ) );
        fprintf(fph,"              </technique>\r\n");
        fprintf(fph,"            </extra>\r\n");
        fprintf(fph,"            <instance_geometry url=\"%s_%d-Mesh\"/>\r\n", mBaseName, i);
        fprintf(fph,"         </shape>\r\n");
      }

      fprintf(fph,"        <dynamic>true</dynamic>\r\n");
      fprintf(fph,"        <mass>1</mass>\r\n");
      fprintf(fph,"		   	</technique_common>\r\n");
      fprintf(fph,"        <extra>\r\n");
      fprintf(fph,"          <technique profile=\"PhysX\">\r\n");
      fprintf(fph,"            <extra>\r\n");
      fprintf(fph,"              <technique profile=\"PhysX\">\r\n");
      fprintf(fph,"                <linearDamping>0</linearDamping>\r\n");
      fprintf(fph,"                <angularDamping>0</angularDamping>\r\n");
      fprintf(fph,"                <solverIterationCount>32</solverIterationCount>\r\n");
      fprintf(fph,"              </technique>\r\n");
      fprintf(fph,"            </extra>\r\n");
      fprintf(fph,"            <disable_collision>false</disable_collision>\r\n");
      fprintf(fph,"          </technique>\r\n");
      fprintf(fph,"        </extra>\r\n");
      fprintf(fph,"  	  </rigid_body>\r\n");
      fprintf(fph,"    </physics_model>\r\n");
      fprintf(fph,"  </library_physics_models>\r\n");
      fprintf(fph,"  <library_physics_scenes>\r\n");
      fprintf(fph,"    <physics_scene id=\"Scene0-Physics\">\r\n");
      fprintf(fph,"      <instance_physics_model url=\"#Scene0-PhysicsModel\">\r\n");
      fprintf(fph,"        <instance_rigid_body target=\"#%s-Node\" body=\"%s-RigidBody\"/>\r\n", mBaseName, mBaseName );
      fprintf(fph,"      <extra>\r\n");
      fprintf(fph,"        <technique profile=\"PhysX\">\r\n");
      fprintf(fph,"        </technique>\r\n");
      fprintf(fph,"      </extra>\r\n");
      fprintf(fph,"      </instance_physics_model>\r\n");
      fprintf(fph,"      <technique_common>\r\n");
      fprintf(fph,"        <gravity>0 -9.800000191 0</gravity>\r\n");
      fprintf(fph,"      </technique_common>\r\n");
      fprintf(fph,"    </physics_scene>\r\n");
      fprintf(fph,"  </library_physics_scenes>\r\n");
      fprintf(fph,"  <scene>\r\n");
      fprintf(fph,"    <instance_visual_scene url=\"#Scene0-Visual\"/>\r\n");
      fprintf(fph,"    <instance_physics_scene url=\"#Scene0-Physics\"/>\r\n");
      fprintf(fph,"  </scene>\r\n");
      fprintf(fph,"</COLLADA>\r\n");


      fclose(fph);
  	}
  	else
  	{
  		printf("Failed to open file '%s' for write access.\r\n", scratch );
  	}
	}

	void saveXML(FILE *fph,unsigned int index,ConvexResult *cr)
	{

    fprintf(fph,"  <NxConvexMeshDesc id=\"%s_%d\">\r\n", mBaseName, index);
    fprintf(fph,"    <points>\r\n");
		fprintf(fph,"      ");
    for (unsigned int i=0; i<cr->mHullVcount; i++)
    {
    	const double *p = &cr->mHullVertices[i*3];
    	fprintf(fph,"%s %s %s  ", fstring((float)p[0]), fstring((float)p[1]), fstring((float)p[2]) );
			if ( ((i+1)&3) == 0 && (i+1) < cr->mHullVcount )
    	{
    		fprintf(fph,"\r\n");
    		fprintf(fph,"      ");
    	}
    }
    fprintf(fph,"\r\n");
    fprintf(fph,"    </points>\r\n");
    fprintf(fph,"    <triangles>\r\n");
		fprintf(fph,"      ");
    for (unsigned int i=0; i<cr->mHullTcount; i++)
    {
    	unsigned int *tri = &cr->mHullIndices[i*3];
    	fprintf(fph,"%d %d %d  ", tri[0], tri[1], tri[2] );
			if ( ((i+1)&3) == 0 && (i+1) < cr->mHullTcount )
    	{
    		fprintf(fph,"\r\n");
    		fprintf(fph,"      ");
    	}
    }
    fprintf(fph,"\r\n");
    fprintf(fph,"    </triangles>\r\n");
    fprintf(fph,"  </NxConvexMeshDesc>\r\n");

	}

  void saveNxuStream(void)
  {
  	char scratch[512];
		sprintf(scratch,"%s.xml", mBaseName );
		FILE *fph = fopen(scratch,"wb");

  	if ( fph )
  	{
  		printf("Saving convex decomposition of %d hulls to NxuStream file '%s'\r\n", mHulls.size(), scratch );

      fprintf(fph,"<?xml version=\"1.0\" encoding=\"utf-8\"?>\r\n");
      fprintf(fph,"  <NXUSTREAM2>\r\n");
      fprintf(fph,"    <NxuPhysicsCollection id=\"beshon\" sdkVersion=\"244\" nxuVersion=\"100\">\r\n");
      fprintf(fph,"      <NxPhysicsSDKDesc id=\"SDK\">\r\n");
      fprintf(fph,"        <hwPageSize>65536</hwPageSize>\r\n");
      fprintf(fph,"        <hwPageMax>256</hwPageMax>\r\n");
      fprintf(fph,"        <hwConvexMax>2048</hwConvexMax>\r\n");
      fprintf(fph,"      </NxPhysicsSDKDesc>\r\n");

			for (unsigned int i=0; i<mHulls.size(); i++)
			{
				saveXML(fph, i, mHulls[i] );
			}


      fprintf(fph,"  <NxSceneDesc id=\"%s\">\r\n", mBaseName);
      fprintf(fph,"        <filterBool>false</filterBool>\r\n");
      fprintf(fph,"        <filterOp0>NX_FILTEROP_AND</filterOp0>\r\n");
      fprintf(fph,"        <filterOp1>NX_FILTEROP_AND</filterOp1>\r\n");
      fprintf(fph,"        <filterOp2>NX_FILTEROP_AND</filterOp2>\r\n");
      fprintf(fph,"        <NxGroupsMask id=\"groupMask0\" bits0=\"0\" bits1=\"0\" bits2=\"0\" bits3=\"0\"/>\r\n");
      fprintf(fph,"        <NxGroupsMask id=\"groupMask1\" bits0=\"0\" bits1=\"0\" bits2=\"0\" bits3=\"0\"/>\r\n");
      fprintf(fph,"        <gravity>0 -9.800000191 0</gravity>\r\n");
      fprintf(fph,"        <maxTimestep>0.016666668</maxTimestep>\r\n");
      fprintf(fph,"        <maxIter>8</maxIter>\r\n");
      fprintf(fph,"        <timeStepMethod>NX_TIMESTEP_FIXED</timeStepMethod>\r\n");
      fprintf(fph,"        <maxBounds>FLT_MIN FLT_MIN FLT_MIN  FLT_MAX FLT_MAX FLT_MAX</maxBounds>\r\n");
      fprintf(fph,"        <NxSceneLimits id=\"limits\">\r\n");
      fprintf(fph,"          <maxNbActors>0</maxNbActors>\r\n");
      fprintf(fph,"          <maxNbBodies>0</maxNbBodies>\r\n");
      fprintf(fph,"          <maxNbStaticShapes>0</maxNbStaticShapes>\r\n");
      fprintf(fph,"          <maxNbDynamicShapes>0</maxNbDynamicShapes>\r\n");
      fprintf(fph,"          <maxNbJoints>0</maxNbJoints>\r\n");
      fprintf(fph,"        </NxSceneLimits>\r\n");
      fprintf(fph,"        <simType>NX_SIMULATION_SW</simType>\r\n");
      fprintf(fph,"        <groundPlane>true</groundPlane>\r\n");
      fprintf(fph,"        <boundsPlanes>false</boundsPlanes>\r\n");
      fprintf(fph,"        <NxSceneFlags id=\"flags\">\r\n");
      fprintf(fph,"          <NX_SF_DISABLE_SSE>false</NX_SF_DISABLE_SSE>\r\n");
      fprintf(fph,"          <NX_SF_DISABLE_COLLISIONS>false</NX_SF_DISABLE_COLLISIONS>\r\n");
      fprintf(fph,"          <NX_SF_SIMULATE_SEPARATE_THREAD>true</NX_SF_SIMULATE_SEPARATE_THREAD>\r\n");
      fprintf(fph,"          <NX_SF_ENABLE_MULTITHREAD>false</NX_SF_ENABLE_MULTITHREAD>\r\n");
      fprintf(fph,"        </NxSceneFlags>\r\n");
      fprintf(fph,"        <internalThreadCount>0</internalThreadCount>\r\n");
      fprintf(fph,"        <backgroundThreadCount>0</backgroundThreadCount>\r\n");
      fprintf(fph,"        <threadMask>1431655764</threadMask>\r\n");
      fprintf(fph,"        <backgroundThreadMask>1431655764</backgroundThreadMask>\r\n");
      fprintf(fph,"        <NxMaterialDesc id=\"Material_0\" userProperties=\"\" hasSpring=\"false\" materialIndex=\"0\">\r\n");
      fprintf(fph,"          <dynamicFriction>0.5</dynamicFriction>\r\n");
      fprintf(fph,"          <staticFriction>0.5</staticFriction>\r\n");
      fprintf(fph,"          <restitution>0</restitution>\r\n");
      fprintf(fph,"          <dynamicFrictionV>0</dynamicFrictionV>\r\n");
      fprintf(fph,"          <staticFrictionV>0</staticFrictionV>\r\n");
      fprintf(fph,"          <frictionCombineMode>NX_CM_AVERAGE</frictionCombineMode>\r\n");
      fprintf(fph,"          <restitutionCombineMode>NX_CM_AVERAGE</restitutionCombineMode>\r\n");
      fprintf(fph,"          <dirOfAnisotropy>1 0 0</dirOfAnisotropy>\r\n");
      fprintf(fph,"          <NxMaterialFlag id=\"flags\">\r\n");
      fprintf(fph,"            <NX_MF_ANISOTROPIC>false</NX_MF_ANISOTROPIC>\r\n");
      fprintf(fph,"            <NX_MF_DISABLE_FRICTION>false</NX_MF_DISABLE_FRICTION>\r\n");
      fprintf(fph,"            <NX_MF_DISABLE_STRONG_FRICTION>false</NX_MF_DISABLE_STRONG_FRICTION>\r\n");
      fprintf(fph,"          </NxMaterialFlag>\r\n");
      fprintf(fph,"        </NxMaterialDesc>\r\n");
      fprintf(fph,"    <NxActorDesc id=\"%s\" userProperties=\"\" hasBody=\"true\" name=\"%s\">\r\n", mBaseName, mBaseName);
      fprintf(fph,"      <globalPose>1 0 0    0 1 0    0 0 1    0 0 0 </globalPose>\r\n");
      fprintf(fph,"			<NxBodyDesc>\r\n");
      fprintf(fph,"        <mass>1</mass>\r\n");
      fprintf(fph,"        <linearDamping>0</linearDamping>\r\n");
      fprintf(fph,"        <angularDamping>0</angularDamping>\r\n");
      fprintf(fph,"        <solverIterationCount>32</solverIterationCount>\r\n");
      fprintf(fph,"        <NxBodyFlag id=\"flags\">\r\n");
      fprintf(fph,"          <NX_BF_POSE_SLEEP_TEST>true</NX_BF_POSE_SLEEP_TEST>\r\n");
      fprintf(fph,"          <NX_AF_DISABLE_COLLISION>false</NX_AF_DISABLE_COLLISION>\r\n");
      fprintf(fph,"        </NxBodyFlag>\r\n");
      fprintf(fph,"      </NxBodyDesc>\r\n");
      fprintf(fph,"      <name>Bip01 Pelvis</name>\r\n");
      for (unsigned int i=0; i<mHulls.size(); i++)
      {
        fprintf(fph,"      <NxConvexShapeDesc id=\"Shape_%d\" meshData=\"%s_%d\">\r\n", i, mBaseName, i);
        fprintf(fph,"      <NxShapeDesc>\r\n");
        fprintf(fph,"        <localPose>1 0 0    0 1 0    0 0 1    0 0 0 </localPose>\r\n");
        fprintf(fph,"        <skinWidth>0</skinWidth>\r\n");
        fprintf(fph,"      </NxShapeDesc>\r\n");
        fprintf(fph,"      </NxConvexShapeDesc>\r\n");
      }
      fprintf(fph,"    </NxActorDesc>\r\n");

      fprintf(fph,"  </NxSceneDesc>\r\n");
      fprintf(fph,"	 <NxSceneInstance id=\"%s\">\r\n", mBaseName);
      fprintf(fph,"	 	 <sceneName>beshon</sceneName>\r\n");
      fprintf(fph,"	 	 <NxuUserProperties></NxuUserProperties>\r\n");
      fprintf(fph,"	 	 <rootNode>1	0	0	0	1	0	0	0	1	0	0 0</rootNode>\r\n");
      fprintf(fph,"	 	 <newScene>false</newScene>\r\n");
      fprintf(fph,"	 	 <ignorePlane>true</ignorePlane>\r\n");
      fprintf(fph,"	 	 <numSceneInstances>0</numSceneInstances>\r\n");
      fprintf(fph,"	 </NxSceneInstance>\r\n");
      fprintf(fph,"  </NxuPhysicsCollection>\r\n");
      fprintf(fph,"</NXUSTREAM2>\r\n");

    }
    else
    {
    	printf("Failed to open file '%s' for write access.\r\n", scratch );
    }
  }

  float             mSkinWidth;
	unsigned int      mHullCount;
  FILE              *mFph;
  unsigned int       mBaseCount;
  char               mObjName[512];
  char               mBaseName[512];
  ConvexResultVector mHulls;

};


int main(int argc,const char **argv)
{
	if ( argc < 2 )
	{
		printf("Usage: Test <meshanme.obj> (options)\r\n");
		printf("\r\n");
		printf("Options:\r\n");
		printf("\r\n");
		printf("            -d<depth>       : How deep to recursively split. Values of 3-7 are reasonable.\r\n");
		printf("            -c<percent>     : Percentage of concavity to keep splitting. 0-20% is reasonable.\r\n");
		printf("            -p<percent>     : Percentage of volume delta to collapse hulls.  0-30% is reasonable.\r\n");
		printf("            -v<maxverts>    : Maximum number of vertices in the output hull.  Default is 32.\r\n");
		printf("            -s<skinwidth>   : Skin Width inflation.  Default is 0.\r\n");
	}
	else
	{
		unsigned int depth = 5;
		float cpercent     = 5;
		float ppercent     = 5;
		unsigned int maxv  = 32;
		float skinWidth    = 0;

		// process command line switches.
		for (int i=2; i<argc; i++)
		{
			const char *o = argv[i];

			if ( strncmp(o,"-d",2) == 0 )
			{
				depth = (unsigned int) atoi( &o[2] );
				if ( depth < 0 || depth > 10 )
				{
					depth = 5;
					printf("Invalid depth value in switch, defaulting to 5.\r\n");
				}
			}

			if ( strncmp(o,"-c",2) == 0 )
			{
				cpercent = (float) atof( &o[2] );
				if ( cpercent < 0 || cpercent > 100 )
				{
					cpercent = 5;
					printf("Invalid concavity percentage in switch, defaulting to 5.\r\n");
				}
			}

			if ( strncmp(o,"-p",2) == 0 )
			{
				ppercent = (float) atof( &o[2] );
				if ( ppercent < 0 || ppercent > 100 )
				{
					ppercent = 5;
					printf("Invalid hull merge percentage in switch, defaulting to 5.\r\n");
				}
			}

			if ( strncmp(o,"-v",2) == 0 )
			{
				maxv = (unsigned int) atoi( &o[2] );
				if ( maxv < 8 || maxv > 256 )
				{
					maxv = 32;
					printf("Invalid max vertices in switch, defaulting to 32.\r\n");
				}
			}

			if ( strncmp(o,"-s",2) == 0 )
			{
				skinWidth = (float) atof( &o[2] );
				if ( skinWidth < 0 || skinWidth > 0.1f )
				{
					skinWidth = 0;
					printf("Invalid skinWidth in switch, defaulting to 0.\r\n");
				}
			}

		}

		WavefrontObj wo;

    unsigned int tcount = wo.loadObj(argv[1]);

    if ( tcount )
    {


    	DecompDesc d;

      d.mVcount       =	wo.mVertexCount;
      d.mVertices     = wo.mVertices;
      d.mTcount       = wo.mTriCount;
      d.mIndices      = (unsigned int *)wo.mIndices;
      d.mDepth        = depth;
      d.mCpercent     = cpercent;
      d.mPpercent     = ppercent;
      d.mMaxVertices  = maxv;
      d.mSkinWidth    = skinWidth;


    	CBuilder   cb(argv[1],d); // receives the answers and writes out a wavefront OBJ file.

      d.mCallback     = &cb;

			unsigned int count = performConvexDecomposition(d);

			if ( count )
			{
				printf("Input triangle mesh with %d triangles produced %d output hulls.\r\n", d.mTcount, count );

			  cb.saveNxuStream();  // save results in NxuStream format.
			  cb.saveCOLLADA();    // save results in COLLADA physics 1.4.1 format.

			}
			else
			{
				printf("Failed to produce any convex hulls.\r\n");
			}

    }
    else
    {
    	// sorry..no
    	printf("Sorry unable to load file '%s'\r\n", argv[1] );
    }

	}

}
