#ifndef CVMESHBLURDEFORMER_H
#define CVMESHBLURDEFORMER_H


#include <maya/MPlug.h> 
#include <maya/MPoint.h> 
#include <maya/MPointArray.h> 
#include <maya/MMatrix.h> 
#include <maya/MTypeId.h> 
#include <maya/MPlug.h>
#include <maya/MDataBlock.h>
#include <maya/MDataHandle.h>
#include <maya/MArrayDataHandle.h>
#include <maya/MFloatArray.h>
#include <maya/MDoubleArray.h>
#include <maya/MIntArray.h>
#include <maya/MMatrix.h>
#include <maya/MGlobal.h>
#include <maya/MTime.h>
#include <maya/MThreadPool.h>
#include <maya/MVector.h>
#include <maya/MVectorArray.h>

#include <maya/MItGeometry.h>

#include <maya/MPxDeformerNode.h>
#include <maya/MFnEnumAttribute.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MFnMatrixAttribute.h>
#include <maya/MFnMessageAttribute.h>
#include <maya/MFnTypedAttribute.h>
#include <maya/MFnCompoundAttribute.h>
#include <maya/MFnUnitAttribute.h>
#include <maya/MFnGenericAttribute.h>
#include <maya/MFnPointArrayData.h>
#include <maya/MFnDoubleArrayData.h>
#include <maya/MFnIntArrayData.h>
#include <maya/MFnMesh.h>
#include <maya/MFnNurbsSurface.h>
#include <maya/MFnNurbsCurve.h>
#include <maya/MFnSubd.h>
#include <maya/MFnData.h>
#include <array>

struct TaskData
{
    unsigned int numDeformVerts;
    double smearRate;
    double minSmearVelocity;
    double maxSmearVelocity;
    float normalOffset;
    float angleMagnitude;
    MPointArray* pGoal;
    MPointArray* pCurrent;
    MPointArray* pPreviousGoal;
    MPointArray* pDeformedPointsLocal;
    MPointArray* pDeformedPointsWorld;
    MVectorArray* pNormals;
    MFloatArray* pWeights;
    MMatrix localToWorldMatrix;
    MMatrix worldToLocalMatrix;

};

struct ThreadData
{
    unsigned int start;
    unsigned int end;
    unsigned int numTasks;
    TaskData* pData;
};

class cvMeshBlur : public MPxDeformerNode
{
public:
    cvMeshBlur();
    virtual	~cvMeshBlur();

    virtual MStatus deform(MDataBlock& data, MItGeometry& iter, const MMatrix& mat,
                           unsigned int mIndex);

    static  void* creator();
    static  MStatus initialize();
    void CreateThreadData();
    static void CreateTasks(void *data, MThreadRootTask *pRoot);
    static MThreadRetVal ThreadEvaluate(void *pParam);

public:

    static const int taskCount;
    static MTypeId id;
    static MObject aTime;
    static MObject aStartFrame;
    static MObject aMinSmearVelocity;
    static MObject aMaxSmearVelocity;
    static MObject aWorldMatrix;
    static MObject aSmearFrames;
    static MObject aNormalOffset;
    static MObject aAngleMagnitude;

private:
    bool m_initialized;
    MPointArray m_previousPositions;
    MPointArray m_currentPositions;
    MTime m_previousTime;
    TaskData m_taskData;
    std::array<ThreadData, 16> m_threadData;

};

#endif
