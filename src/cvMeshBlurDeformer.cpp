#include "cvMeshBlurDeformer.h"

MTypeId cvMeshBlur::id(0x00115812);
MObject cvMeshBlur::aTime;
MObject cvMeshBlur::aNormalOffset;
MObject cvMeshBlur::aAngleMagnitude;
MObject cvMeshBlur::aStartFrame;
MObject cvMeshBlur::aSmearFrames;
MObject cvMeshBlur::aMinSmearVelocity;
MObject cvMeshBlur::aMaxSmearVelocity;
MObject cvMeshBlur::aWorldMatrix;
const int cvMeshBlur::taskCount = 16;

MStatus cvMeshBlur::initialize()
{
    MFnMatrixAttribute      mAttr;
    MFnNumericAttribute     nAttr;
    MFnUnitAttribute        uAttr;
    MStatus				    status;

    aTime = uAttr.create("time", "time", MFnUnitAttribute::kTime, 0.0);
    addAttribute(aTime);
    attributeAffects(aTime, outputGeom);

    aWorldMatrix = mAttr.create("worldMatrix", "worldMatrix");
    addAttribute(aWorldMatrix);
    attributeAffects(aWorldMatrix, outputGeom);

    aStartFrame = nAttr.create("startFrame", "startFrame", MFnNumericData::kInt, 0, &status);
    nAttr.setKeyable(true);
    addAttribute(aStartFrame);

    aSmearFrames = nAttr.create("smearFrames", "smearFrames", MFnNumericData::kInt, 1, &status);
    nAttr.setMin(1);
    nAttr.setMax(100);
    nAttr.setKeyable(true);
    addAttribute(aSmearFrames);
    attributeAffects(aSmearFrames, outputGeom);

    aNormalOffset = nAttr.create("normalOffset", "normalOffset", MFnNumericData::kFloat, 0.0, &status);
    nAttr.setMin(0.0);
    nAttr.setKeyable(true);
    addAttribute(aNormalOffset);
    attributeAffects(aNormalOffset, outputGeom);

    aAngleMagnitude = nAttr.create("angleMagnitude", "angleMagnitude", MFnNumericData::kFloat, 1.0, &status);
    nAttr.setMin(0.0);
    nAttr.setKeyable(true);
    addAttribute(aAngleMagnitude);
    attributeAffects(aAngleMagnitude, outputGeom);

    aMinSmearVelocity = nAttr.create("minSmearVelocity", "minSmearVelocity", MFnNumericData::kDouble, 0, &status);
    nAttr.setMin(0.0);
    nAttr.setKeyable(true);
    addAttribute(aMinSmearVelocity);
    attributeAffects(aMinSmearVelocity, outputGeom);

    aMaxSmearVelocity = nAttr.create("maxSmearVelocity", "maxSmearVelocity", MFnNumericData::kDouble, 5, &status);
    nAttr.setMin(0.0);
    nAttr.setKeyable(true);
    addAttribute(aMaxSmearVelocity);
    attributeAffects(aMaxSmearVelocity, outputGeom);

    MGlobal::executeCommand("makePaintable -attrType multiFloat -sm deformer cvMeshBlur weights");

    return MS::kSuccess;
}


cvMeshBlur::cvMeshBlur()
{
	m_initialized = false;
	MThreadPool::init();
}

cvMeshBlur::~cvMeshBlur()
{
	MThreadPool::release();
}

void* cvMeshBlur::creator() { return new cvMeshBlur(); }


MStatus cvMeshBlur::deform(MDataBlock& data, MItGeometry& itGeo, const MMatrix& localToWorldMatrix,
                           unsigned int geomIndex) {
	MStatus status;

	MMatrix worldToLocalMatrix = localToWorldMatrix.inverse();

	// Get envelope
	float env = data.inputValue(envelope).asFloat();
	if (env == 0.0f)
	{
		return MS::kSuccess;
	}

    MArrayDataHandle hInput = data.outputArrayValue(input, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status)
    status = hInput.jumpToElement(geomIndex);
    CHECK_MSTATUS_AND_RETURN_IT(status)
    MObject oInputGeom = hInput.outputValue().child(inputGeom).asMesh();
	MFnMesh fnMesh(oInputGeom, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	MTime time = data.inputValue(aTime).asTime();
	MMatrix matrix = data.inputValue(aWorldMatrix).asMatrix();
	int startFrame = data.inputValue(aStartFrame, &status).asInt();
	double minSmearVelocity = data.inputValue(aMinSmearVelocity).asDouble();
	double maxSmearVelocity = data.inputValue(aMaxSmearVelocity).asDouble();
	int smearFrames = data.inputValue(aSmearFrames).asInt();
	float normalOffset = data.inputValue(aNormalOffset, &status).asFloat();
	float angleMagnitude = data.inputValue(aAngleMagnitude, &status).asFloat();

	MPointArray previousGoal, current;
	double difference = time.value() - m_previousTime.value();
	if (!m_initialized)
	{
		m_previousTime = time;
	}

	MPointArray goal;
	itGeo.allPositions(goal);
	if (!m_initialized ||
		difference != 1.0 && difference != 0.0 ||
		time.value() < (double)startFrame)
	{
		m_previousPositions.copy(goal);
		// Put in world space
		for (unsigned int i = 0; i < m_previousPositions.length(); i++)
		{
			m_previousPositions[i] *= localToWorldMatrix;
		}
		m_currentPositions.copy(m_previousPositions);
		m_initialized = true;
	}

	// Get the painted weights and vertex normals
	MFloatArray weights(itGeo.count());
	MVectorArray normals(itGeo.count());
	int i = 0;
	for (itGeo.reset(); !itGeo.isDone(); itGeo.next(), i++)
	{
		weights[i] = weightValue(data, geomIndex, itGeo.index()) * env;
		status = fnMesh.getVertexNormal(itGeo.index(), false, normals[i]);
	}

	if (smearFrames < 1)
	{
		smearFrames = 1;
	}
	double smearRate = 1.0 / (double)smearFrames;
	unsigned int numVerts = goal.length();
	MPointArray deformedPointsLocal(goal);
	MPointArray deformedPointsWorld(m_currentPositions);

	m_taskData.numDeformVerts = numVerts;
	m_taskData.pGoal = &goal;
	m_taskData.pCurrent = &m_currentPositions;
	m_taskData.pPreviousGoal = &m_previousPositions;
	m_taskData.smearRate = smearRate;
	m_taskData.minSmearVelocity = minSmearVelocity;
	m_taskData.maxSmearVelocity = maxSmearVelocity;
	m_taskData.pDeformedPointsLocal = &deformedPointsLocal;
	m_taskData.pDeformedPointsWorld = &deformedPointsWorld;
	m_taskData.pNormals = &normals;
	m_taskData.pWeights = &weights;
	m_taskData.localToWorldMatrix = localToWorldMatrix;
	m_taskData.worldToLocalMatrix = worldToLocalMatrix;
	m_taskData.normalOffset = normalOffset;
	m_taskData.angleMagnitude = angleMagnitude;

	CreateThreadData();
	status = MThreadPool::newParallelRegion(CreateTasks, (void *)&m_threadData[0]);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	status = itGeo.setAllPositions(deformedPointsLocal);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	// Store previous for next calculation
	m_previousPositions = goal;

	// Store current for next calculation
	m_currentPositions = deformedPointsWorld;

	// Store previous time
	m_previousTime = time;

	return status;
}

void cvMeshBlur::CreateThreadData()
{
    // TODO: Reuse thread data
    int taskCount = (int)m_threadData.size();
    unsigned int taskLength = (m_taskData.numDeformVerts + taskCount - 1) / taskCount;
	unsigned int start = 0;
	unsigned int end = taskLength;

	int lastTask = taskCount - 1;
	for (int i = 0; i < taskCount; i++)
	{
		if (i == lastTask)
		{
			end = m_taskData.numDeformVerts;
		}
        m_threadData[i].start = start;
        m_threadData[i].end = end;
        m_threadData[i].numTasks = taskCount;
        m_threadData[i].pData = &m_taskData;

		start += taskLength;
		end += taskLength;
	}
}

void cvMeshBlur::CreateTasks(void* pData, MThreadRootTask* pRoot)
{
    ThreadData* pThreadData = static_cast<ThreadData*>(pData);

	if (pThreadData)
	{
		int taskCount = pThreadData[0].numTasks;
		for (int i = 0; i < taskCount; ++i)
		{
			MThreadPool::createTask(ThreadEvaluate, (void *)&pThreadData[i], pRoot);
		}
		MThreadPool::executeAndJoin(pRoot);
	}
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
Summary:
	Evaluation

Returns:
	0
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
MThreadRetVal cvMeshBlur::ThreadEvaluate(void *pParam)
{
	ThreadData* pThreadData = static_cast<ThreadData*>(pParam);
    TaskData* pData = pThreadData->pData;
    double smearRate = pData->smearRate;
	double minSmearVelocity = pData->minSmearVelocity;
	double maxSmearVelocity = pData->maxSmearVelocity;
	MPointArray& currentPositions = (*(pData->pCurrent));
	MPointArray& goal = (*(pData->pGoal));
	MPointArray& previousGoal = (*(pData->pPreviousGoal));
	MPointArray& deformedPointsLocal = (*(pData->pDeformedPointsLocal));
	MPointArray& deformedPointsWorld = (*(pData->pDeformedPointsWorld));
	MFloatArray& weights = *(pData->pWeights);
	MVectorArray& normals = *(pData->pNormals);
	MMatrix& localToWorldMatrix = pData->localToWorldMatrix;
	MMatrix& worldToLocalMatrix = pData->worldToLocalMatrix;
	float normalOffset = pData->normalOffset;
	float angleMagnitude = pData->angleMagnitude;

	unsigned int taskStart = pThreadData->start;
	unsigned int taskEnd = pThreadData->end;
	double dot, goalVelocity, velocityDelta;
	MVector velocity, velocityUnit;

	for (unsigned int i = taskStart; i < taskEnd; i++)
	{
		if (i >= goal.length())
		{
			continue;
		}
		MPoint ptOrig(goal[i]);
		// Put input points into world space
		goal[i] *= localToWorldMatrix;

		velocity = (goal[i] - currentPositions[i]);
		velocityUnit = velocity.normal();
		goalVelocity = (goal[i] - previousGoal[i]).length();

		if (goalVelocity != 0.0)
		{
			velocityDelta = goalVelocity - minSmearVelocity;
			if (velocityDelta > 0.0)
			{
				if (velocityDelta > maxSmearVelocity)
				{
					velocityDelta = maxSmearVelocity;
				}
				currentPositions[i] = goal[i] + ((currentPositions[i] - goal[i]) * (velocityDelta / goalVelocity));
			}
			else
			{
				// If there is no velocity delta, do not smear
				currentPositions[i] = goal[i];
			}
		}

		dot = velocityUnit * normals[i];
		if (weights[i] == 0.0f || dot >= 0.0 || goalVelocity == 0.0)
		{
			// No need to calculate because the vertex is either
			// 1) Painted 0
			// 2) Facing the velocity vector
			// 3) Not moving
			deformedPointsWorld[i] = goal[i];
			continue;
		}

		currentPositions[i] += (goal[i] - currentPositions[i]) * smearRate;

		dot = -dot;
		dot = (dot + normalOffset) * angleMagnitude;
		if (dot > 1.0)
		{
			dot = 1.0;
		}
		// Scale offset by normal-velocity vector dot product
		currentPositions[i] = ((currentPositions[i] - goal[i]) * dot) + goal[i];
		deformedPointsWorld[i] = currentPositions[i];

		deformedPointsLocal[i] = currentPositions[i] * worldToLocalMatrix;
		deformedPointsLocal[i] = ptOrig + ((deformedPointsLocal[i] - ptOrig) * weights[i]);
	}
	return 0;
}

