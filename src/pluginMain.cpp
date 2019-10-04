
#include "cvMeshBlurCmd.h"
#include "cvMeshBlurDeformer.h"
#include <maya/MFnPlugin.h>

MStatus initializePlugin(MObject obj)
{
    MStatus status;

    MFnPlugin plugin(obj, "Chad Vernon", "1.0", "any");

    status = plugin.registerNode("cvMeshBlur", cvMeshBlur::id, cvMeshBlur::creator, cvMeshBlur::initialize, MPxNode::kDeformerNode);
    CHECK_MSTATUS_AND_RETURN_IT(status);
    status = plugin.registerCommand(cvMeshBlurCmd::kName, cvMeshBlurCmd::creator, cvMeshBlurCmd::newSyntax);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    return status;
}

MStatus uninitializePlugin(MObject obj)
{
    MStatus	status;
    MFnPlugin plugin(obj);

    status = plugin.deregisterCommand(cvMeshBlurCmd::kName);
    CHECK_MSTATUS_AND_RETURN_IT(status);
    status = plugin.deregisterNode(cvMeshBlur::id);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    return status;
}
