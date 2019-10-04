#include "cvMeshBlurCmd.h"
#include "cvMeshBlurDeformer.h"


bool IsShapeNode(MDagPath& path) {
    return path.node().hasFn(MFn::kMesh) ||
        path.node().hasFn(MFn::kNurbsCurve) ||
        path.node().hasFn(MFn::kNurbsSurface);
}


MStatus GetShapeNode(MDagPath& path, bool intermediate) {
    MStatus status;

    if (IsShapeNode(path)) {
        // Start at the transform so we can honor the intermediate flag.
        path.pop();
    }

    if (path.hasFn(MFn::kTransform)) {
        unsigned int shapeCount = path.childCount();

        for (unsigned int i = 0; i < shapeCount; ++i) {
            status = path.push(path.child(i));
            CHECK_MSTATUS_AND_RETURN_IT(status);
            if (!IsShapeNode(path)) {
                path.pop();
                continue;
            }

            MFnDagNode fnNode(path, &status);
            CHECK_MSTATUS_AND_RETURN_IT(status);
            if ((!fnNode.isIntermediateObject() && !intermediate) ||
                (fnNode.isIntermediateObject() && intermediate)) {
                return MS::kSuccess;
            }
            // Go to the next shape
            path.pop();
        }
    }

    // No valid shape node found.
    return MS::kFailure;
}

const char* cvMeshBlurCmd::kName = "cvMeshBlur";


cvMeshBlurCmd::cvMeshBlurCmd()
    : name_("cvMeshBlur#") {
}


MSyntax cvMeshBlurCmd::newSyntax()
{
    MSyntax syntax;
    syntax.addFlag("-n", "-name", MSyntax::kString);
    syntax.setObjectType(MSyntax::kSelectionList, 1, 1);
    syntax.useSelectionAsDefault(true);
    return syntax;
}

void* cvMeshBlurCmd::creator()
{
    return new cvMeshBlurCmd;
}

bool cvMeshBlurCmd::isUndoable() const {
    return true;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
Summary:
    Creates an cvMeshBlur deformer based on the selection.
Parameters:
    [in]    args    - MArgList for command.
Returns:
    MStatus
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
MStatus cvMeshBlurCmd::doIt(const MArgList& args)
{
    MStatus status;
    MArgDatabase argData(syntax(), args);
    argData.getObjects(selectionList_);

    if (argData.isFlagSet("-n")) {
        name_ = argData.flagArgumentString("-n", 0, &status);
        CHECK_MSTATUS_AND_RETURN_IT(status);
    }

    status = GetGeometryPath();
    CHECK_MSTATUS_AND_RETURN_IT(status);
    
    // Add the cvWrap creation command to the modifier.
    MString command = "deformer -type cvMeshBlur -n \"" + name_ + "\"";
    MFnDagNode fnNode(pathMesh_);
    command += " " + fnNode.partialPathName();
    status = dgMod_.commandToExecute(command);
    CHECK_MSTATUS_AND_RETURN_IT(status);
    
    return redoIt();
}


MStatus cvMeshBlurCmd::GetGeometryPath() {
    MStatus status;
    // The driver is selected last
    status = selectionList_.getDagPath(0, pathMesh_);
    CHECK_MSTATUS_AND_RETURN_IT(status);
    status = GetShapeNode(pathMesh_, false);
    // The driver must be a mesh for this specific algorithm.
    if (!pathMesh_.hasFn(MFn::kMesh)) {
        MGlobal::displayError("cvMeshBlur only works on meshes.");
        return MS::kFailure;
    }

    return MS::kSuccess;
}


MStatus cvMeshBlurCmd::redoIt() {
    MStatus status;

    status = dgMod_.doIt();
    CHECK_MSTATUS_AND_RETURN_IT(status);

    // Reacquire the path because on referenced geo, a new mesh is created (the ShapeDeformed).
    status = GetGeometryPath();
    CHECK_MSTATUS_AND_RETURN_IT(status);
    // Get the created mesh blur deformer node.
    status = GetLatestMeshBlurNode();
    CHECK_MSTATUS_AND_RETURN_IT(status);

    MFnDependencyNode fnDeformerNode(oMeshBlurNode_, &status);
    MPlug plugInTime = fnDeformerNode.findPlug("time", false, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);
    MPlug plugInMatrix = fnDeformerNode.findPlug("worldMatrix", false, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    MSelectionList list;
    status = MGlobal::getSelectionListByName("time1", list);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    MObject oTime;
    list.getDependNode(0, oTime);
    MFnDependencyNode fnTime(oTime, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);
    MPlug plugOutTime = fnTime.findPlug("outTime", false, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    MDagPath pathTransform(pathMesh_);
    pathMesh_.pop();
    MFnDependencyNode fnTransform(pathTransform.node(), &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);
    MPlug plugOutWorldMatrix = fnTransform.findPlug("worldMatrix", false, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);
    plugOutWorldMatrix = plugOutWorldMatrix.elementByLogicalIndex(0, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    MDGModifier dgMod;
    dgMod.connect(plugOutTime, plugInTime);
    dgMod.connect(plugOutWorldMatrix, plugInMatrix);
    status = dgMod.doIt();
    CHECK_MSTATUS_AND_RETURN_IT(status);

    setResult(fnDeformerNode.name());
    
    return MS::kSuccess;
}


MStatus cvMeshBlurCmd::GetLatestMeshBlurNode() {
    MStatus status;
    MObject oMesh = pathMesh_.node();

    // Since we use MDGModifier to execute the deformer command, we can't get
    // the created deformer node, so we need to find it in the deformation chain.
    MItDependencyGraph itDG(oMesh,
        MFn::kGeometryFilt,
        MItDependencyGraph::kUpstream,
        MItDependencyGraph::kDepthFirst,
        MItDependencyGraph::kNodeLevel,
        &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);
    MObject oDeformerNode;
    for (; !itDG.isDone(); itDG.next()) {
        oDeformerNode = itDG.currentItem();
        MFnDependencyNode fnNode(oDeformerNode, &status);
        CHECK_MSTATUS_AND_RETURN_IT(status);
        if (fnNode.typeId() == cvMeshBlur::id) {
            oMeshBlurNode_ = oDeformerNode;
            return MS::kSuccess;
        }
    }
    return MS::kFailure;
}


MStatus cvMeshBlurCmd::undoIt() {
    MStatus status;
    status = dgMod_.undoIt();
    CHECK_MSTATUS_AND_RETURN_IT(status);

    return MS::kSuccess;
}
