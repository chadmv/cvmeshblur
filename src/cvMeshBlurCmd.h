#ifndef CVMESHBLURCMD_H
#define CVMESHBLURCMD_H

#include <maya/MSelectionList.h>
#include <maya/MDagPath.h>
#include <maya/MDagPathArray.h>
#include <maya/MPlug.h>
#include <maya/MString.h>
#include <maya/MStringArray.h>
#include <maya/MGlobal.h>
#include <maya/MSyntax.h>
#include <maya/MString.h>
#include <maya/MArgList.h>
#include <maya/MArgDatabase.h>
#include <maya/MObject.h>
#include <maya/MDGModifier.h>

#include <maya/MItGeometry.h>
#include <maya/MItDependencyGraph.h>

#include <maya/MFnDagNode.h>
#include <maya/MFnDependencyNode.h>
#include <maya/MFn.h>
#include <maya/MFnData.h>
#include <maya/MFnMesh.h>
#include <maya/MFnTransform.h>

#include <maya/MPxCommand.h>


class cvMeshBlurCmd : public MPxCommand
{
public:
    cvMeshBlurCmd();
    virtual MStatus	doIt(const MArgList&);
    virtual MStatus undoIt();
    virtual MStatus redoIt();
    virtual bool isUndoable() const;
    static void* creator();
    static MSyntax newSyntax();

    const static char* kName;  /**< The name of the command. */


private:
    MStatus GetGeometryPath();
    MStatus GetLatestMeshBlurNode();

    MString name_;  /**< Name of cvMeshBlur node to create. */
    MSelectionList selectionList_;  /**< Selected command input nodes. */
    MDagPath pathMesh_;
    MObject oMeshBlurNode_;  /**< MObject to the cvMeshBlur node in focus. */
    MDGModifier dgMod_;

};

#endif
