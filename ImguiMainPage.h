#pragma once

#include "commonOsg/OsgImguiHandler.h"
#include "commonOsg/osgPickHandler.h"
#include "commonOsg/osgCameraHandler.h"

#include <osgViewer/Viewer>
#include <osgDB/ReadFile>
#include <osg/ref_ptr>

class ImguiMainPage : public OsgImGuiHandler {
public:
    ImguiMainPage();
    ImguiMainPage(osgViewer::Viewer& viewer, osg::ref_ptr< CameraHandler> pCameraHandler);
    ~ImguiMainPage();

protected:
    void drawUi();

private:
    osg::ref_ptr<osgViewer::Viewer> pviewer = nullptr;
    osg::ref_ptr< CameraHandler> m_pCameraHandler = nullptr;
};
