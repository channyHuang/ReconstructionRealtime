#pragma once

#include <unordered_map>
#include <mutex>

#include <osgViewer/Viewer>
#include <osg/Group>
#include <osg/Switch>
#include <osg/Program>
#include <osgDB/ReadFile>
#include <osg/LineWidth>
#include <osgViewer/Renderer>
#include <osg/Texture2D>
#include <osg/PolygonMode>
#include <osg/BindImageTexture>

class OsgManager
{
public:
	static OsgManager* getInstance() {
		if (instance == nullptr) {
			instance = new OsgManager();
		}
		return instance;
	}

	~OsgManager();

	void setViewer(osgViewer::Viewer& viewer);
	void show();
	void switchScene();
	void updatePoints(const std::vector<std::vector<double>>& pts);
	void updatePoints(osg::ref_ptr<osg::Geometry> geomPoints);
	void updatePose(double x, double y, double z, double w, double tx, double ty, double tz);
public:

private:
	OsgManager();
	
private:
	static OsgManager* instance;

	osg::ref_ptr<osg::Group> root = nullptr;
	osg::ref_ptr<osg::Geometry> pathGeom = nullptr;
	osg::ref_ptr<osg::Group> sunLight = nullptr;
	osg::ref_ptr<osg::Group> rootGeomTerrain = nullptr;
	osg::ref_ptr<osgViewer::Viewer> pviewer = nullptr;
	osg::ref_ptr<osg::Switch> sceneSwitch;
	int sceneMaxIdx = 0;
	int sceneIdx = 0;

	osg::ref_ptr<osg::MatrixTransform> localAxisNode = nullptr;
	osg::Matrix localAxisMatrix;
	std::mutex notifyMutex;
};

