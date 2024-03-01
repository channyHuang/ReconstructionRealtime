#include "osgManager.h"

#include "commonOsg/commonOsg.h"

OsgManager* OsgManager::instance = nullptr;

OsgManager::OsgManager() {
	root = new osg::Group;
	root->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
	root->addChild(createAxis());

	sceneSwitch = new osg::Switch;
	sceneSwitch->setAllChildrenOn();
	root->addChild(sceneSwitch);

	localAxisNode = new osg::MatrixTransform;
	localAxisMatrix.makeIdentity();
	localAxisNode->setMatrix(localAxisMatrix);
	localAxisNode->addChild(createAxis());
	sceneSwitch->addChild(localAxisNode);
}

OsgManager::~OsgManager() {
	pviewer.release();
}

void OsgManager::setViewer(osgViewer::Viewer& viewer) {
	pviewer = &viewer;

	//pviewer->addEventHandler(new PickHandler());
	pviewer->setSceneData(root);
}

void OsgManager::switchScene() {
	sceneMaxIdx = sceneSwitch->getNumChildren();
	if (sceneIdx >= sceneMaxIdx) {
		sceneSwitch->setAllChildrenOn();
		sceneIdx = 0;
	}
	else {
		sceneSwitch->setSingleChildOn(sceneIdx);
		sceneIdx++;
	}
}

void OsgManager::show(const std::string &str, bool braw) {
	if (!braw) {
		loadObjectModel(str.c_str(), "E:/projects/r3live-lab-res/", pviewer);
	}
	else {
		osg::ref_ptr<osg::Geometry> geom = loadObjModel(str, false);
		sceneSwitch->addChild(geom);

		osg::ref_ptr<osg::Geometry> geomWireframe = new osg::Geometry(*geom.get(), osg::CopyOp::SHALLOW_COPY);
		osg::ref_ptr<osg::Vec4Array> vcolors = new osg::Vec4Array;
		vcolors->push_back(osg::Vec4(0.f, 1.f, 0.f, 1.f));
		geomWireframe->setColorArray(vcolors, osg::Array::Binding::BIND_OVERALL);
		setWireFrame(geomWireframe->getOrCreateStateSet(), ShowType::SHOW_WIREFRAME);
		sceneSwitch->addChild(geomWireframe);
	}
}

void OsgManager::updatePoints(const std::vector<std::vector<double>>& pts) {
	std::lock_guard<std::mutex> locker(notifyMutex);
	osg::ref_ptr<osg::Geometry> geomPoints = new osg::Geometry();
	osg::Vec3Array* varray = new osg::Vec3Array();
	osg::Vec3Array* carray = new osg::Vec3Array();

	for (int i = 0; i < pts.size(); ++i) {
		varray->push_back(osg::Vec3(pts[i][0], pts[i][1], pts[i][2]));
		carray->push_back(osg::Vec3(pts[i][3], pts[i][4], pts[i][5]));
	}
	
	geomPoints->setVertexArray(varray);
	geomPoints->setColorArray(carray, osg::Array::Binding::BIND_PER_VERTEX);
	geomPoints->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, varray->size()));

	//geomPoints->dirtyBound();
	//geomPoints->dirtyGLObjects();
	sceneSwitch->addChild(geomPoints);
}

void OsgManager::updatePose(double x, double y, double z, double w, double tx, double ty, double tz) {
	localAxisMatrix.setRotate(osg::Quat(x, y, z, w));
	localAxisMatrix.setTrans(osg::Vec3d(tx, ty, tz));
	localAxisNode->setMatrix(localAxisMatrix);
}