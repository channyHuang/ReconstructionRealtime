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

	show();
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

void OsgManager::show() {
	int height = 5;
	osg::Vec3Array* varray = new osg::Vec3Array();
	osg::Vec3Array* carray = new osg::Vec3Array();

	int curx = 0, cury = 0;

	varray->push_back(osg::Vec3(curx, cury, height));
	carray->push_back(osg::Vec3(1, 0, 1));

	for (int y = 5; y >= -20; y -= 3) {
		varray->push_back(osg::Vec3(curx, y, height));
		curx = 15 - curx;
		varray->push_back(osg::Vec3(curx, y, height));
	}

	osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
	geom->setVertexArray(varray);
	geom->setColorArray(carray, osg::Array::Binding::BIND_OVERALL);
	geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINE_STRIP, 0, varray->size()));

	sceneSwitch->addChild(geom);
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

	sceneSwitch->addChild(geomPoints);
}

void OsgManager::updatePoints(osg::ref_ptr<osg::Geometry> geomPoints) {
	sceneSwitch->addChild(geomPoints);
}

void OsgManager::updatePose(double x, double y, double z, double w, double tx, double ty, double tz) {
	localAxisMatrix.setRotate(osg::Quat(x, y, z, w));
	localAxisMatrix.setTrans(osg::Vec3d(tx, ty, tz));
	localAxisNode->setMatrix(localAxisMatrix);

	if (pathGeom == nullptr) {
		osg::Vec3Array* varray = new osg::Vec3Array();
		osg::Vec3Array* carray = new osg::Vec3Array();

		varray->push_back(osg::Vec3(tx, ty, tz));
		carray->push_back(osg::Vec3(1, 1, 0));

		pathGeom = new osg::Geometry;
		pathGeom->setVertexArray(varray);
		pathGeom->setColorArray(carray, osg::Array::Binding::BIND_OVERALL);
		pathGeom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINE_STRIP, 0, varray->size()));

		sceneSwitch->addChild(pathGeom);
	}
	else {
		osg::Vec3Array* varray = (osg::Vec3Array*)pathGeom->getVertexArray();
		varray->push_back(osg::Vec3(tx, ty, tz));

		pathGeom->setVertexArray(varray);
		pathGeom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINE_STRIP, 0, varray->size()));
		pathGeom->dirtyBound();
		pathGeom->dirtyGLObjects();
	}
}