#include "ImguiMainPage.h"

#include "osgManager.h"

GLuint textureID;

#include "reconstruct.h"

Reconstruction::Reconstruction()
{
    //notifyPoint.connect(OsgManager::getInstance(), &OsgManager::addPoint);
    //notifyPoints.connect(OsgManager::getInstance(), &OsgManager::addPoints);
    //notifyPose.connect(OsgManager::getInstance(), &OsgManager::addPose);
    //notifyEdge.connect(OsgManager::getInstance(), &OsgManager::addEdge);
    //notifyFaces.connect(OsgManager::getInstance(), &OsgManager::addTri);
}


ImguiMainPage::ImguiMainPage() {
    
}

ImguiMainPage::ImguiMainPage(osgViewer::Viewer& viewer, osg::ref_ptr< CameraHandler> pCameraHandler) {
    pviewer = &viewer;
    m_pCameraHandler = pCameraHandler;
    OsgManager::getInstance()->setViewer(viewer);
}

ImguiMainPage::~ImguiMainPage() {
    pviewer = nullptr;
}

void ImguiMainPage::drawUi() {
    ImGui::Begin("3d reconstruction");
    if (ImGui::Button("Switch Scene")) {
        OsgManager::getInstance()->switchScene();
    }
    ImGui::Checkbox("Rotate By Axis", &m_pCameraHandler->bRotateByAxis);
    if (m_pCameraHandler->bRotateByAxis) {
        ImGui::Text("Rotate by axis: 0-x, 1-y, 2-z");
        ImGui::SliderInt("Axis x y z", &m_pCameraHandler->axis, 0, 3);
    }
    if (ImGui::BeginTabBar("Functions", ImGuiTabBarFlags_None))
    {
        if (ImGui::BeginTabItem("test1"))
        {
            if (ImGui::Button("show in glsl")) {
                OsgManager::getInstance()->show("E:/projects/r3live-lab-res/textured_mesh.obj", false);
            }
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("test show model"))
        {
            if (ImGui::Button("show in osg")) {
                OsgManager::getInstance()->show("E:/projects/r3live-lab-res/textured_mesh.obj");
            }
            ImGui::EndTabItem();
        }
        
        if (ImGui::BeginTabItem("parse bag and reconstruct"))
        {
            //std::string fileName = "D:/dataset/lab/c2_lvi/20230630-obs-lvi.bag";
            //std::string fileName = "D:/dataset/lab/c2_lvi/20230607lvi.bag";
            std::string fileName = "D:/dataset/lab/c2_lvi/20230630-object-lvi.bag";

            //if (ImGui::Button("parse header")) {
            //    ParseBag::getInstance()->parseBag(fileName);
            //}

            if (ImGui::Button("reconstruct point map")) {
                std::thread thread([]() {
                    Reconstruction::getInstance()->reconstruct_points();
                    });
                thread.detach();
            }
            if (ImGui::Button("reconstruct mesh")) {
                std::thread thread([]() {
                    Reconstruction::getInstance()->reconstruct_main();
                    });
                thread.detach();
            }
            ImGui::EndTabItem();
        }

        ImGui::EndTabBar();
    }

    ImGui::End();
}
