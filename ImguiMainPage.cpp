#include "ImguiMainPage.h"

#include "osgManager.h"

#include "reconstruct.h"
#include "r3live.hpp"
#include "parseBag.h"

Reconstruction::Reconstruction()
{
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

    if (ImGui::BeginTabBar("Functions", ImGuiTabBarFlags_None))
    {   
        if (ImGui::BeginTabItem("parse bag and reconstruct"))
        {
            if (ImGui::Button("reconstruct point map")) {
                std::thread thread([]() {
                    Eigen::initParallel();
                    R3LIVE* fast_lio_instance = new R3LIVE();

                    //ParseBag::getInstance()->notifyImu.connect(fast_lio_instance, &R3LIVE::imu_cbk);
                    //ParseBag::getInstance()->notifyImage.connect(fast_lio_instance, &R3LIVE::image_comp_callback);
                    //ParseBag::getInstance()->notifyPoints.connect(fast_lio_instance, &R3LIVE::feat_points_cbk);

                    std::string fileName = "D:/dataset/lab/c2_lvi/20230607lvi.bag";
                    ParseBag::getInstance()->parseBag(fileName);
                    });
                thread.detach();
            }
            if (ImGui::Button("reconstruct mesh")) {
                std::thread thread([]() {
                    Reconstruction::getInstance()->reconFromFile("");
                    });
                thread.detach();
            }
            if (ImGui::Button("read from UE")) {
                std::thread thread([]() {
                    Eigen::initParallel();
                    R3LIVE* fast_lio_instance = new R3LIVE();

                    // AirSimManager::getInstance()->notifyImu.connect(fast_lio_instance, &R3LIVE::imu_cbk);
                    // AirSimManager::getInstance()->notifyImage.connect(fast_lio_instance, &R3LIVE::image_comp_callback);
                    // AirSimManager::getInstance()->notifyPoints.connect(fast_lio_instance, &R3LIVE::feat_points_cbk);

                    // AirSimManager::getInstance()->run();
                    });
                thread.detach();
            }
            ImGui::EndTabItem();
        }

        ImGui::EndTabBar();
    }

    ImGui::End();
}
