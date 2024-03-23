#include "r3live.hpp"


#include "reconstruct.h"

void R3LIVE::service_recon_update() {
    int count = 100;
    while (1)
    {
        g_mutex_render.lock();
        if (m_mvs_recorder.m_pts_in_views_vec.size() <= count) {
            g_mutex_render.unlock();

            std::this_thread::sleep_for(std::chrono::milliseconds(THREAD_SLEEP_TIM));
            std::this_thread::yield();

            continue;
        }
        count += 100;
        Reconstruction::getInstance()->reconFromRecorder(m_mvs_recorder);
        g_mutex_render.unlock();
    }
}
