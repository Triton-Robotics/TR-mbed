#include "ui_g.h"

const int TOTAL_FIGURE = 0;
const int TOTAL_STRING = 4;

class UIStatus: UI <TOTAL_FIGURE, TOTAL_STRING>{
    public: 
        uint8_t ui_g_max_send_count[TOTAL_FIGURE + TOTAL_STRING] = {
            1,
            1,
            1,
            1,
        };

        #define ui_g_Ungroup_Bayblade (&(ui_g_now_strings[0]))
        #define ui_g_Ungroup_Flywheel (&(ui_g_now_strings[1]))
        #define ui_g_Ungroup_CV (&(ui_g_now_strings[2]))
        #define ui_g_Ungroup_Alignment (&(ui_g_now_strings[3]))


        #define ui_g_Ungroup_Bayblade_max_send_count (ui_g_max_send_count[0])
        #define ui_g_Ungroup_Flywheel_max_send_count (ui_g_max_send_count[1])
        #define ui_g_Ungroup_CV_max_send_count (ui_g_max_send_count[2])
        #define ui_g_Ungroup_Alignment_max_send_count (ui_g_max_send_count[3])

        #ifdef MANUAL_DIRTY

        #define ui_g_Ungroup_Bayblade_dirty (ui_g_dirty_string[0])
        #define ui_g_Ungroup_Flywheel_dirty (ui_g_dirty_string[1])
        #define ui_g_Ungroup_CV_dirty (ui_g_dirty_string[2])
        #define ui_g_Ungroup_Alignment_dirty (ui_g_dirty_string[3])
        #endif

        UIStatus(uint16_t robot_id, std::function<void(uint8_t*, uint16_t )> send_func) : UI (robot_id, send_func){}
        void ui_init_g();
        void ui_update_g();
};