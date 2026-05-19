/*
Notes:
- ui_g.c & ui_g.h can be replaced all the time
- use bismark to create ui and generate the DYNAMIC version of the code 
*/

#include "string.h"
#include "ui_status.h"



/** 
 * @brief Setup up all figures & strings
*/
void UIStatus::ui_init_g() {
    ui_g_Ungroup_Bayblade->figure_type = 7;
    ui_g_Ungroup_Bayblade->operate_type = 1;
    ui_g_Ungroup_Bayblade->layer = 0;
    ui_g_Ungroup_Bayblade->color = 2;
    ui_g_Ungroup_Bayblade->start_x = 688;
    ui_g_Ungroup_Bayblade->start_y = 839;
    ui_g_Ungroup_Bayblade->width = 8;
    ui_g_Ungroup_Bayblade->font_size = 80;
    ui_g_Ungroup_Bayblade->str_length = 7;
    strcpy(ui_g_Ungroup_Bayblade->string, "SPIN!!!");

    ui_g_Ungroup_Flywheel->figure_type = 7;
    ui_g_Ungroup_Flywheel->operate_type = 1;
    ui_g_Ungroup_Flywheel->layer = 0;
    ui_g_Ungroup_Flywheel->color = 6;
    ui_g_Ungroup_Flywheel->start_x = 758;
    ui_g_Ungroup_Flywheel->start_y = 370;
    ui_g_Ungroup_Flywheel->width = 6;
    ui_g_Ungroup_Flywheel->font_size = 60;
    ui_g_Ungroup_Flywheel->str_length = 7;
    strcpy(ui_g_Ungroup_Flywheel->string, "FLY-OFF");

    ui_g_Ungroup_CV->figure_type = 7;
    ui_g_Ungroup_CV->operate_type = 1;
    ui_g_Ungroup_CV->layer = 0;
    ui_g_Ungroup_CV->color = 0;
    ui_g_Ungroup_CV->start_x = 42;
    ui_g_Ungroup_CV->start_y = 751;
    ui_g_Ungroup_CV->width = 5;
    ui_g_Ungroup_CV->font_size = 50;
    ui_g_Ungroup_CV->str_length = 6;
    strcpy(ui_g_Ungroup_CV->string, "CV-OFF");

    ui_g_Ungroup_Alignment->figure_type = 7;
    ui_g_Ungroup_Alignment->operate_type = 1;
    ui_g_Ungroup_Alignment->layer = 0;
    ui_g_Ungroup_Alignment->color = 3;
    ui_g_Ungroup_Alignment->start_x = 48;
    ui_g_Ungroup_Alignment->start_y = 607;
    ui_g_Ungroup_Alignment->width = 6;
    ui_g_Ungroup_Alignment->font_size = 60;
    ui_g_Ungroup_Alignment->str_length = 2;
    strcpy(ui_g_Ungroup_Alignment->string, "AL");

    uint32_t idx = 0;
    for (int i = 0; i < TOTAL_STRING; i++) {
        ui_g_now_strings[i].figure_name[2] = idx & 0xFF;
        ui_g_now_strings[i].figure_name[1] = (idx >> 8) & 0xFF;
        ui_g_now_strings[i].figure_name[0] = (idx >> 16) & 0xFF;
        ui_g_now_strings[i].operate_type = 1;
#ifndef MANUAL_DIRTY
        ui_g_last_strings[i] = ui_g_now_strings[i];
#endif
        ui_g_dirty_string[i] = 1;
        idx++;
    }

    SCAN_AND_SEND();

    for (int i = 0; i < TOTAL_STRING; i++) {
        ui_g_now_strings[i].operate_type = 2;
    }
}

/**
    @brief Checks what figures/strings has been updated and only sends those that have changes
        to the server
*/
void UIStatus::ui_update_g() {
    #ifndef MANUAL_DIRTY
    for (int i = 0; i < TOTAL_STRING; i++) {
        if (memcmp(&ui_g_now_strings[i], &ui_g_last_strings[i], sizeof(ui_g_now_strings[i])) != 0) {
            ui_g_dirty_string[i] = ui_g_max_send_count[TOTAL_FIGURE + i];
            ui_g_last_strings[i] = ui_g_now_strings[i];
        }
    }
    #endif
    SCAN_AND_SEND();
}
