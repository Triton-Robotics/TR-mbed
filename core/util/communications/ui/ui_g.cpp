//
// Created by RM UI Designer
// Dynamic Edition
//

/*
Notes:
- ui_g.c & ui_g.h can be replaced all the time
- use bismark to create ui and generate the DYNAMIC version of the code 
*/

#include "string.h"
#include "ui_interface.h"
#include "ui_g.h"

#define TOTAL_FIGURE 7
#define TOTAL_STRING 1

// Init types
ui_interface_figure_t ui_g_now_figures[TOTAL_FIGURE];
uint8_t ui_g_dirty_figure[TOTAL_FIGURE];
ui_interface_string_t ui_g_now_strings[TOTAL_STRING];
uint8_t ui_g_dirty_string[TOTAL_STRING];

// Maximum amount of times a certain figure should be sent
uint8_t ui_g_max_send_count[TOTAL_FIGURE + TOTAL_STRING] = {
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
};

// Creates last figure/string lists to compare changes between the current and prev itterations
//      if not doing manual dirty
#ifndef MANUAL_DIRTY
ui_interface_figure_t ui_g_last_figures[TOTAL_FIGURE];
ui_interface_string_t ui_g_last_strings[TOTAL_STRING];
#endif

// Macro for sending data (Currently only just prints)
#define SCAN_AND_SEND() ui_scan_and_send(ui_g_now_figures, ui_g_dirty_figure, ui_g_now_strings, ui_g_dirty_string, TOTAL_FIGURE, TOTAL_STRING)

/** 
 * @brief Setup up all figures & strings
*/
void ui_init_g() {
    ui_g_Shapes_GreenRect->figure_type = 1;
    ui_g_Shapes_GreenRect->operate_type = 1;
    ui_g_Shapes_GreenRect->layer = 0;
    ui_g_Shapes_GreenRect->color = 2;
    ui_g_Shapes_GreenRect->start_x = 852;
    ui_g_Shapes_GreenRect->start_y = 437;
    ui_g_Shapes_GreenRect->width = 20;
    ui_g_Shapes_GreenRect->end_x = 1037;
    ui_g_Shapes_GreenRect->end_y = 622;

    ui_g_Shapes_RedLine->figure_type = 0;
    ui_g_Shapes_RedLine->operate_type = 1;
    ui_g_Shapes_RedLine->layer = 0;
    ui_g_Shapes_RedLine->color = 0;
    ui_g_Shapes_RedLine->start_x = 731;
    ui_g_Shapes_RedLine->start_y = 529;
    ui_g_Shapes_RedLine->width = 10;
    ui_g_Shapes_RedLine->end_x = 1211;
    ui_g_Shapes_RedLine->end_y = 527;

    ui_g_Shapes_PurpleCircle->figure_type = 2;
    ui_g_Shapes_PurpleCircle->operate_type = 1;
    ui_g_Shapes_PurpleCircle->layer = 0;
    ui_g_Shapes_PurpleCircle->color = 4;
    ui_g_Shapes_PurpleCircle->start_x = 953;
    ui_g_Shapes_PurpleCircle->start_y = 528;
    ui_g_Shapes_PurpleCircle->width = 10;
    ui_g_Shapes_PurpleCircle->r = 166;

    ui_g_Shapes_RedEllipse->figure_type = 3;
    ui_g_Shapes_RedEllipse->operate_type = 1;
    ui_g_Shapes_RedEllipse->layer = 0;
    ui_g_Shapes_RedEllipse->color = 0;
    ui_g_Shapes_RedEllipse->start_x = 953;
    ui_g_Shapes_RedEllipse->start_y = 531;
    ui_g_Shapes_RedEllipse->width = 10;
    ui_g_Shapes_RedEllipse->rx = 230;
    ui_g_Shapes_RedEllipse->ry = 230;

    ui_g_Shapes_WhiteArc->figure_type = 4;
    ui_g_Shapes_WhiteArc->operate_type = 1;
    ui_g_Shapes_WhiteArc->layer = 0;
    ui_g_Shapes_WhiteArc->color = 8;
    ui_g_Shapes_WhiteArc->start_x = 1035;
    ui_g_Shapes_WhiteArc->start_y = 561;
    ui_g_Shapes_WhiteArc->width = 30;
    ui_g_Shapes_WhiteArc->start_angle = 70;
    ui_g_Shapes_WhiteArc->end_angle = 190;
    ui_g_Shapes_WhiteArc->rx = 178;
    ui_g_Shapes_WhiteArc->ry = 178;

    ui_g_Ungroup_RedNumber->figure_type = 6;
    ui_g_Ungroup_RedNumber->operate_type = 1;
    ui_g_Ungroup_RedNumber->layer = 1;
    ui_g_Ungroup_RedNumber->color = 0;
    ui_g_Ungroup_RedNumber->start_x = 231;
    ui_g_Ungroup_RedNumber->start_y = 679;
    ui_g_Ungroup_RedNumber->width = 5;
    ui_g_Ungroup_RedNumber->font_size = 50;
    ui_g_Ungroup_RedNumber->number = 12345;

    ui_g_Ungroup_RedFloat->figure_type = 5;
    ui_g_Ungroup_RedFloat->operate_type = 1;
    ui_g_Ungroup_RedFloat->layer = 1;
    ui_g_Ungroup_RedFloat->color = 0;
    ui_g_Ungroup_RedFloat->start_x = 204;
    ui_g_Ungroup_RedFloat->start_y = 597;
    ui_g_Ungroup_RedFloat->width = 5;
    ui_g_Ungroup_RedFloat->font_size = 50;
    ui_g_Ungroup_RedFloat->number = 12345;

    ui_g_Text_CyanText->figure_type = 7;
    ui_g_Text_CyanText->operate_type = 1;
    ui_g_Text_CyanText->layer = 1;
    ui_g_Text_CyanText->color = 6;
    ui_g_Text_CyanText->start_x = 206;
    ui_g_Text_CyanText->start_y = 747;
    ui_g_Text_CyanText->width = 3;
    ui_g_Text_CyanText->font_size = 30;
    ui_g_Text_CyanText->str_length = 11;
    strcpy(ui_g_Text_CyanText->string, "Hello World");

    // Sets up ids for each figure/string
    uint32_t idx = 0;
    for (int i = 0; i < TOTAL_FIGURE; i++) {
        ui_g_now_figures[i].figure_name[2] = idx & 0xFF;
        ui_g_now_figures[i].figure_name[1] = (idx >> 8) & 0xFF;
        ui_g_now_figures[i].figure_name[0] = (idx >> 16) & 0xFF;
        ui_g_now_figures[i].operate_type = 1;
        // Sets up last_figures if not doing manual dirty
        #ifndef MANUAL_DIRTY
            ui_g_last_figures[i] = ui_g_now_figures[i];
        #endif
        // Indicates that the current figure should be sent
        ui_g_dirty_figure[i] = 1;
        idx++;
    }
    for (int i = 0; i < TOTAL_STRING; i++) {
        ui_g_now_strings[i].figure_name[2] = idx & 0xFF;
        ui_g_now_strings[i].figure_name[1] = (idx >> 8) & 0xFF;
        ui_g_now_strings[i].figure_name[0] = (idx >> 16) & 0xFF;
        ui_g_now_strings[i].operate_type = 1;
        // Sets up last_strings if not doing manual dirty
        #ifndef MANUAL_DIRTY
            ui_g_last_strings[i] = ui_g_now_strings[i];
        #endif
        // Indicates that the current string should be sent
        ui_g_dirty_string[i] = 1;
        idx++;
    }

    SCAN_AND_SEND();

    // Once all figure has been created, default to mode for all figures/strings
    for (int i = 0; i < TOTAL_FIGURE; i++) {
        ui_g_now_figures[i].operate_type = 2;
    }
    for (int i = 0; i < TOTAL_STRING; i++) {
        ui_g_now_strings[i].operate_type = 2;
    }
}

/**
    @brief Checks what figures/strings has been updated and only sends those that have changes
        to the server
*/
void ui_update_g() {
    #ifndef MANUAL_DIRTY
        // Records which figures/strings have changed (based on the last itteration) and raises
        //      a flag (ui_g_dirty_figure) as a signal to udpate
        for (int i = 0; i < TOTAL_FIGURE; i++) {
            if (memcmp(&ui_g_now_figures[i], &ui_g_last_figures[i], sizeof(ui_g_now_figures[i])) != 0) {
                ui_g_dirty_figure[i] = ui_g_max_send_count[i];
                ui_g_last_figures[i] = ui_g_now_figures[i];
            }
        }
        for (int i = 0; i < TOTAL_STRING; i++) {
            if (memcmp(&ui_g_now_strings[i], &ui_g_last_strings[i], sizeof(ui_g_now_strings[i])) != 0) {
                ui_g_dirty_string[i] = ui_g_max_send_count[TOTAL_FIGURE + i];
                ui_g_last_strings[i] = ui_g_now_strings[i];
            }
        }
    #endif
    // Updates all figures/strings that have been changed based on the flags (ui_g_dirty_figure) that have been raised
    SCAN_AND_SEND();
}
