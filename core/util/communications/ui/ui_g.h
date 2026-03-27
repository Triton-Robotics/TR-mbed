//
// Created by RM UI Designer
// Dynamic Edition
//

#ifndef UI_g_H
#define UI_g_H

#include "ui_interface.h"

// Grabs these variables from ui_g.c
extern ui_interface_figure_t ui_g_now_figures[7];
extern uint8_t ui_g_dirty_figure[7];
extern ui_interface_string_t ui_g_now_strings[1];
extern uint8_t ui_g_dirty_string[1];

extern uint8_t ui_g_max_send_count[8];

// Macros to get address of specific figures/strings
#define ui_g_Shapes_GreenRect ((ui_interface_rect_t*)&(ui_g_now_figures[0]))
#define ui_g_Shapes_RedLine ((ui_interface_line_t*)&(ui_g_now_figures[1]))
#define ui_g_Shapes_PurpleCircle ((ui_interface_round_t*)&(ui_g_now_figures[2]))
#define ui_g_Shapes_RedEllipse ((ui_interface_ellipse_t*)&(ui_g_now_figures[3]))
#define ui_g_Shapes_WhiteArc ((ui_interface_arc_t*)&(ui_g_now_figures[4]))
#define ui_g_Ungroup_RedNumber ((ui_interface_number_t*)&(ui_g_now_figures[5]))
#define ui_g_Ungroup_RedFloat ((ui_interface_number_t*)&(ui_g_now_figures[6]))

#define ui_g_Text_CyanText (&(ui_g_now_strings[0]))

// Macros to get size of specific figures/strings
#define ui_g_Shapes_GreenRect_max_send_count (ui_g_max_send_count[0])
#define ui_g_Shapes_RedLine_max_send_count (ui_g_max_send_count[1])
#define ui_g_Shapes_PurpleCircle_max_send_count (ui_g_max_send_count[2])
#define ui_g_Shapes_RedEllipse_max_send_count (ui_g_max_send_count[3])
#define ui_g_Shapes_WhiteArc_max_send_count (ui_g_max_send_count[4])
#define ui_g_Ungroup_RedNumber_max_send_count (ui_g_max_send_count[5])
#define ui_g_Ungroup_RedFloat_max_send_count (ui_g_max_send_count[6])

#define ui_g_Text_CyanText_max_send_count (ui_g_max_send_count[7])

// Macros to get amount of proccess needed for a specific figure/string
#ifdef MANUAL_DIRTY
#define ui_g_Shapes_GreenRect_dirty (ui_g_dirty_figure[0])
#define ui_g_Shapes_RedLine_dirty (ui_g_dirty_figure[1])
#define ui_g_Shapes_PurpleCircle_dirty (ui_g_dirty_figure[2])
#define ui_g_Shapes_RedEllipse_dirty (ui_g_dirty_figure[3])
#define ui_g_Shapes_WhiteArc_dirty (ui_g_dirty_figure[4])
#define ui_g_Ungroup_RedNumber_dirty (ui_g_dirty_figure[5])
#define ui_g_Ungroup_RedFloat_dirty (ui_g_dirty_figure[6])

#define ui_g_Text_CyanText_dirty (ui_g_dirty_string[0])
#endif

void ui_init_g();
void ui_update_g();

#endif // UI_g_H
