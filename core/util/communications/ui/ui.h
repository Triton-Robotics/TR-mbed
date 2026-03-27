//
// Created by RM UI Designer
// Dynamic Edition
//

// Uses c importation methods rather than c++ if using c++
// Not 100% sure why it is done this way, possible theories are:
//  (1) Class linkage for stdint.h is only found in C++11 so used for compaibility
//  (2) Some teams code in c, used for generic template that can easily be transferred
// Will need to ask someone later, but for now, 

#ifndef UI_H
#define UI_H
#ifdef __cplusplus
extern "C" {
#endif

#include "ui_interface.h"

#include "ui_g.h"

#ifdef __cplusplus
}
#endif

#endif // UI_H
