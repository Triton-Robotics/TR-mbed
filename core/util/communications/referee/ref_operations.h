#ifndef _REF_OPERATIONS_HPP
#define _REF_OPERATIONS_HPP

#include <string>
#include "ref_ui.h"

static bool enablePrintRefData = 0;

// Outlines all referee system serial communication operations (receiving & sending) in an iteration
void refereeThread(BufferedSerial* referee);

#endif