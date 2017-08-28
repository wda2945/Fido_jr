//
//  debug.h
//
//  Created by Martin Lane-Smith on 2/7/17.
//  Copyright © 2016 Martin Lane-Smith. All rights reserved.
//

#ifndef debug_h
#define debug_h

#include <stdio.h>

#define MAIN_DEBUG
#define I2C_DEBUG
#define NAVIGATOR_DEBUG
#define SCANNER_DEBUG
#define AUTOPILOT_DEBUG
#define BEHAVIOR_DEBUG

#define MAX_DEBUG_TEXT	100
//debug helpers
FILE *fopen_logfile(const char *name);

#define tprintf(...) {char tmp[MAX_DEBUG_TEXT];\
    snprintf(tmp,MAX_DEBUG_TEXT,__VA_ARGS__);\
    tmp[MAX_DEBUG_TEXT-1] = 0;\
    print_debug_message_to_file(stdout, tmp);}

#define tfprintf(dbgfile, ...) {char tmp[MAX_DEBUG_TEXT];\
    snprintf(tmp,MAX_DEBUG_TEXT,__VA_ARGS__);\
    tmp[MAX_DEBUG_TEXT-1] = 0;\
    print_debug_message_to_file(dbgfile, tmp);}

#ifdef __cplusplus
extern "C" {
#endif

void print_debug_message_to_file(FILE *dbgfile, const char *text);
void print_debug_message(const char *text);

#ifdef __cplusplus
}
#endif

#endif /* debug_h */
