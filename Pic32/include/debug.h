//
//  debug.h
//
//  Created by Martin Lane-Smith on 2/7/17.
//  Copyright © 2016 Martin Lane-Smith. All rights reserved.
//

#ifndef debug_h
#define debug_h

#include <stdio.h>

#define MAX_DEBUG_TEXT	100

//debug helpers

#define tprintf(...) {char tmp[MAX_DEBUG_TEXT];\
    snprintf(tmp,MAX_DEBUG_TEXT,__VA_ARGS__);\
    tmp[MAX_DEBUG_TEXT-1] = 0;\
    print_debug_message(tmp);}

#define DEBUGPRINT(...) tprintf(__VA_ARGS__);
#define ERRORPRINT(...) tprintf(__VA_ARGS__);

#ifdef __cplusplus
extern "C" {
#endif

void print_debug_message(const char *text);

#ifdef __cplusplus
}
#endif

#endif /* debug_h */
