/* 
 * File:   main.cpp
 * Author: martin
 *
 * Created on November 16, 2016, 11:47 AM
 */

#include <cstdlib>

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>

#define _SUPPRESS_PLIB_WARNING
#define _DISABLE_OPENADC10_CONFIGPORT_WARNING
#include "plib.h"

#include "FreeRTOS.h"
#include "task.h"

#include "hardware_profile.h"
#include "software_profile.h"

#include "debug.h"
#include "serial/serial.h"
#include "fido_jr.h"

int MainTaskInit();

using namespace std;

/*
 * 
 */
int main(int argc, char** argv) {
    /* Prepare the hardware */
    BoardInit();
    
    MainTaskInit();

    PORTSetPinsDigitalOut(BUZZ_IOPORT, BUZZ_BIT);
    
    vTaskStartScheduler();
    return 0;
}

/*-----------------------------------------------------------*/

void _ExceptionMessage(const char *buffer) {
    while (*buffer) {
        while (!UARTTransmitterIsReady(LOG_UART));
        UARTSendDataByte(LOG_UART, *buffer);
        buffer++;
    }
    while (!UARTTransmissionHasCompleted(LOG_UART));
}

/*-----------------------------------------------------------*/
extern "C"
{
    void vApplicationMallocFailedHook(void);
    void vApplicationIdleHook(void);
    void vApplicationStackOverflowHook(TaskHandle_t pxTask, signed char *pcTaskName);
    void vApplicationTickHook(void);
}
/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook(void) {
    /* vApplicationMallocFailedHook() will only be called if
    configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.  It is a hook
    function that will get called if a call to pvPortMalloc() fails.
    pvPortMalloc() is called internally by the kernel whenever a task, queue,
    timer or semaphore is created.  It is also called by various parts of the
    demo application.  If heap_1.c or heap_2.c are used, then the size of the
    heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
    FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
    to query the size of free heap space that remains (although it does not
    provide information on how the remaining heap might be fragmented). */
    taskDISABLE_INTERRUPTS();

    print_debug_message("MALLOC Failed");
    for (;;);
}

/*-----------------------------------------------------------*/

void vApplicationIdleHook(void) {
    /* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
    to 1 in FreeRTOSConfig.h.  It will be called on each iteration of the idle
    task.  It is essential that code added to this hook function never attempts
    to block in any way (for example, call xQueueReceive() with a block time
    specified, or call vTaskDelay()).  If the application makes use of the
    vTaskDelete() API function (as this demo application does) then it is also
    important that vApplicationIdleHook() is permitted to return to its calling
    function, because it is the responsibility of the idle task to clean up
    memory allocated by the kernel to any task that has since been deleted. */
    
    PORTToggleBits(BUZZ_IOPORT, BUZZ_BIT); 
    
}

/*-----------------------------------------------------------*/
int reOverflow = 0;

void vApplicationStackOverflowHook(TaskHandle_t pxTask, signed char *pcTaskName) {
    (void) pcTaskName;
    (void) pxTask;

    /* Run time task stack overflow checking is performed if
    configCHECK_FOR_STACK_OVERFLOW is defined >1.  This hook	function is
    called if a task stack overflow is detected.  Note the system/interrupt
    stack is not checked. */
    //    taskDISABLE_INTERRUPTS();

    //    if (reOverflow == 0) {
    //        SysLog(SYSLOG_FAILURE, "Stack: %s", pcTaskName);
    //        reOverflow = 1;
    //    }
    for (;;);
}

/*-----------------------------------------------------------*/

void vApplicationTickHook(void) {
    /* This function will be called by each tick interrupt if
    configUSE_TICK_HOOK is set to 1 in FreeRTOSConfig.h.  User code can be
    added here, but the tick hook is called from an interrupt context, so
    code must not attempt to block, and only the interrupt safe FreeRTOS API
    functions can be used (those that end in FromISR()). */
}

/*-----------------------------------------------------------*/
static unsigned int _excep_code;
static unsigned int _excep_addr;

// Declared static in case exception condition would prevent
// an auto variable from being created

static enum {
    EXCEP_IRQ = 0, // interrupt
    EXCEP_AdEL = 4, // address error exception (load or ifetch)
    EXCEP_AdES, // address error exception (store)
    EXCEP_IBE, // bus error (ifetch)
    EXCEP_DBE, // bus error (load/store)
    EXCEP_Sys, // syscall
    EXCEP_Bp, // breakpoint
    EXCEP_RI, // reserved instruction
    EXCEP_CpU, // coprocessor unusable
    EXCEP_Overflow, // arithmetic overflow
    EXCEP_Trap, // trap (possible divide by zero)
    EXCEP_IS1 = 16, // implementation specfic 1
    EXCEP_CEU, // CorExtend Unuseable
    EXCEP_C2E, // coprocessor 2
    EXCEP_COUNT
} _excep_code_enum;

const char *exceptionCode[] = {
    "interrupt",
    "address error exception (load or ifetch)",
    "address error exception (store)",
    "bus error (ifetch)",
    "bus error (load/store)",
    "syscall",
    "breakpoint",
    "reserved instruction",
    "coprocessor unusable",
    "arithmetic overflow",
    "trap (possible divide by zero)",
    "implementation specfic",
    "CorExtend Unuseable",
    "coprocessor 2"
};

static char exceptionMessage[80];

void _general_exception_handler(unsigned long ulCause, unsigned long ulStatus) {
    /* This overrides the definition provided by the kernel.  Other exceptions
    should be handled here. */

    asm volatile("mfc0 %0,$13" : "=r" (_excep_code));
    asm volatile("mfc0 %0,$14" : "=r" (_excep_addr));

    _excep_code = (_excep_code & 0x0000007C) >> 2;

    if (_excep_code < EXCEP_COUNT)
        _ExceptionMessage("\nGeneral Exception\n");
    if (_excep_code < EXCEP_COUNT)
        sprintf(exceptionMessage, "Gen Except @ %x : %s\n", _excep_addr, exceptionCode[_excep_code]);
    else
        sprintf(exceptionMessage, "Gen Except @ %x : 0x%02x\n", _excep_addr, _excep_code);
    _ExceptionMessage(exceptionMessage);

    while (1) {
#define COUNT1 1000000
        long i;
        for (i = 0; i < COUNT1; i++);
//        PORTToggleBits(USER_LED_IOPORT, USER_LED_BIT);
    }
}

/*-----------------------------------------------------------*/

void vAssertCalled(const char * pcFile, unsigned long ulLine) {
    volatile unsigned long ul = 0;

    //    taskENTER_CRITICAL();
    {
        sprintf(exceptionMessage, "\nvAssert @ %s : %li\n", pcFile, ulLine);
        _ExceptionMessage(exceptionMessage);

        /* Set ul to a non-zero value using the debugger to step out of this
        function. */
        while (ul == 0) {
#define COUNT2 2000000
            long i;
            for (i = 0; i < COUNT2; i++);
//            PORTToggleBits(USER_LED_IOPORT, USER_LED_BIT);
        }
    }
    //    taskEXIT_CRITICAL();
}
