#ifndef INC_ADD_H
#define INC_ADD_H

#ifdef ESP_PLATFORM
    #ifndef traceISR_EXIT_TO_SCHEDULER
        #define traceISR_EXIT_TO_SCHEDULER()
    #endif

    #undef _REENT_INIT_PTR
    #define _REENT_INIT_PTR    esp_reent_init
    extern void esp_vApplicationIdleHook( void );
	
    #ifndef traceISR_EXIT
        #define traceISR_EXIT()
    #endif

    #ifndef traceISR_ENTER
        #define traceISR_ENTER( _n_ )
    #endif

    #ifndef traceQUEUE_SEMAPHORE_RECEIVE
        #define traceQUEUE_SEMAPHORE_RECEIVE( pxQueue )
    #endif

    #ifndef traceQUEUE_GIVE_FROM_ISR
        #define traceQUEUE_GIVE_FROM_ISR( pxQueue )
    #endif

    #ifndef traceQUEUE_GIVE_FROM_ISR_FAILED
        #define traceQUEUE_GIVE_FROM_ISR_FAILED( pxQueue )
    #endif
	#define tskNO_AFFINITY  ( 0x7FFFFFFF )
#endif // ESP_PLATFORM

#endif

