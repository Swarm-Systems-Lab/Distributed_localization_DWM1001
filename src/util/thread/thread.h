#ifndef _THREAD_
#define _THREAD_

#include "ch.h"
#include <stddef.h>

/* Struct that threds must receive where
   is defined the function that the thread
   will execute and the arguments of the function. */
typedef struct {
	void (*function)(void*);
	void* function_arguments;
}th_args;

// Create and launch a dynamic thread.
thread_t* dynamic_thread(th_args* args, const char th_name[], const tprio_t prio);

// Creates and launch a static thread.
thread_t* static_thread(th_args* args, const char th_name[], const tprio_t prio, void *wsp);

#endif // _THREAD_
