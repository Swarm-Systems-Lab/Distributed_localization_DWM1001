#include "thread.h"

THD_FUNCTION(dynamic_thread_funct, arg) {
	th_args *a = (th_args*) arg;
	(a->function)(a->function_arguments);
}

static THD_FUNCTION(static_thread_funct, arg) {
	th_args *a = (th_args*) arg;
	(a->function)(a->function_arguments);
}

thread_t* dynamic_thread(th_args *args, const char th_name[], const tprio_t prio) {

	return chThdCreateFromHeap(NULL, THD_WORKING_AREA_SIZE( sizeof(*args->function)
			+ sizeof(*args->function_arguments)), th_name, prio, dynamic_thread_funct, args);
}

thread_t* static_thread(th_args *args, const char th_name[], const tprio_t prio, void *wsp) {

	return chThdCreateStatic(wsp, sizeof(wsp), prio, static_thread_funct,
			args);
}
