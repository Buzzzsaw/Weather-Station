#pragma once

#ifdef HEADER_H
  #define EXTERN
#else
  #define EXTERN extern
#endif
/*
    Put your ULP globals here you want visibility
    for your sketch. Add "ulp_" to the beginning
    of the variable name and must be size 'uint32_t'
*/
EXTERN uint32_t ulp_entry;

EXTERN uint32_t ulp_next_edge;
EXTERN uint32_t ulp_debounce_counter;
EXTERN uint32_t ulp_debounce_max_count;
EXTERN uint32_t ulp_edge_count;
EXTERN uint32_t ulp_edge_count_to_wake_up;
EXTERN uint32_t ulp_io_number;
