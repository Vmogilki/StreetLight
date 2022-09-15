#ifndef _CONTROL_BLOCK_H
#define _CONTROL_BLOCK_H

/* State of Control Block: No slaves; In Master Mode */
#define FOREACH_CB_STATE(CB_STATE) \
            CB_STATE(WAITING_FOR_SLAVE)   \
            CB_STATE(MASTER)  \
            CB_STATE(CB_NUMBER)     \

typedef enum {
    FOREACH_CB_STATE(GENERATE_ENUM)
} cb_state;

extern const char* cb_state_to_string[];
#endif