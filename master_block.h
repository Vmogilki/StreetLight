#ifndef _MASTER_BLOCK_H
#define _MASTER_BLOCK_H

/* Timeouts */
#define	TM_SLAVE_NEEDED_SENT    (3*HZ)  /* Master mode */
#define	TM_GET_DATA_CYCLE       (5*HZ)  /* Master mode */

/* Attempts */
#define MAX_SLAVE_NEEDED_REQS   2
#define SET_DATA_CYCLES         6


struct master_part {
    int t_accum;
    int b_accum;
    int count_accum;

    struct display_data data_for_slaves;

    int set_data_cycles;
};

static inline void init_master_part(struct master_part* _mp) {
    _mp->t_accum = 0;
    _mp->b_accum = 0;
    _mp->count_accum = 0;
    _mp->set_data_cycles = 0;
    clean_display_data(&_mp->data_for_slaves);
}

struct slave_part;

struct common_block {
    struct timer_list timer;
    struct net_device* dev;
    int state;
    int type;

    spinlock_t lock;
    int attempts;

    struct master_part* mp;
    struct slave_part* sp;
};

typedef int (*packet_handler_func)(struct sk_buff* skb, struct common_block* _cb);

static inline void store_sensor_data(struct master_part* _mp, struct sensor_data* sns) {
    _mp->t_accum += sns->temperature;
    _mp->b_accum += sns->brightness;
    _mp->count_accum++;
}

int packet_stub(struct sk_buff* skb, struct common_block* _cb);
int ph_is_rep_process(struct sk_buff* skb, struct common_block* _cb);
int ph_mn_req_process(struct sk_buff* skb, struct common_block* _cb);
int ph_gd_rep_process(struct sk_buff* skb, struct common_block* _cb);

void slave_needed_sent_timer_fn(struct timer_list* t);
#endif
