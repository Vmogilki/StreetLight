#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/timer.h>
#include <linux/if_arp.h>
#include <linux/time.h>

# include "cbp_base.h"
# include "master_block.h"

/* Timeouts */
#define	TM_MASTER_NEEDED_SENT   (1*HZ)  /* Slave mode */
#define	TM_NO_REQ_FROM_MASTER   (6*TM_GET_DATA_CYCLE)   /* Slave mode */

/* Attempts */
#define MAX_MASTER_NEEDED_REQS  3

/* Module params */
int debug = DEFAULT_DEBUG_LEVEL;
static char* eth_device_name = "ens160";

static struct packet_type cb_packet_type __read_mostly = {
    .type = cpu_to_be16(ETH_P_CBP),
    .func = cbp_rcv,
};


/*
State of Indication Block: No slaves; In Master Mode; No connection to Master; In Slave Mode
First two states similar to CB states (for Master mode):
IB_WAITING_FOR_SLAVE == CB_WAITING_FOR_SLAVE, IB_MASTER == CB_MASTER 
*/
#define FOREACH_IB_STATE(IB_STATE) \
            IB_STATE(WAITING_FOR_SLAVE)   \
            IB_STATE(MASTER)  \
            IB_STATE(WAITING_FOR_MASTER)   \
            IB_STATE(SLAVE)   \
            IB_STATE(IB_NUMBER)     \

typedef enum {
    FOREACH_IB_STATE(GENERATE_ENUM)
} ib_state;

static const char* ib_state_to_string[] = {
    FOREACH_IB_STATE(GENERATE_STRING)
};


static packet_handler_func packet_handlers[CBP_NUMBER][IB_NUMBER];

struct slave_part {
    bool oldest;
    struct sensor_data sensors;
    u8 master_addr[ETH_ALEN];
    cbp_block_type master_type;
};

static struct common_block cb;
static struct master_part mp;
static struct slave_part sp;


static struct sensor_data* read_sensors(struct slave_part* _sp) {

    int tmp;
    get_random_bytes(&tmp, sizeof(tmp));

    _sp->sensors.temperature = tmp % 35;
    _sp->sensors.brightness = 500 + tmp % 100;

    return &_sp->sensors;
}


static inline void display_data_from_master(const struct display_data* pck) {
    PRINTD(DEBUG_INFO, KERN_INFO DEV_LABEL ": Displayed: Time[%s] Info[%s] Temperature[%s] Brightness[%d]\n",
        (char*)pck->time, (char*)pck->text, (char*)pck->temperature, ntohs(pck->brightness));
}


/*
    Timer function. Slave mode. Send master_needed request N times. 
    If no slave with bigger MAC, goto Master mode
*/
static void master_needed_sent_timer_fn(struct timer_list* t) {

    struct common_block* _cb = from_timer(_cb, t, timer);

    spin_lock_bh(&_cb->lock);

	if (_cb->state == WAITING_FOR_MASTER) {
		if (--_cb->attempts) {
            /* try one more time */
            mod_timer(&_cb->timer, jiffies + TM_MASTER_NEEDED_SENT);
			cbp_send_dst(CBP_MASTER_NEEDED_REQ, CBP_DT_SLAVE, _cb->dev, NULL, NULL);
		}
		else if (_cb->sp->oldest) {
			/* Still waiting for master and no older IBs after N attempts, so I'll be temporary master waiting for slaves */
			_cb->state = WAITING_FOR_SLAVE;
			_cb->attempts = MAX_SLAVE_NEEDED_REQS;
			_cb->timer.function = slave_needed_sent_timer_fn;
			_cb->timer.expires = jiffies + TM_SLAVE_NEEDED_SENT;
			add_timer(&_cb->timer);
			cbp_send_dst(CBP_SLAVE_NEEDED_REQ, CBP_DT_MASTER_TEMP, _cb->dev, NULL, NULL);
		}
	}

    spin_unlock_bh(&_cb->lock);
}


static void send_master_needed(struct common_block* _cb) {

    _cb->state = WAITING_FOR_MASTER;
    _cb->sp->oldest = true;

    eth_zero_addr(_cb->sp->master_addr);

    _cb->attempts = MAX_MASTER_NEEDED_REQS;

    _cb->timer.function = master_needed_sent_timer_fn;
    _cb->timer.expires = jiffies + TM_MASTER_NEEDED_SENT;
    add_timer(&_cb->timer);

    cbp_send_dst(CBP_MASTER_NEEDED_REQ, CBP_DT_SLAVE, _cb->dev, NULL, NULL);
}

/*
    Called for CBP_MASTER_NEEDED_REQ when IB in Wait_for_Master or Slave state.
    Check if sender has greater MAC which potentially makes this IB poor candidate for Master role
*/
static int ph_mn_req_ignore(struct sk_buff* skb, struct common_block* _cb) {

    /* check if any IB older than this IB */
    if (ether_addr_greater(eth_hdr(skb)->h_source, skb->dev->dev_addr)) {

        _cb->sp->oldest = false;

        PRINTD(DEBUG_DBG, KERN_INFO DEV_LABEL": Other IB with MACSRC=%pM older than this IB MAC=%pM with state [%s]\n",
            eth_hdr(skb)->h_source, skb->dev->dev_addr, ib_state_to_string[_cb->state]);
    }

    consume_skb(skb);
    return NET_RX_SUCCESS;
}


/*
    Timer function. Slave mode. Send master_needed request after no get_data request 
    within configured interval.
*/
static void no_request_from_master_timer_fn(struct timer_list* t) {

    struct common_block* _cb = from_timer(_cb, t, timer);

    spin_lock_bh(&_cb->lock);

    send_master_needed(_cb);

    spin_unlock_bh(&_cb->lock);
}

/*
    Called for CBP_I_AM_MASTER_REP when IB in Wait_for_Master state. Go to Slave state and set
    timer to control Master availability
*/
static int ph_im_rep_process_s(struct sk_buff* skb, struct common_block* _cb) {

    u16 block_type = ntohs(cbp_hdr(skb)->block_type);

    del_timer(&_cb->timer);

    _cb->state = SLAVE;

    ether_addr_copy(_cb->sp->master_addr, eth_hdr(skb)->h_source);
    _cb->sp->master_type = block_type;

    _cb->attempts = 0;

    _cb->timer.function = no_request_from_master_timer_fn;
    _cb->timer.expires = jiffies + TM_NO_REQ_FROM_MASTER;
    add_timer(&_cb->timer);

    PRINTD(DEBUG_DBG, KERN_INFO DEV_LABEL": New master is set (MAC=%pM TYPE=%s). New state: [%s]\n",
        _cb->sp->master_addr, block_type_to_string[block_type], ib_state_to_string[_cb->state]);

    consume_skb(skb);
    return NET_RX_SUCCESS;
}

/*
    Called for CBP_I_AM_MASTER_REP when IB in Wait_for_Slave or Master state. If packet not from CB, 
    then ignore it (not expected scenario), otherwise go to Slave state and set timer to control Master availability 
*/
static int ph_im_rep_process_m(struct sk_buff* skb, struct common_block* _cb) {

    if (ntohs(cbp_hdr(skb)->block_type) != CBP_DT_MASTER) {
 
        consume_skb(skb);
        return NET_RX_SUCCESS;
    }

    return ph_im_rep_process_s(skb, _cb);

}


/*
    Called for CBP_SLAVE_NEEDED_REQ when IB in Wait_for_Master state
*/
static int ph_sn_req_process_wm(struct sk_buff* skb, struct common_block* _cb) {

    /* No matter if master is temp or not, we go to slave state anyway */
    int ret = ph_im_rep_process_s(skb, _cb);

    /* Send response with confirmation  */
    cbp_send_dst(CBP_I_AM_SLAVE_REP, CBP_DT_SLAVE, _cb->dev, eth_hdr(skb)->h_source, NULL);

    return ret;
}

/*
    Called for CBP_SLAVE_NEEDED_REQ when IB in Slave state
*/
static int ph_sn_req_process_s(struct sk_buff* skb, struct common_block* _cb) {

    /* We ignore packet, if our current master is not temp, or packet from temp master with lower MAC, 
      otherwise set new master with confirmation to it, 
      i.e. if current master is temp and (new master is not temp or has greater MAC). */

    if (_cb->sp->master_type == CBP_DT_MASTER ||
        (ether_addr_greater(_cb->sp->master_addr, eth_hdr(skb)->h_source)
            && ntohs(cbp_hdr(skb)->block_type) == CBP_DT_MASTER_TEMP)) {
    
        consume_skb(skb);
        return NET_RX_SUCCESS;
    }

    return ph_sn_req_process_wm(skb, _cb);
}


/*
    Called for CBP_SLAVE_NEEDED_REQ when IB in Master or Waiting for Slave state
*/
static int ph_sn_req_process_m(struct sk_buff* skb, struct common_block* _cb) {

    /* We ignore packet, if packet from temp master with lower MAC,
      otherwise go to Slave and set new master with confirmation to it,
      i.e. if new master is not temp, or temp but it has greater MAC. */

    if (ether_addr_greater(skb->dev->dev_addr, eth_hdr(skb)->h_source)
            && ntohs(cbp_hdr(skb)->block_type) == CBP_DT_MASTER_TEMP) {

        consume_skb(skb);
        return NET_RX_SUCCESS;
    }

    return ph_sn_req_process_wm(skb, _cb);
}


/*
    Called for CBP_GET_DATA_REQ when IB in Slave state
*/
static int ph_gd_req_process(struct sk_buff* skb, struct common_block* _cb) {

    /* Check Master addr, i.e. the packet from our Master */
    if (ether_addr_equal(eth_hdr(skb)->h_source, _cb->sp->master_addr)) {

        /* process get_data request ... */
        struct sensor_data* sns = read_sensors(_cb->sp);

        PRINTD(DEBUG_DBG, KERN_INFO DEV_LABEL": GET DATA request from (MAC=%pM). Current Temperature=%d, Brightness=%d\n",
            eth_hdr(skb)->h_source, sns->temperature, sns->brightness);

        /* reset no_request_from_master timer */
        mod_timer(&_cb->timer, jiffies + TM_NO_REQ_FROM_MASTER);

        /* send get_data response to master */
        cbp_send_payload(CBP_GET_DATA_REP, CBP_DT_SLAVE, sns, _cb->dev, eth_hdr(skb)->h_source, NULL);
    }

    consume_skb(skb);
    return NET_RX_SUCCESS;
}


/*
    Called for CBP_SET_DATA when IB in Slave state
*/
static int ph_sd_process(struct sk_buff* skb, struct common_block* _cb) {

    struct display_data* dsp;

    if (!pskb_may_pull(skb, cbp_hdr_len() + sizeof(struct display_data))) {

        PRINTD(DEBUG_ERR, KERN_ERR DEV_LABEL": SET DATA response from (MAC=%pM) without display_data payload\n",
            eth_hdr(skb)->h_source);

        kfree_skb(skb);
        return NET_RX_DROP;
    }

    /* Check Master addr, i.e. the packet from our Master */
    if (ether_addr_equal(eth_hdr(skb)->h_source, _cb->sp->master_addr)) {

        /* Display data */
        dsp = (struct display_data*)payload_hdr(skb);

        PRINTD(DEBUG_DBG, KERN_INFO DEV_LABEL": SET DATA from (MAC=%pM).\n",
            eth_hdr(skb)->h_source);

        display_data_from_master(dsp);
    }

    consume_skb(skb);
    return NET_RX_SUCCESS;
}


static inline void init_handler_with_stub(void) {
    int i, j;
    for (i = 0; i < CBP_NUMBER; i++)
        for (j = 0; j < IB_NUMBER; j++)
            packet_handlers[i][j] = packet_stub;
}


static int client_packet_handler(u16 cbp_op, struct sk_buff* skb) {
    
    int ret;

    spin_lock_bh(&cb.lock);

    /* be sure that cb->state < IB_NUMBER!! But we don't receive it through network, so we don't expect incorrect state */
    ret = packet_handlers[cbp_op][cb.state](skb, &cb);

    spin_unlock_bh(&cb.lock);

    return ret;

}

static inline void init_slave_part(struct slave_part* _sp) {
    _sp->oldest = true;
    _sp->master_type = CBP_DT_MASTER;
    eth_zero_addr(_sp->master_addr);
    clean_sensor_data(&_sp->sensors);
}

static void init_indication_block(struct common_block* _cb, struct net_device* dev) {
    _cb->mp = &mp;
    _cb->sp = &sp;

    _cb->dev = dev;
    _cb->attempts = 0;
    _cb->state = WAITING_FOR_MASTER;
    _cb->type = CBP_DT_MASTER_TEMP;

    spin_lock_init(&_cb->lock);
    timer_setup(&_cb->timer, master_needed_sent_timer_fn, 0);

    init_master_part(_cb->mp);
    init_slave_part(_cb->sp);
   
    init_handler_with_stub();
    packet_handlers[CBP_MASTER_NEEDED_REQ][WAITING_FOR_MASTER] = 
    packet_handlers[CBP_MASTER_NEEDED_REQ][SLAVE] = ph_mn_req_ignore;
    packet_handlers[CBP_MASTER_NEEDED_REQ][WAITING_FOR_SLAVE] =
    packet_handlers[CBP_MASTER_NEEDED_REQ][MASTER] = ph_mn_req_process;

    packet_handlers[CBP_I_AM_MASTER_REP][WAITING_FOR_MASTER] = ph_im_rep_process_s;
    packet_handlers[CBP_I_AM_MASTER_REP][WAITING_FOR_SLAVE] = 
    packet_handlers[CBP_I_AM_MASTER_REP][MASTER] = ph_im_rep_process_m;

    packet_handlers[CBP_SLAVE_NEEDED_REQ][WAITING_FOR_MASTER] = ph_sn_req_process_wm;
    packet_handlers[CBP_SLAVE_NEEDED_REQ][SLAVE] = ph_sn_req_process_s;
    packet_handlers[CBP_SLAVE_NEEDED_REQ][WAITING_FOR_SLAVE] = 
    packet_handlers[CBP_SLAVE_NEEDED_REQ][MASTER] = ph_sn_req_process_m;
   
    packet_handlers[CBP_I_AM_SLAVE_REP][WAITING_FOR_SLAVE] =
    packet_handlers[CBP_I_AM_SLAVE_REP][MASTER] = ph_is_rep_process;

    packet_handlers[CBP_GET_DATA_REQ][SLAVE] = ph_gd_req_process;

    packet_handlers[CBP_GET_DATA_REP][MASTER] = ph_gd_rep_process;

    packet_handlers[CBP_SET_DATA][SLAVE] = ph_sd_process;
}


extern void cb_module(void);
void ib_module(void) {}
EXPORT_SYMBOL(ib_module);


static int __init init_ib(void) {
    
    struct net_device* eth_dev = NULL;

    PRINTD(DEBUG_INFO, KERN_INFO DEV_LABEL ": Setup Client Block Module\n");

    if (symbol_get(cb_module)) {
        PRINTD(DEBUG_ERR, KERN_ERR DEV_LABEL ": Cannot load module. CB instance has been already loaded!\n");
        return -EEXIST;
    }


    eth_dev = dev_get_by_name(&init_net, eth_device_name);

    if (!eth_dev) {
        PRINTD(DEBUG_ERR, KERN_ERR DEV_LABEL ": Cannot open %s\n", eth_device_name);
        return -ENODEV;
    }
    else if (eth_dev->type != ARPHRD_ETHER) {
            PRINTD(DEBUG_ERR, KERN_ERR DEV_LABEL ": %s - not an Ethernet device\n", eth_device_name);
            return -ENODEV;
        }
        else {
            PRINTD(DEBUG_INFO, KERN_INFO DEV_LABEL ": Open %s\n", eth_device_name);

            cbp_packet_handler = client_packet_handler;

            init_indication_block(&cb, eth_dev);

            dev_add_pack(&cb_packet_type);

            send_master_needed(&cb);
        }
    
    return 0;
}

static void __exit exit_ib(void) {

    if (cb.dev) {
        del_timer(&cb.timer);

        dev_remove_pack(&cb_packet_type);
        dev_put(cb.dev);
        PRINTD(DEBUG_INFO, KERN_INFO DEV_LABEL ": Release %s\n", eth_device_name);
    }

    PRINTD(DEBUG_INFO, KERN_INFO DEV_LABEL ": Cleanup Client Block Module\n");
    return;
}

module_init(init_ib);
module_exit(exit_ib);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("V.Mogilkin <viktor.mogilkin@yandex.ru>");
MODULE_DESCRIPTION("StreetLight Indication Module");
MODULE_VERSION("2.00");

module_param(debug, int, 0);
MODULE_PARM_DESC(debug, "Debugging messages level");

module_param(eth_device_name, charp, 0);
MODULE_PARM_DESC(debug, "Ethernet device name");
