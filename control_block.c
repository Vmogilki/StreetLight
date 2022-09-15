#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/if_arp.h>

#include "cbp_base.h"
#include "master_block.h"
#include "control_block.h"

/* Module params */
int debug = DEFAULT_DEBUG_LEVEL;
static char* eth_device_name = "ens160";
static bool cleanup_needed = false;

static struct packet_type cb_packet_type __read_mostly = {
    .type = cpu_to_be16(ETH_P_CBP),
    .func = cbp_rcv,
};

static struct common_block cb;
static struct master_part mp;

static packet_handler_func packet_handlers[CBP_NUMBER][CB_NUMBER];

static void send_slave_needed(struct common_block* _cb) {

    _cb->state = WAITING_FOR_SLAVE;

    _cb->attempts = MAX_SLAVE_NEEDED_REQS;

    _cb->timer.function = slave_needed_sent_timer_fn;
    _cb->timer.expires = jiffies + TM_SLAVE_NEEDED_SENT;
    add_timer(&_cb->timer);

    cbp_send_dst(CBP_SLAVE_NEEDED_REQ, CBP_DT_MASTER, _cb->dev, NULL, NULL);
}

/*
    Called for CBP_SLAVE_NEEDED_REQ when CB in Master or Waiting for Slave state
*/
static int ph_sn_req_process(struct sk_buff* skb, struct common_block* _cb) {

    /* We reply to sender with CBP_I_AM_MASTER_REP. If sender is CBP_DT_MASTER_TEMP (i.e. IB), 
    it must go to Slave state, if sender is CBP_DT_MASTER (i.e. CB), the behavior is currently
    undefined since we cannot have more than one CB in network. In such a case to avoid races between 
    CBs we may stop listening network by that module and indicate this in system logs as temp solution */

    cbp_send_dst(CBP_I_AM_MASTER_REP, CBP_DT_MASTER, _cb->dev, eth_hdr(skb)->h_source, NULL);

    consume_skb(skb);
    return NET_RX_SUCCESS;
}

/*
    Called for CBP_I_AM_MASTER_REP when CB in Master or Waiting for Slave state
*/
static int ph_im_rep_process(struct sk_buff* skb, struct common_block* _cb) {

    const struct cbphdr* cbp = cbp_hdr(skb);

    /* Packet is expected only from CB. IB in Master state should not send it, but go to Slave instead. 
    So we ignore such packet. */
    
    if (ntohs(cbp->block_type) == CBP_DT_MASTER_TEMP) {

        PRINTD(DEBUG_WARN, KERN_WARNING DEV_LABEL ": Warning! Unexpected [%s] [CBP_DT_MASTER_TEMP] received from MACSRC=%pM\n",
            packet_type_to_string[ntohs(cbp->cbp_op)], eth_hdr(skb)->h_source);
        goto end_im_rep;
    }

    PRINTD(DEBUG_WARN, KERN_WARNING DEV_LABEL ": Warning! Another CB is detected in network. Unexpected [%s] [CBP_DT_MASTER] received from MACSRC=%pM\n",
        packet_type_to_string[ntohs(cbp->cbp_op)], eth_hdr(skb)->h_source);

    /* We may consider of stopping listening network by this module (to avoid races) and indicate it in system logs as temp solution. */
    if (cleanup_needed) {
        del_timer(&_cb->timer);

        dev_remove_pack(&cb_packet_type);
        dev_put(_cb->dev);
        PRINTD(DEBUG_INFO, KERN_INFO DEV_LABEL ": Release %s\n", eth_device_name);

        cleanup_needed = false;

        PRINTD(DEBUG_WARN, KERN_WARNING DEV_LABEL ": Warning! The module has been cleaned and must be unloaded/reloaded.\n");
    }
end_im_rep:
    consume_skb(skb);
    return NET_RX_SUCCESS;
}

static int client_packet_handler(u16 cbp_op, struct sk_buff* skb) {

    int ret;

    spin_lock_bh(&cb.lock);

    /* be sure that cb->state < CB_NUMBER!! But we don't receive it through network, so we don't expect incorrect state */
    ret = packet_handlers[cbp_op][cb.state](skb, &cb);

    spin_unlock_bh(&cb.lock);

    return ret;
}


static inline void init_handler_with_stub(void) {
    int i, j;
    for (i = 0; i < CBP_NUMBER; i++)
        for (j = 0; j < CB_NUMBER; j++)
            packet_handlers[i][j] = packet_stub;
}


static void init_control_block(struct common_block* _cb, struct net_device* dev) {
    _cb->mp = &mp;
    _cb->sp = NULL;

    _cb->dev = dev;
    _cb->attempts = 0;
    _cb->state = WAITING_FOR_SLAVE;
    _cb->type = CBP_DT_MASTER;

    spin_lock_init(&_cb->lock);
    timer_setup(&_cb->timer, slave_needed_sent_timer_fn, 0);

    init_master_part(_cb->mp);

    init_handler_with_stub();
    packet_handlers[CBP_MASTER_NEEDED_REQ][WAITING_FOR_SLAVE] =
    packet_handlers[CBP_MASTER_NEEDED_REQ][MASTER] = ph_mn_req_process;

    packet_handlers[CBP_SLAVE_NEEDED_REQ][WAITING_FOR_SLAVE] =
    packet_handlers[CBP_SLAVE_NEEDED_REQ][MASTER] = ph_sn_req_process;

    packet_handlers[CBP_I_AM_MASTER_REP][WAITING_FOR_SLAVE] =
    packet_handlers[CBP_I_AM_MASTER_REP][MASTER] = ph_im_rep_process;

    packet_handlers[CBP_I_AM_SLAVE_REP][WAITING_FOR_SLAVE] =
    packet_handlers[CBP_I_AM_SLAVE_REP][MASTER] = ph_is_rep_process;

    packet_handlers[CBP_GET_DATA_REP][MASTER] = ph_gd_rep_process;
}


static int __init init_cb(void) {

    struct net_device* eth_dev = NULL;

    PRINTD(DEBUG_INFO, KERN_INFO DEV_LABEL ": Setup Control Block Module\n");

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

        init_control_block(&cb, eth_dev);

        dev_add_pack(&cb_packet_type);

        send_slave_needed(&cb);

        cleanup_needed = true;
    }

    return 0;
}

void __exit exit_cb(void) {

    if (cb.dev && cleanup_needed) {
        del_timer(&cb.timer);

        dev_remove_pack(&cb_packet_type);
        dev_put(cb.dev);
        PRINTD(DEBUG_INFO, KERN_INFO DEV_LABEL ": Release %s\n", eth_device_name);
    }

    PRINTD(DEBUG_INFO, KERN_INFO DEV_LABEL ": Cleanup Control Block Module\n");
    return;
}

module_init(init_cb);
module_exit(exit_cb);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("V.Mogilkin <viktor.mogilkin@yandex.ru>");
MODULE_DESCRIPTION("StreetLight Control Module");
MODULE_VERSION("2.00");

module_param(debug, int, 0);
MODULE_PARM_DESC(debug, "Debugging messages level");

module_param(eth_device_name, charp, 0);
MODULE_PARM_DESC(debug, "Ethernet device name");