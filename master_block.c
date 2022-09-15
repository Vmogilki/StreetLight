#include <linux/timer.h>
#include <linux/if_arp.h>
#include <linux/time.h>

#include "cbp_base.h"
#include "master_block.h"
#include "control_block.h"

const char* cb_state_to_string[] = {
    FOREACH_CB_STATE(GENERATE_STRING)
};

int packet_stub(struct sk_buff* skb, struct common_block* _cb) {

    u16 cbp_op = ntohs(cbp_hdr(skb)->cbp_op);
    u16 block_type = ntohs(cbp_hdr(skb)->block_type);

    PRINTD(DEBUG_DBG, KERN_INFO DEV_LABEL ": Stub called for packet from MACSRC=%pM [%s] [%s]\n",
        eth_hdr(skb)->h_source, packet_type_to_string[cbp_op], block_type_to_string[block_type]);

    consume_skb(skb);
    return NET_RX_SUCCESS;
}


/*
    Setup display block basing on accumulated sensor data (temperature and brightness)
    and clean accumulated data for next cycle.
*/
static void calculate_average(struct master_part* _mp) {

    if (_mp->count_accum) {
        _mp->data_for_slaves.brightness = _mp->b_accum / _mp->count_accum;

        snprintf(_mp->data_for_slaves.temperature, DISPLAY_TEMPERATURE_LEN, "%+02d °C",
            (int)(_mp->t_accum / _mp->count_accum));

        PRINTD(DEBUG_DBG, KERN_INFO DEV_LABEL ": Average calculated: T=%s, B=%d\n",
            (char*)_mp->data_for_slaves.temperature, _mp->data_for_slaves.brightness);
    }

    _mp->t_accum = _mp->b_accum = _mp->count_accum = 0;
}


/*
*/
static void send_data(struct common_block* _cb) {
    struct timeval current_time; struct tm broken;

    /* Set time (UTC+3) and text. Temp and Bri must be filled by average() */
    do_gettimeofday(&current_time);
    time_to_tm(current_time.tv_sec, 3, &broken);

    snprintf(_cb->mp->data_for_slaves.time, DISPLAY_TIME_LEN, "%02d:%02d:%02d",
        broken.tm_hour, broken.tm_min, broken.tm_sec);

    snprintf(_cb->mp->data_for_slaves.text, DISPLAY_TXT_LEN, "Information message for indication block!");

    /* send set_data to slaves */
    cbp_send_payload(CBP_SET_DATA, _cb->type, &_cb->mp->data_for_slaves, _cb->dev, NULL, NULL);

    /* Next packet after N cycles */
    _cb->mp->set_data_cycles = SET_DATA_CYCLES;
}

/*
    Timer function. Master mode. Check if there are responses from slaves in previous cycle. If yes, then
    calculate average for previous cycle, clean accumulators, send getdata request for next cycle.
*/
static void getdata_cycle_timer_fn(struct timer_list* t) {
    struct common_block* _cb = from_timer(_cb, t, timer);

    spin_lock_bh(&_cb->lock);

    /* Still temp master */
    if (_cb->state == MASTER) {
        /* At least one response has been received from slave(s), then send another get_data request */
        if (_cb->attempts) {

            calculate_average(_cb->mp);

            _cb->attempts = 0;
            mod_timer(&_cb->timer, jiffies + TM_GET_DATA_CYCLE);
            cbp_send_dst(CBP_GET_DATA_REQ, _cb->type, _cb->dev, NULL, NULL);

            /* Send set_data */
            if (!--_cb->mp->set_data_cycles) {
                send_data(_cb);
            }
        }
        else {
            /* seems no slaves. goto in Waiting for Slave state and wait for master_needed request */
            _cb->state = WAITING_FOR_SLAVE;
        }
    }

    spin_unlock_bh(&_cb->lock);
}

/*
    Timer function. Master mode. Send slave_needed request N times.
*/
void slave_needed_sent_timer_fn(struct timer_list* t) {

    struct common_block* _cb = from_timer(_cb, t, timer);

    spin_lock_bh(&_cb->lock);

    if (_cb->state == WAITING_FOR_SLAVE && --_cb->attempts) {
        /* try one more time */
        mod_timer(&_cb->timer, jiffies + TM_SLAVE_NEEDED_SENT);
        cbp_send_dst(CBP_SLAVE_NEEDED_REQ, _cb->type, _cb->dev, NULL, NULL);
    }
    /* Otherwise do nothing. Wait for master needed reqs */
    spin_unlock_bh(&_cb->lock);
}

/*
    Called for CBP_I_AM_SLAVE_REP when IB/CB in Master or Waiting for Slave state
*/
int ph_is_rep_process(struct sk_buff* skb, struct common_block* _cb) {

    /* First slave appears - go to Master mode and schedule get_data reqs */
    if (_cb->state == WAITING_FOR_SLAVE) {
        /* del wait_for_slave timer */
        del_timer(&_cb->timer);

        _cb->state = MASTER;

        /* Iniate get_data request from getdata cycle timer */
        _cb->attempts = 1;

        /* Send set_data after N get_data cycles */
        _cb->mp->set_data_cycles = SET_DATA_CYCLES;

        /* set getdata cycle timer */
        _cb->timer.function = getdata_cycle_timer_fn;
        _cb->timer.expires = jiffies + TM_GET_DATA_CYCLE;
        add_timer(&_cb->timer);
    }

    PRINTD(DEBUG_DBG, KERN_INFO DEV_LABEL": Another slave IB with MAC=%pM\n",
        eth_hdr(skb)->h_source);

    consume_skb(skb);
    return NET_RX_SUCCESS;
}

/*
    Called for CBP_MASTER_NEEDED_REQ when IB in Wait_for_Slave or Master state.
 */
int ph_mn_req_process(struct sk_buff* skb, struct common_block* _cb) {

    int ret = ph_is_rep_process(skb, _cb);
    cbp_send_dst(CBP_I_AM_MASTER_REP, _cb->type, _cb->dev, eth_hdr(skb)->h_source, NULL);
    return ret;
}

/*
    Called for CBP_GET_DATA_REP when IB in Master state
*/
 int ph_gd_rep_process(struct sk_buff* skb, struct common_block* _cb) {
    /* sensor_data is quite small - 4 bytes. Use local variable */
    struct sensor_data data;

    if (!pskb_may_pull(skb, cbp_hdr_len() + sizeof(struct sensor_data))) {

        PRINTD(DEBUG_ERR, KERN_ERR DEV_LABEL": GET DATA response from (MAC=%pM) without sensor_data payload\n",
            eth_hdr(skb)->h_source);

        kfree_skb(skb);
        return NET_RX_DROP;
    }

    unpack_sensor_data(&data, (struct sensor_data*)payload_hdr(skb));

    /* Store data for average calculation */
    store_sensor_data(_cb->mp, &data);

    /* reflect get_data response for get_data_cycle timer */
    _cb->attempts++;

    PRINTD(DEBUG_DBG, KERN_INFO DEV_LABEL": GET DATA response from (MAC=%pM). Temperature=%d, Brightness=%d. Total responses (%d)\n",
        eth_hdr(skb)->h_source, data.temperature, data.brightness, _cb->attempts);


    consume_skb(skb);
    return NET_RX_SUCCESS;
}
