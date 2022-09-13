#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/timer.h>
#include <linux/if_arp.h>
#include <linux/time.h>


/* Module label */
#define DEV_LABEL "IB"

/* Debug printing */
#define PRINTD(level,format,args...) \
   (((level) & debug) ? printk(format, ## args) : 1)

#define DEBUG_NONE   0x00
#define DEBUG_ERR    0x01
#define DEBUG_WARN   0x02
#define DEBUG_INFO   0x04
#define DEBUG_DBG    0x08
#define DEBUG_ANY    0x0f
#define DEFAULT_DEBUG_LEVEL (DEBUG_ERR | DEBUG_WARN | DEBUG_INFO)

/* Custom packet type Client/Control Block Protocol (CBP) */
#define ETH_P_CBP        0x5050                

/* Timeouts */
#define	TM_SLAVE_NEEDED_SENT    (3*HZ)  /* Master mode */
#define	TM_GET_DATA_CYCLE       (5*HZ)  /* Master mode */
#define	TM_MASTER_NEEDED_SENT   (1*HZ)  /* Slave mode */
#define	TM_NO_REQ_FROM_MASTER   (6*TM_GET_DATA_CYCLE)   /* Slave mode */

/* Attempts */
#define MAX_MASTER_NEEDED_REQS  3
#define MAX_SLAVE_NEEDED_REQS   2
#define SET_DATA_CYCLES         6

/* Module params */
static int debug = DEFAULT_DEBUG_LEVEL;
static char* eth_device_name = "ens160";

/* CBP header */
struct __attribute__((__packed__)) cbphdr {
    __be16		cbp_op;		    
    __be16		block_type;		
};

/* Sensor data reported by Slaves. CBP_GET_DATA_REP packet contains cbphdr + sensor_data */
struct __attribute__((__packed__)) sensor_data {
    short int   temperature;
    u16		    brightness;
};

/* Pack sensor data into packet */
inline void pack_sensor_data(struct sensor_data* pkt, const struct sensor_data* src) {
    pkt->temperature = htons(src->temperature);
    pkt->brightness = htons(src->brightness);
}

/* Unpack sensor data from packet */
inline void unpack_sensor_data(struct sensor_data* dst, const struct sensor_data* pkt) {
    dst->temperature = ntohs(pkt->temperature);
    dst->brightness = ntohs(pkt->brightness);
}


#define DISPLAY_TXT_LEN             45  /* Text string ended by trailing zero */
#define DISPLAY_TEMPERATURE_LEN     8   /* Format [+/-]TT °C and trailing zero */
#define DISPLAY_TIME_LEN            9   /* Format HH:MM:SS and trailing zero */

/* Display data sent by Master. CBP_SET_DATA packet contains cbphdr + display_data */
struct __attribute__((__packed__)) display_data {
    u16 brightness;
    char text[DISPLAY_TXT_LEN];
    char temperature[DISPLAY_TEMPERATURE_LEN];
    char time[DISPLAY_TIME_LEN];
};

/* Pack display data into packet */
inline void pack_display_data(struct display_data* pkt, const struct display_data* src) {
    pkt->brightness = htons(src->brightness);
    memcpy(pkt->text, src->text, sizeof(pkt->text));
    memcpy(pkt->temperature, src->temperature, sizeof(pkt->temperature));
    memcpy(pkt->time, src->time, sizeof(pkt->time));
}

/* Unpack display data from packet */
inline void unpack_display_data(struct display_data* dst, const struct display_data* pkt) {
    dst->brightness = ntohs(pkt->brightness);
    memcpy(dst->text, pkt->text, sizeof(pkt->text));
    memcpy(dst->temperature, pkt->temperature, sizeof(pkt->temperature));
    memcpy(dst->time, pkt->time, sizeof(pkt->time));
}




/* Packets - protocol codes */
#define FOREACH_PACKET_TYPE(PACKET_TYPE) \
            PACKET_TYPE(CBP_MASTER_NEEDED_REQ)   \
            PACKET_TYPE(CBP_I_AM_MASTER_REP)  \
            PACKET_TYPE(CBP_SLAVE_NEEDED_REQ)   \
            PACKET_TYPE(CBP_I_AM_SLAVE_REP)  \
            PACKET_TYPE(CBP_GET_DATA_REQ)  \
            PACKET_TYPE(CBP_GET_DATA_REP)  \
            PACKET_TYPE(CBP_SET_DATA)   \
            PACKET_TYPE(CBP_NUMBER)     \

/* State of Indication Block: No connection to Master; In Master Mode;  In Slave Mode */
#define FOREACH_STATE(STATE) \
            STATE(IB_WAITING_FOR_MASTER)   \
            STATE(IB_SLAVE)   \
            STATE(IB_WAITING_FOR_SLAVE)   \
            STATE(IB_MASTER)  \
            STATE(IB_NUMBER)     \

/* Device mode in packet: Indication Block in Slave mode; Control Block; Indication Block in Master mode */
#define FOREACH_BLOCK_TYPE(BLOCK_TYPE) \
            BLOCK_TYPE(CBP_DT_MASTER)   \
            BLOCK_TYPE(CBP_DT_SLAVE)   \
            BLOCK_TYPE(CBP_DT_MASTER_TEMP)   \
            BLOCK_TYPE(CBP_DT_NUMBER)   \

#define GENERATE_ENUM(ENUM) ENUM,
#define GENERATE_STRING(STRING) #STRING,

typedef enum {
    FOREACH_PACKET_TYPE(GENERATE_ENUM)
} cbp_paket_type;

static const char *packet_type_to_string[] = {
    FOREACH_PACKET_TYPE(GENERATE_STRING)
};

typedef enum {
    FOREACH_STATE(GENERATE_ENUM)
} ib_state;

static const char* state_to_string[] = {
    FOREACH_STATE(GENERATE_STRING)
};

typedef enum {
    FOREACH_BLOCK_TYPE(GENERATE_ENUM)
} cbp_block_type;

static const char* block_type_to_string[] = {
    FOREACH_BLOCK_TYPE(GENERATE_STRING)
};


static inline bool is_packet_block_correct(cbp_paket_type packet, cbp_block_type block) {
    switch (packet) {
            /* slave packets */
            case CBP_MASTER_NEEDED_REQ:
            case CBP_I_AM_SLAVE_REP:
            case CBP_GET_DATA_REP:
                return (block == CBP_DT_SLAVE) ? true : false;

            /* master packets */
            case CBP_I_AM_MASTER_REP:
            case CBP_SLAVE_NEEDED_REQ:
            case CBP_GET_DATA_REQ:
            case CBP_SET_DATA:
                return (block == CBP_DT_SLAVE) ? false : true;
            
            /* wrong packet type */
            default: 
                return false;
    }
}

struct indication_block;

typedef int (*packet_handler_func)(struct sk_buff* skb, struct indication_block* _ib);

struct indication_block {
    /* Common Part*/
    struct timer_list timer;
    struct net_device* dev;
    ib_state state;
    spinlock_t lock;
    bool oldest;
    int attempts;
    packet_handler_func packet_handlers[CBP_NUMBER][IB_NUMBER];

    /* Slave Part */
    struct sensor_data sensors;

    /* Master Part */
    int t_accum;
    int b_accum;
    int count_accum;

    struct display_data data_for_slaves;

    int set_data_cycles;

    u8 master_addr[ETH_ALEN];
    cbp_block_type master_type;
} ib;


static inline struct cbphdr* cbp_hdr(const struct sk_buff* skb) {
    return (struct cbphdr*)skb_network_header(skb);
}

static inline int cbp_hdr_len(void) {
    return sizeof(struct cbphdr);
}

static inline void* payload_hdr(const struct sk_buff* skb) {
    return (void*)skb_network_header(skb) + cbp_hdr_len();
}

static inline int cbp_payload_len(cbp_paket_type packet) {
    if (packet != CBP_GET_DATA_REP && packet != CBP_SET_DATA)
        return 0;

    return (packet == CBP_GET_DATA_REP) ? 
        sizeof(struct sensor_data) : 
        sizeof(struct display_data);
}

static inline void pack_cbp_payload(void* pkt, void* src, cbp_paket_type packet) {
    switch (packet) {
    case CBP_GET_DATA_REP:
        pack_sensor_data((struct sensor_data*) pkt, (struct sensor_data*) src);
        break;
    case CBP_SET_DATA:
        pack_display_data((struct display_data*) pkt, (struct display_data*) src);
        break;
    default:
        break;
    }
}

static struct sensor_data* read_sensors(struct indication_block* _ib) {

    int tmp;
    get_random_bytes(&tmp, sizeof(tmp));

    _ib->sensors.temperature = tmp % 35;
    _ib->sensors.brightness = 500 + tmp % 100;

    return &_ib->sensors;
}


static inline void store_sensor_data(struct indication_block* _ib, struct sensor_data* sns) {
    _ib->t_accum += sns->temperature;
    _ib->b_accum += sns->brightness;
    _ib->count_accum++;
}

/*
    Setup display block basing on accumulated sensor data (temperature and brightness)
    and clean accumulated data for next cycle.
*/
static void calculate_average(struct indication_block* _ib) {

    if (_ib->count_accum) {
        _ib->data_for_slaves.brightness = _ib->b_accum / _ib->count_accum;

        snprintf(_ib->data_for_slaves.temperature, DISPLAY_TEMPERATURE_LEN, "%+02d °C",
            (int)(_ib->t_accum / _ib->count_accum));

        PRINTD(DEBUG_DBG, KERN_INFO DEV_LABEL ": Average calculated: T=%s, B=%d\n",
            (char*)_ib->data_for_slaves.temperature, _ib->data_for_slaves.brightness);
    }

    _ib->t_accum = _ib->b_accum = _ib->count_accum = 0;
}


static inline void display_data_from_master(const struct display_data* pck) {
    PRINTD(DEBUG_INFO, KERN_INFO DEV_LABEL ": Displayed: Time[%s] Info[%s] Temperature[%s] Brightness[%d]\n",
        (char*)pck->time, (char*)pck->text, (char*)pck->temperature, ntohs(pck->brightness));
}


static int packet_stub(struct sk_buff* skb, struct indication_block* _ib) {

    u16 cbp_op = ntohs(cbp_hdr(skb)->cbp_op);
    u16 block_type = ntohs(cbp_hdr(skb)->block_type);

    PRINTD(DEBUG_DBG, KERN_INFO DEV_LABEL ": Stub called for packet from MACSRC=%pM [%s] [%s] with IB [%s]\n",
        eth_hdr(skb)->h_source, packet_type_to_string[cbp_op], 
        block_type_to_string[block_type], state_to_string[_ib->state]);

    consume_skb(skb);
    return NET_RX_SUCCESS;
}


/* return true, if eth addr1 > eth addr2 */
static inline bool ether_addr_greater(const u8* addr1, const u8* addr2)
{
    /* temp solution: maybe not the most efficient way - bytes comparing */
    int i;

    for (i = 0; i < ETH_ALEN; i++)
        if (addr1[i] > addr2[i])
            return true;
        else if (addr1[i] < addr2[i])
            return false;
    
    return false;
}


static struct sk_buff* cbp_create(int op_command, int block_type, 
	void* payload, struct net_device* dev, 
	const unsigned char* dest_hw,
	const unsigned char* src_hw)
{
	struct sk_buff* skb;
	struct cbphdr* cbp;
    void* pkt_payload;
    	
	int hlen = LL_RESERVED_SPACE(dev);
	int tlen = dev->needed_tailroom;

	skb = alloc_skb(cbp_hdr_len() + cbp_payload_len(op_command) + hlen + tlen, GFP_ATOMIC);
	if (!skb)
		return NULL;

	skb_reserve(skb, hlen);
	skb_reset_network_header(skb);
    
    if (payload && cbp_payload_len(op_command)) {
        
        cbp = skb_put(skb, cbp_hdr_len() + cbp_payload_len(op_command));
        pkt_payload = (void*) (cbp + 1);

        /* we have payload */
        pack_cbp_payload(pkt_payload, payload, op_command);
    } else {
        cbp = skb_put(skb, cbp_hdr_len());
    }

	skb->dev = dev;
	skb->protocol = htons(ETH_P_CBP);
	if (!src_hw)
		src_hw = dev->dev_addr;
	if (!dest_hw)
		dest_hw = dev->broadcast;

	if (dev_hard_header(skb, dev, ETH_P_CBP, dest_hw, src_hw, skb->len) < 0)
		goto out;

	cbp->cbp_op = htons(op_command);
	cbp->block_type = htons(block_type);
 	
	return skb;

out:
	kfree_skb(skb);
	return NULL;
}

/* Create and send CBP packet with payload */
static void cbp_send_payload(int op, int bt, void* payload, struct net_device* dev,
    const unsigned char* dest_hw, const unsigned char* src_hw)
{
    struct sk_buff* skb;
       
    /* Packet <-> Block correctness. To avoid creating packet/block by mistake that will be ignored by receiver anyway */
    if (!is_packet_block_correct(op, bt)) {
        PRINTD(DEBUG_WARN, KERN_WARNING DEV_LABEL ": Warning! Attempt to create incorrect [%s] - [%s]\n",
            packet_type_to_string[op], block_type_to_string[bt]);
        return;
    }

    /* payload must be not NULL for packet types with payload */
    if (cbp_payload_len(op))
        if (!payload)
            return;

    skb = cbp_create(op, bt, payload, dev, dest_hw, src_hw);

    if (!skb)
        return;

    dev_queue_xmit(skb);

    PRINTD(DEBUG_DBG, KERN_INFO DEV_LABEL ": Sent MACSRC=%pM MACDST=%pM MACPROTO=%04x [%s] [%s]\n",
        eth_hdr(skb)->h_source, eth_hdr(skb)->h_dest,
        ntohs(eth_hdr(skb)->h_proto), packet_type_to_string[op], block_type_to_string[bt]);

    return;
}

/* Common send variant for most CBP packets without payload to exlude silly mistakes */
static inline void cbp_send_dst(int op, int bt, struct net_device* dev,
    const unsigned char* dest_hw, const unsigned char* src_hw) {
    cbp_send_payload(op, bt, NULL, dev, dest_hw, src_hw);
}

/*
Process incoming CBP packet. Use global structure IB to reflect current IB state
*/
static int cbp_process(struct sk_buff* skb) {
    
    const struct cbphdr* cbp = cbp_hdr(skb);
    u16 cbp_op = ntohs(cbp->cbp_op);
    u16 block_type = ntohs(cbp->block_type);
    int ret;
    
    /* Check CBP header, values might be damaged */
    if (cbp_op >= CBP_NUMBER || block_type >= CBP_DT_NUMBER)
        goto bad_packet;

    /* Packet <-> Block correctness */
    if (!is_packet_block_correct(cbp_op, block_type))
        goto bad_packet;

    PRINTD(DEBUG_DBG, KERN_INFO DEV_LABEL ": Received MACSRC=%pM MACDST=%pM MACPROTO=%04x [%s] [%s]\n",
        eth_hdr(skb)->h_source, eth_hdr(skb)->h_dest,
        ntohs(eth_hdr(skb)->h_proto), packet_type_to_string[cbp_op], block_type_to_string[block_type]);

    spin_lock_bh(&ib.lock);

    /* be sure that ib->state < IB_NUMBER!! But we don't receive it through network, so we don't expect incorrect state */
    ret = ib.packet_handlers[cbp_op][ib.state](skb, &ib);
    
    spin_unlock_bh(&ib.lock);
    
    return ret;         
    
 bad_packet:
     PRINTD(DEBUG_WARN, KERN_WARNING DEV_LABEL ": Warning! Bad packet received MACSRC=%pM MACDST=%pM MACPROTO=%04x PT=%02d DT=%02d\n",
         eth_hdr(skb)->h_source, eth_hdr(skb)->h_dest,
         ntohs(eth_hdr(skb)->h_proto), cbp_op, block_type);

    kfree_skb(skb);
    return NET_RX_DROP;
}


static int cbp_rcv(struct sk_buff* skb, struct net_device* dev,
    struct packet_type* pt, struct net_device* orig_dev)
{
    if (skb->pkt_type == PACKET_OTHERHOST ||
        skb->pkt_type == PACKET_LOOPBACK)
        goto consumeskb;

    skb = skb_share_check(skb, GFP_ATOMIC);
    if (!skb)
        goto out_of_mem;

    if (!pskb_may_pull(skb, cbp_hdr_len()))
        goto freeskb;

    return cbp_process(skb);

consumeskb:
    consume_skb(skb);
    return NET_RX_SUCCESS;

freeskb:
    kfree_skb(skb);
out_of_mem:
    return NET_RX_DROP;
}

/*
*/
static void send_data(struct indication_block* _ib) {
    struct timeval current_time; struct tm broken;
    
    /* Set time and text. Temp and Bri must be filled by average() */
    do_gettimeofday(&current_time);
    time_to_tm(current_time.tv_sec, 0, &broken);
 
    snprintf(_ib->data_for_slaves.time, DISPLAY_TIME_LEN, "%02d:%02d:%02d",
        broken.tm_hour, broken.tm_min, broken.tm_sec);

    snprintf(_ib->data_for_slaves.text, DISPLAY_TXT_LEN, "Information message for indication block!");

    /* send set_data to slaves */
    cbp_send_payload(CBP_SET_DATA, CBP_DT_MASTER_TEMP, &_ib->data_for_slaves, _ib->dev, NULL, NULL);

    /* Next packet after N cycles */
    _ib->set_data_cycles = SET_DATA_CYCLES;
}

/* 
    Timer function. Master mode. Check if there are responses from slaves in previous cycle. If yes, then
    calculate average for previous cycle, clean accumulators, send getdata request for next cycle.
*/
static void getdata_cycle_timer_fn(struct timer_list* t) {
    struct indication_block* _ib = from_timer(_ib, t, timer);

    spin_lock_bh(&_ib->lock);

    /* Still temp master */
    if (_ib->state == IB_MASTER) {
        /* At least one response has been received from slave(s), then send another get_data request */
        if (_ib->attempts) {

            calculate_average(_ib);

            _ib->attempts = 0;
            mod_timer(&_ib->timer, jiffies + TM_GET_DATA_CYCLE);
            cbp_send_dst(CBP_GET_DATA_REQ, CBP_DT_MASTER_TEMP, _ib->dev, NULL, NULL);

            /* Send set_data */
            if (!--_ib->set_data_cycles) {
                send_data(_ib);           
            }
        }
        else {
        /* seems no slaves. goto in Waiting for Slave state and wait for master_needed request */
            _ib->state = IB_WAITING_FOR_SLAVE;
        }
    }
   
    spin_unlock_bh(&_ib->lock);
}

/*
    Timer function. Master mode. Send slave_needed request N times.
*/
static void slave_needed_sent_timer_fn(struct timer_list* t) {

    struct indication_block* _ib = from_timer(_ib, t, timer);

    spin_lock_bh(&_ib->lock);

    if (_ib->state == IB_WAITING_FOR_SLAVE && --_ib->attempts) {
            /* try one more time */
            mod_timer(&_ib->timer, jiffies + TM_SLAVE_NEEDED_SENT);
            cbp_send_dst(CBP_SLAVE_NEEDED_REQ, CBP_DT_MASTER_TEMP, _ib->dev, NULL, NULL);
        }
    /* Otherwise do nothing. Wait for master needed reqs */
    spin_unlock_bh(&_ib->lock);
}


/*
    Timer function. Slave mode. Send master_needed request N times. 
    If no slave with bigger MAC, goto Master mode
*/
static void master_needed_sent_timer_fn(struct timer_list* t) {

    struct indication_block* _ib = from_timer(_ib, t, timer);

    spin_lock_bh(&_ib->lock);

	if (_ib->state == IB_WAITING_FOR_MASTER) {
		if (--_ib->attempts) {
            /* try one more time */
            mod_timer(&_ib->timer, jiffies + TM_MASTER_NEEDED_SENT);
			cbp_send_dst(CBP_MASTER_NEEDED_REQ, CBP_DT_SLAVE, _ib->dev, NULL, NULL);
		}
		else if (_ib->oldest) {
			/* Still waiting for master and no older IBs after N attempts, so I'll be temporary master waiting for slaves */
			_ib->state = IB_WAITING_FOR_SLAVE;
			_ib->attempts = MAX_SLAVE_NEEDED_REQS;
			_ib->timer.function = slave_needed_sent_timer_fn;
			_ib->timer.expires = jiffies + TM_SLAVE_NEEDED_SENT;
			add_timer(&_ib->timer);
			cbp_send_dst(CBP_SLAVE_NEEDED_REQ, CBP_DT_MASTER_TEMP, _ib->dev, NULL, NULL);
		}
	}

    spin_unlock_bh(&_ib->lock);
}


static void send_master_needed(struct indication_block* _ib) {

    _ib->state = IB_WAITING_FOR_MASTER;
    _ib->oldest = true;

    eth_zero_addr(_ib->master_addr);

    _ib->attempts = MAX_MASTER_NEEDED_REQS;

    _ib->timer.function = master_needed_sent_timer_fn;
    _ib->timer.expires = jiffies + TM_MASTER_NEEDED_SENT;
    add_timer(&_ib->timer);

    cbp_send_dst(CBP_MASTER_NEEDED_REQ, CBP_DT_SLAVE, _ib->dev, NULL, NULL);
}

/*
    Called for CBP_MASTER_NEEDED_REQ when IB in Wait_for_Master or Slave state.
    Check if sender has greater MAC which potentially makes this IB poor candidate for Master role
*/
static int ph_mn_req_ignore(struct sk_buff* skb, struct indication_block* _ib) {

    /* check if any IB older than this IB */
    if (ether_addr_greater(eth_hdr(skb)->h_source, skb->dev->dev_addr)) {

        _ib->oldest = false;

        PRINTD(DEBUG_DBG, KERN_INFO DEV_LABEL": Other IB with MACSRC=%pM older than this IB MAC=%pM with state [%s]\n",
            eth_hdr(skb)->h_source, skb->dev->dev_addr, state_to_string[_ib->state]);
    }

    consume_skb(skb);
    return NET_RX_SUCCESS;
}

/*
    Called for CBP_I_AM_SLAVE_REP when IB in Master or Waiting for Slave state
*/
static int ph_is_rep_process(struct sk_buff* skb, struct indication_block* _ib) {

    /* First slave appears - go to Master mode and schedule get_data reqs */
    if (_ib->state == IB_WAITING_FOR_SLAVE) {
        /* del wait_for_slave timer */
        del_timer(&_ib->timer);

        _ib->state = IB_MASTER;

        /* Iniate get_data request from getdata cycle timer */
        _ib->attempts = 1;

        /* Send set_data after N get_data cycles */
        _ib->set_data_cycles = SET_DATA_CYCLES;

        /* set getdata cycle timer */
        _ib->timer.function = getdata_cycle_timer_fn;
        _ib->timer.expires = jiffies + TM_GET_DATA_CYCLE;
        add_timer(&_ib->timer);
    }

    PRINTD(DEBUG_DBG, KERN_INFO DEV_LABEL": Another slave IB with MAC=%pM\n",
        eth_hdr(skb)->h_source);

    consume_skb(skb);
    return NET_RX_SUCCESS;
}

/*
    Called for CBP_MASTER_NEEDED_REQ when IB in Wait_for_Slave or Master state.
 */
static int ph_mn_req_process(struct sk_buff* skb, struct indication_block* _ib) {

    int ret = ph_is_rep_process(skb, _ib);
    cbp_send_dst(CBP_I_AM_MASTER_REP, CBP_DT_MASTER_TEMP, _ib->dev, eth_hdr(skb)->h_source, NULL);
    return ret;
}

/*
    Timer function. Slave mode. Send master_needed request after no get_data request 
    within configured interval.
*/
static void no_request_from_master_timer_fn(struct timer_list* t) {

    struct indication_block* _ib = from_timer(_ib, t, timer);

    spin_lock_bh(&_ib->lock);

    send_master_needed(_ib);

    spin_unlock_bh(&_ib->lock);
}

/*
    Called for CBP_I_AM_MASTER_REP when IB in Wait_for_Master state. Go to Slave state and set
    timer to control Master availability
*/
static int ph_im_rep_process(struct sk_buff* skb, struct indication_block* _ib) {

    u16 block_type = ntohs(cbp_hdr(skb)->block_type);

    del_timer(&_ib->timer);

    _ib->state = IB_SLAVE;

    ether_addr_copy(_ib->master_addr, eth_hdr(skb)->h_source);
    _ib->master_type = block_type;

    _ib->attempts = 0;

    _ib->timer.function = no_request_from_master_timer_fn;
    _ib->timer.expires = jiffies + TM_NO_REQ_FROM_MASTER;
    add_timer(&_ib->timer);

    PRINTD(DEBUG_DBG, KERN_INFO DEV_LABEL": New master is set (MAC=%pM TYPE=%s). New state: [%s]\n",
        _ib->master_addr, block_type_to_string[block_type], state_to_string[_ib->state]);

    consume_skb(skb);
    return NET_RX_SUCCESS;
}

/*
    Called for CBP_SLAVE_NEEDED_REQ when IB in Wait_for_Master state
*/
static int ph_sn_req_process_wm(struct sk_buff* skb, struct indication_block* _ib) {

    /* No matter if master is temp or not, we go to slave state anyway */
    int ret = ph_im_rep_process(skb, _ib);

    /* Send response with confirmation  */
    cbp_send_dst(CBP_I_AM_SLAVE_REP, CBP_DT_SLAVE, _ib->dev, eth_hdr(skb)->h_source, NULL);

    return ret;
}

/*
    Called for CBP_SLAVE_NEEDED_REQ when IB in Slave state
*/
static int ph_sn_req_process_s(struct sk_buff* skb, struct indication_block* _ib) {

    /* We ignore packet, if our current master is not temp, or packet from temp master with lower MAC, 
      otherwise set new master with confirmation to it, 
      i.e. if current master is temp and (new master is not temp or has greater MAC). */

    if (_ib->master_type == CBP_DT_MASTER ||
        (ether_addr_greater(_ib->master_addr, eth_hdr(skb)->h_source)
            && ntohs(cbp_hdr(skb)->block_type) == CBP_DT_MASTER_TEMP)) {
    
        consume_skb(skb);
        return NET_RX_SUCCESS;
    }

    return ph_sn_req_process_wm(skb, _ib);
}


/*
    Called for CBP_SLAVE_NEEDED_REQ when IB in Master or Waiting for Slave state
*/
static int ph_sn_req_process_m(struct sk_buff* skb, struct indication_block* _ib) {

    /* We ignore packet, if packet from temp master with lower MAC,
      otherwise go to Slave and set new master with confirmation to it,
      i.e. if new master is not temp, or temp but it has greater MAC. */

    if (ether_addr_greater(skb->dev->dev_addr, eth_hdr(skb)->h_source)
            && ntohs(cbp_hdr(skb)->block_type) == CBP_DT_MASTER_TEMP) {

        consume_skb(skb);
        return NET_RX_SUCCESS;
    }

    return ph_sn_req_process_wm(skb, _ib);
}


/*
    Called for CBP_GET_DATA_REQ when IB in Slave state
*/
static int ph_gd_req_process(struct sk_buff* skb, struct indication_block* _ib) {

    /* Check Master addr, i.e. the packet from our Master */
    if (ether_addr_equal(eth_hdr(skb)->h_source, _ib->master_addr)) {

        /* process get_data request ... */
        struct sensor_data* sns = read_sensors(_ib);

        PRINTD(DEBUG_DBG, KERN_INFO DEV_LABEL": GET DATA request from (MAC=%pM). Current Temperature=%d, Brightness=%d\n",
            eth_hdr(skb)->h_source, sns->temperature, sns->brightness);

        /* reset no_request_from_master timer */
        mod_timer(&_ib->timer, jiffies + TM_NO_REQ_FROM_MASTER);

        /* send get_data response to master */
        cbp_send_payload(CBP_GET_DATA_REP, CBP_DT_SLAVE, sns, _ib->dev, eth_hdr(skb)->h_source, NULL);
    }

    consume_skb(skb);
    return NET_RX_SUCCESS;
}


/*
    Called for CBP_GET_DATA_REP when IB in Master state
*/
static int ph_gd_rep_process(struct sk_buff* skb, struct indication_block* _ib) {
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
    store_sensor_data(_ib, &data);

    /* reflect get_data response for get_data_cycle timer */
    _ib->attempts++;

    PRINTD(DEBUG_DBG, KERN_INFO DEV_LABEL": GET DATA response from (MAC=%pM). Temperature=%d, Brightness=%d. Total responses (%d)\n",
        eth_hdr(skb)->h_source, data.temperature, data.brightness, _ib->attempts);


    consume_skb(skb);
    return NET_RX_SUCCESS;
}


/*
    Called for CBP_SET_DATA when IB in Slave state
*/
static int ph_sd_process(struct sk_buff* skb, struct indication_block* _ib) {

    struct display_data* dsp;

    if (!pskb_may_pull(skb, cbp_hdr_len() + sizeof(struct display_data))) {

        PRINTD(DEBUG_ERR, KERN_ERR DEV_LABEL": SET DATA response from (MAC=%pM) without display_data payload\n",
            eth_hdr(skb)->h_source);

        kfree_skb(skb);
        return NET_RX_DROP;
    }

    /* Check Master addr, i.e. the packet from our Master */
    if (ether_addr_equal(eth_hdr(skb)->h_source, _ib->master_addr)) {

        /* Display data */
        dsp = (struct display_data*)payload_hdr(skb);

        PRINTD(DEBUG_DBG, KERN_INFO DEV_LABEL": SET DATA from (MAC=%pM).\n",
            eth_hdr(skb)->h_source);

        display_data_from_master(dsp);
    }

    consume_skb(skb);
    return NET_RX_SUCCESS;
}

static struct packet_type cb_packet_type __read_mostly = {
    .type = cpu_to_be16(ETH_P_CBP),
    .func = cbp_rcv,
};

static inline void init_handler_with_stub(struct indication_block* _ib) {
    int i, j;
    for (i = 0; i < CBP_NUMBER; i++)
        for (j = 0; j < IB_NUMBER; j++)
            _ib->packet_handlers[i][j] = packet_stub;
}

static inline void clean_sensor_data(struct sensor_data* _s) {
    _s->temperature = 0;
    _s->brightness = 0;
}

static inline void clean_display_data(struct display_data* _d) {
    _d->brightness = 0;
    memset(_d->text, 0, sizeof(_d->text));
    memset(_d->temperature, 0, sizeof(_d->temperature));
    memset(_d->time, 0, sizeof(_d->time));
}

static void init_ib(struct indication_block* _ib, struct net_device* dev) {
    _ib->dev = dev;
    _ib->attempts = 0;
    _ib->state = IB_WAITING_FOR_MASTER;
    _ib->oldest = true;
    _ib->set_data_cycles = 0;
    _ib->t_accum = 0;
    _ib->b_accum = 0;
    _ib->count_accum = 0;
    _ib->set_data_cycles = 0;
    _ib->master_type = CBP_DT_MASTER;

    clean_sensor_data(&_ib->sensors);
    clean_display_data(&_ib->data_for_slaves);

    spin_lock_init(&_ib->lock);
    eth_zero_addr(_ib->master_addr);
    timer_setup(&_ib->timer, master_needed_sent_timer_fn, 0);
    init_handler_with_stub(_ib);

    _ib->packet_handlers[CBP_MASTER_NEEDED_REQ][IB_WAITING_FOR_MASTER] = 
    _ib->packet_handlers[CBP_MASTER_NEEDED_REQ][IB_SLAVE] = ph_mn_req_ignore;
    _ib->packet_handlers[CBP_MASTER_NEEDED_REQ][IB_WAITING_FOR_SLAVE] =
    _ib->packet_handlers[CBP_MASTER_NEEDED_REQ][IB_MASTER] = ph_mn_req_process;

    _ib->packet_handlers[CBP_I_AM_MASTER_REP][IB_WAITING_FOR_MASTER] = ph_im_rep_process;

    _ib->packet_handlers[CBP_SLAVE_NEEDED_REQ][IB_WAITING_FOR_MASTER] = ph_sn_req_process_wm;
    _ib->packet_handlers[CBP_SLAVE_NEEDED_REQ][IB_SLAVE] = ph_sn_req_process_s;
    _ib->packet_handlers[CBP_SLAVE_NEEDED_REQ][IB_WAITING_FOR_SLAVE] = 
    _ib->packet_handlers[CBP_SLAVE_NEEDED_REQ][IB_MASTER] = ph_sn_req_process_m;
   
    _ib->packet_handlers[CBP_I_AM_SLAVE_REP][IB_WAITING_FOR_SLAVE] =
    _ib->packet_handlers[CBP_I_AM_SLAVE_REP][IB_MASTER] = ph_is_rep_process;

    _ib->packet_handlers[CBP_GET_DATA_REQ][IB_SLAVE] = ph_gd_req_process;

    _ib->packet_handlers[CBP_GET_DATA_REP][IB_MASTER] = ph_gd_rep_process;

    _ib->packet_handlers[CBP_SET_DATA][IB_SLAVE] = ph_sd_process;
}


int init_module(void) {
    
    struct net_device* eth_dev = NULL;

    PRINTD(DEBUG_INFO, KERN_INFO DEV_LABEL ": Setup Client Block Module\n");

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

            init_ib(&ib, eth_dev);

            dev_add_pack(&cb_packet_type);

            send_master_needed(&ib);
        }
    
    return 0;
}

void cleanup_module(void) {

    if (ib.dev) {
        del_timer(&ib.timer);

        dev_remove_pack(&cb_packet_type);
        dev_put(ib.dev);
        PRINTD(DEBUG_INFO, KERN_INFO DEV_LABEL ": Release %s\n", eth_device_name);
    }

    PRINTD(DEBUG_INFO, KERN_INFO DEV_LABEL ": Cleanup Client Block Module\n");
    return;
}

MODULE_LICENSE("GPL");
MODULE_AUTHOR("V.Mogilkin <vmog@mail.ru>");
MODULE_DESCRIPTION("StreetLight Indication Module");
MODULE_VERSION("1.00");

module_param(debug, int, 0);
MODULE_PARM_DESC(debug, "Debugging messages level");

module_param(eth_device_name, charp, 0);
MODULE_PARM_DESC(debug, "Ethernet device name");
