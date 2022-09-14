#ifndef _CBP_BASE_H
#define _CBP_BASE_H

#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>

/* Module label */
#define DEV_LABEL "CBP"

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
static inline void pack_sensor_data(struct sensor_data* pkt, const struct sensor_data* src) {
    pkt->temperature = htons(src->temperature);
    pkt->brightness = htons(src->brightness);
}

/* Unpack sensor data from packet */
static inline void unpack_sensor_data(struct sensor_data* dst, const struct sensor_data* pkt) {
    dst->temperature = ntohs(pkt->temperature);
    dst->brightness = ntohs(pkt->brightness);
}

static inline void clean_sensor_data(struct sensor_data* _s) {
    _s->temperature = 0;
    _s->brightness = 0;
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
static inline void pack_display_data(struct display_data* pkt, const struct display_data* src) {
    pkt->brightness = htons(src->brightness);
    memcpy(pkt->text, src->text, sizeof(pkt->text));
    memcpy(pkt->temperature, src->temperature, sizeof(pkt->temperature));
    memcpy(pkt->time, src->time, sizeof(pkt->time));
}

/* Unpack display data from packet */
static inline void unpack_display_data(struct display_data* dst, const struct display_data* pkt) {
    dst->brightness = ntohs(pkt->brightness);
    memcpy(dst->text, pkt->text, sizeof(pkt->text));
    memcpy(dst->temperature, pkt->temperature, sizeof(pkt->temperature));
    memcpy(dst->time, pkt->time, sizeof(pkt->time));
}

static inline void clean_display_data(struct display_data* _d) {
    _d->brightness = 0;
    memset(_d->text, 0, sizeof(_d->text));
    memset(_d->temperature, 0, sizeof(_d->temperature));
    memset(_d->time, 0, sizeof(_d->time));
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

typedef enum {
    FOREACH_BLOCK_TYPE(GENERATE_ENUM)
} cbp_block_type;

extern const char* packet_type_to_string[];
extern const char* block_type_to_string[];

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
        pack_sensor_data((struct sensor_data*)pkt, (struct sensor_data*)src);
        break;
    case CBP_SET_DATA:
        pack_display_data((struct display_data*)pkt, (struct display_data*)src);
        break;
    default:
        break;
    }
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

extern int debug;
extern int (*cbp_packet_handler)(u16 cbp_op, struct sk_buff* skb);

int cbp_rcv(struct sk_buff* skb, struct net_device* dev,
    struct packet_type* pt, struct net_device* orig_dev);

struct sk_buff* cbp_create(int op_command, int block_type, void* payload, 
    struct net_device* dev, const unsigned char* dest_hw, const unsigned char* src_hw);

void cbp_send_payload(int op, int bt, void* payload, struct net_device* dev,
    const unsigned char* dest_hw, const unsigned char* src_hw);

/*
    Common send variant for most CBP packets without payload to exlude silly mistakes
*/
static inline void cbp_send_dst(int op, int bt, struct net_device* dev,
    const unsigned char* dest_hw, const unsigned char* src_hw) {
    cbp_send_payload(op, bt, NULL, dev, dest_hw, src_hw);
}
#endif