# include "cbp_base.h"

#define DEV_LABEL "CBP"


struct sk_buff* cbp_create(int op_command, int block_type,
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
        pkt_payload = (void*)(cbp + 1);

        /* we have payload */
        pack_cbp_payload(pkt_payload, payload, op_command);
    }
    else {
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

/*
    Create and send CBP packet with payload
*/
void cbp_send_payload(int op, int bt, void* payload, struct net_device* dev,
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
