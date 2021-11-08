#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <assert.h>

#include "dongle_protocol.h"

static int send_packet_common(struct dongle_packet_handlers* handlers, struct dongle_packet* packet)
{
    packet->magic_number = DONGLE_MAGIC_NUMBER;
    packet->version = 1;
    
    // Write magic number
    if ((*handlers->write_fn)(handlers, &packet->magic_number, sizeof(packet->magic_number)) <= 0) return 0;

    // Write rest of header
    int bytes_to_write = sizeof(packet->version) + sizeof(packet->type) + sizeof(packet->length);
    void* pPacket = &packet->version;
    while (bytes_to_write > 0)
    {
        int nwritten = (*handlers->write_fn)(handlers, pPacket, bytes_to_write);
        if (nwritten <= 0) return 0;
        bytes_to_write -= nwritten;
        pPacket += nwritten;
    }

    // Write data if needed.
    pPacket = &packet->packet_data;
    bytes_to_write = packet->length;
    while (bytes_to_write > 0)
    {
        int nwritten = (*handlers->write_fn)(handlers, pPacket, bytes_to_write);
        if (nwritten <= 0) return 0;
        bytes_to_write -= nwritten;
        pPacket += nwritten;
    }

    (*handlers->flush_packet_fn)(handlers);

    return 1;
}

int send_ack_packet(struct dongle_packet_handlers* handlers)
{
    struct dongle_packet packet;

    packet.type = DONGLE_PACKET_ACK;
    packet.length = 0;

    return send_packet_common(handlers, &packet);
}

int send_audio_packet(struct dongle_packet_handlers* handlers, int16_t* audio)
{
    struct dongle_packet packet;

    packet.type = DONGLE_PACKET_AUDIO;
    memcpy(&packet.packet_data.audio_data, audio, DONGLE_AUDIO_LENGTH * sizeof(int16_t));
    packet.length = DONGLE_AUDIO_LENGTH * sizeof(int16_t);

    return send_packet_common(handlers, &packet);
}

int send_switch_tx_mode_packet(struct dongle_packet_handlers* handlers)
{
    struct dongle_packet packet;

    packet.type = DONGLE_PACKET_SWITCH_TX_MODE;
    packet.length = 0;

    return send_packet_common(handlers, &packet);
}

int send_switch_rx_mode_packet(struct dongle_packet_handlers* handlers)
{
    struct dongle_packet packet;

    packet.type = DONGLE_PACKET_SWITCH_RX_MODE;
    packet.length = 0;

    return send_packet_common(handlers, &packet);
}

int send_set_fdv_mode_packet(struct dongle_packet_handlers* handlers, int mode)
{
    struct dongle_packet packet;

    packet.type = DONGLE_PACKET_SET_FDV_MODE;
    packet.packet_data.fdv_mode_data.mode = mode;
    packet.length = sizeof(packet.packet_data.fdv_mode_data);

    return send_packet_common(handlers, &packet);
}

int read_packet(struct dongle_packet_handlers* handlers, struct dongle_packet* packet)
{
    // Look for magic number.
    packet->magic_number = 0;
    while(packet->magic_number != DONGLE_MAGIC_NUMBER)
    {
        uint8_t* p = ((uint8_t*)&packet->magic_number) + 3;
        packet->magic_number >>= 8;
        if ((*handlers->read_fn)(handlers, p, 1) <= 0) return 0;
        assert(p != 0);
    }
    
    // Read other header info in.
    int full_struct_size = sizeof(struct dongle_packet);
    int data_size = sizeof(packet->packet_data);
    int bytes_to_read = full_struct_size - data_size - sizeof(packet->magic_number);
    uint8_t* ptr = (uint8_t*)&packet->version;
    while(bytes_to_read > 0)
    {
        int nbytes = (*handlers->read_fn)(handlers, ptr, bytes_to_read);
        if (nbytes <= 0) return 0;
        bytes_to_read -= nbytes;
        ptr += nbytes;
    }

    // Read data.
    bytes_to_read = packet->length;
    ptr = (uint8_t*)&packet->packet_data;
    while(bytes_to_read > 0)
    {
        int nbytes = (*handlers->read_fn)(handlers, ptr, bytes_to_read);
        if (nbytes <= 0) return 0;
        bytes_to_read -= nbytes;
        ptr += nbytes;
    }

    return 1;
}