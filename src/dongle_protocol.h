#ifndef DONGLE_PROTOCOL_H
#define DONGLE_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif
    
#include <stdint.h>

#define DONGLE_MAGIC_NUMBER 0x46445644    // FDVD
#define DONGLE_AUDIO_LENGTH 128

#define DONGLE_PACKET_AUDIO 1
#define DONGLE_PACKET_ACK 2
#define DONGLE_PACKET_SWITCH_TX_MODE 3
#define DONGLE_PACKET_SWITCH_RX_MODE 4
#define DONGLE_PACKET_REQ_VERSION 5
#define DONGLE_PACKET_RES_VERSION 6
#define DONGLE_PACKET_SET_FDV_MODE 7
#define DONGLE_PACKET_NACK 8

// Matches freedv_api.h.
#define DONGLE_MODE_700D 7
#define DONGLE_MODE_700E 13
#define DONGLE_MODE_1600 0

struct dongle_packet_handlers
{
    int (*read_fn)(struct dongle_packet_handlers*, void*, int);
    int (*write_fn)(struct dongle_packet_handlers*, void*, int);
    void (*flush_packet_fn)(struct dongle_packet_handlers*);
    void* state;
};

struct dongle_packet
{
    uint32_t magic_number;
    uint8_t version;
    uint8_t type;
    uint16_t length;
    union
    {
        struct {
            int16_t audio[DONGLE_AUDIO_LENGTH];
        } audio_data;
        struct {
            uint32_t major;
            uint32_t minor;
            uint32_t patch;
        } sw_version_data;
        struct {
            uint32_t mode;
        } fdv_mode_data;
    } packet_data;
};

int send_switch_tx_mode_packet(struct dongle_packet_handlers* handlers);
int send_switch_rx_mode_packet(struct dongle_packet_handlers* handlers);
int send_set_fdv_mode_packet(struct dongle_packet_handlers* handlers, int mode);
int send_ack_packet(struct dongle_packet_handlers* handlers);
int send_audio_packet(struct dongle_packet_handlers* handlers, int16_t* audio);
int read_packet(struct dongle_packet_handlers* handlers, struct dongle_packet* packet);

#ifdef __cplusplus
}
#endif

#endif // DONGLE_PROTOCOL_H
