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
        int16_t audio[DONGLE_AUDIO_LENGTH];
    } packet_data;
};

int send_ack_packet(struct dongle_packet_handlers* handlers);
int send_audio_packet(struct dongle_packet_handlers* handlers, int16_t* audio);
int read_packet(struct dongle_packet_handlers* handlers, struct dongle_packet* packet);

#ifdef __cplusplus
}
#endif

#endif // DONGLE_PROTOCOL_H
