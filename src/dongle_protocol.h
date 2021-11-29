#ifndef DONGLE_PROTOCOL_H
#define DONGLE_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif
    
#include <stdint.h>

#define DONGLE_MAGIC_NUMBER 0x46445644    // FDVD
#define DONGLE_AUDIO_LENGTH 128

#define DONGLE_PACKET_RX_AUDIO 0
#define DONGLE_PACKET_TX_AUDIO 1
#define DONGLE_PACKET_ACK 2
#define DONGLE_PACKET_REQ_VERSION 3
#define DONGLE_PACKET_RES_VERSION 4
#define DONGLE_PACKET_SET_FDV_MODE 5
#define DONGLE_PACKET_NACK 6
    
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

int send_set_fdv_mode_packet(struct dongle_packet_handlers* handlers, int mode);
int send_ack_packet(struct dongle_packet_handlers* handlers);
int send_audio_packet(struct dongle_packet_handlers* handlers, int16_t* audio, int tx);
int read_packet(struct dongle_packet_handlers* handlers, struct dongle_packet* packet);

#if defined(__linux__) || defined(__APPLE__)
struct dongle_packet_handlers* dongle_open_port(char* serialPort);
void dongle_close_port(struct dongle_packet_handlers* handle);
int dongle_has_data_available(struct dongle_packet_handlers* hndl, int timeout_sec, int timeout_usec);
#endif // defined(__linux__) || defined(__APPLE__)

#ifdef __cplusplus
}
#endif

#endif // DONGLE_PROTOCOL_H
