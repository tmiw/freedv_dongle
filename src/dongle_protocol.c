#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <assert.h>

#include "dongle_protocol.h"

static void pack_audio_packet(char** pBuf, struct dongle_packet* packet)
{
    memcpy(*pBuf, &packet->packet_data.audio_data.audio, packet->length);
    *pBuf += packet->length;
}

static void pack_version_resp_packet(char** pBuf, struct dongle_packet* packet)
{
    // TBD -- version response
}

static void pack_mode_switch_packet(char** pBuf, struct dongle_packet* packet)
{
    memcpy(*pBuf, &packet->packet_data.fdv_mode_data.mode, sizeof(packet->packet_data.fdv_mode_data.mode));
    *pBuf += sizeof(packet->packet_data.fdv_mode_data.mode);
}

static void unpack_audio_packet(char* pBuf, struct dongle_packet* packet)
{
    memcpy(&packet->packet_data.audio_data.audio, pBuf, packet->length);
    pBuf += packet->length;
}

static void unpack_mode_switch_packet(char* pBuf, struct dongle_packet* packet)
{
    memcpy(&packet->packet_data.fdv_mode_data.mode, pBuf, sizeof(packet->packet_data.fdv_mode_data.mode));
    pBuf += sizeof(packet->packet_data.fdv_mode_data.mode);
}

typedef void (*pack_fn_t)(char**, struct dongle_packet*);
typedef void (*unpack_fn_t)(char*, struct dongle_packet*);
static pack_fn_t packet_pack_fn[] = {
    pack_audio_packet,
    NULL,
    NULL,
    NULL,
    NULL,
    pack_version_resp_packet,
    pack_mode_switch_packet,
    NULL
};

static unpack_fn_t packet_unpack_fn[] = {
    unpack_audio_packet,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    unpack_mode_switch_packet,
    NULL
};

static int send_packet_common(struct dongle_packet_handlers* handlers, struct dongle_packet* packet)
{
    size_t alloc_size =
        sizeof(packet->magic_number) + 
        sizeof(packet->version) +
        sizeof(packet->type) +
        sizeof(packet->length) +
        packet->length;
    
    char* tmpbuf = calloc(alloc_size, 1);
    if (tmpbuf == NULL) return 0;

    packet->magic_number = DONGLE_MAGIC_NUMBER;
    packet->version = 1;
    
    // Pack message header into block of data for transmit.
    char* pBuf = &tmpbuf[0];
    memcpy(pBuf, &packet->magic_number, sizeof(packet->magic_number));
    pBuf += sizeof(packet->magic_number);
    memcpy(pBuf, &packet->version, sizeof(packet->version));
    pBuf += sizeof(packet->version);
    memcpy(pBuf, &packet->type, sizeof(packet->type));
    pBuf += sizeof(packet->type);
    memcpy(pBuf, &packet->length, sizeof(packet->length));
    pBuf += sizeof(packet->length);
    
    // Pack message body, if there is one.
    if (packet_pack_fn[packet->type])
    {
        (*packet_pack_fn[packet->type])(&pBuf, packet);
    }
    
    // Write data over serial port.
    int bytes_to_write = alloc_size;
    pBuf = &tmpbuf[0];
    while (bytes_to_write > 0)
    {
        int nwritten = (*handlers->write_fn)(handlers, pBuf, bytes_to_write);
        if (nwritten <= 0)
        {
             free(tmpbuf);
             return 0;
        }
        bytes_to_write -= nwritten;
        pBuf += nwritten;
    }

    (*handlers->flush_packet_fn)(handlers);

    free(tmpbuf);
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
    memcpy(&packet.packet_data.audio_data.audio, audio, DONGLE_AUDIO_LENGTH * sizeof(int16_t));
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
    
    // Read rest of packet header.
    int bytes_to_read = 
        sizeof(packet->version) +
        sizeof(packet->type) +
        sizeof(packet->length);
    char* pBuf = calloc(1, bytes_to_read);
    if (pBuf == NULL) return 0;
   
    char* tmp = pBuf; 
    while(bytes_to_read > 0)
    {
        int nbytes = (*handlers->read_fn)(handlers, tmp, bytes_to_read);
        if (nbytes <= 0) 
        {
            free(pBuf);
            return 0;
        }
        bytes_to_read -= nbytes;
        tmp += nbytes;
    }
    
    // Copy length to result packet and use that to read in the rest.
    tmp = pBuf;
    memcpy(&packet->version, tmp, sizeof(packet->version));
    tmp += sizeof(packet->version);
    memcpy(&packet->type, tmp, sizeof(packet->type));
    tmp += sizeof(packet->type);
    memcpy(&packet->length, tmp, sizeof(packet->length));
    free(pBuf);

    bytes_to_read = packet->length;
    pBuf = calloc(1, bytes_to_read);
    if (pBuf == NULL) return 0;

    tmp = pBuf;
    while(bytes_to_read > 0)
    {
        int nbytes = (*handlers->read_fn)(handlers, tmp, bytes_to_read);
        if (nbytes <= 0) 
        {
            free(pBuf);
            return 0;
        }
        bytes_to_read -= nbytes;
        tmp += nbytes;
    }

    // Unpack packet body, if needed.
    if (packet_unpack_fn[packet->type])
    {
        (*packet_unpack_fn[packet->type])(pBuf, packet);
    }
    
    free(pBuf);
    return 1;
}

#if defined(__linux__) || defined(__APPLE__)
#include <fcntl.h>
#include <termios.h> 
#include <unistd.h>
#include <sys/select.h>
#include <assert.h>

static int usb_read_data(struct dongle_packet_handlers* hndl, void* ptr, int size)
{
    int sock = (int)hndl->state;
    return read(sock, ptr, size);
}

static int usb_write_data(struct dongle_packet_handlers* hndl, void* ptr, int size)
{
    int sock = (int)hndl->state;
    return write(sock, ptr, size);
}

static void usb_flush_data(struct dongle_packet_handlers* hndl)
{
    // empty
}

struct dongle_packet_handlers* dongle_open_port(char* serialPort)
{
    struct dongle_packet_handlers* retVal = malloc(sizeof(struct dongle_packet_handlers));
    if (retVal == NULL)
    {
        return NULL;
    }
    
    retVal->read_fn = usb_read_data;
    retVal->write_fn = usb_write_data;
    retVal->flush_packet_fn = usb_flush_data;
    
    int portSocket = open(serialPort, O_RDWR | O_NOCTTY);
    if (portSocket < 0)
    {
        free(retVal);
        return NULL;
    }
    
    struct termios tty;
    if(tcgetattr(portSocket, &tty) != 0) 
    {
        close(portSocket);
        free(retVal);
        return NULL;
    }

    // Set required options to make it a raw port.
    tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity
    tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication
    tty.c_cflag |= CS8; // 8 bits per byte
    tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)
    tty.c_lflag &= ~ICANON; // Disable canonical mode
    tty.c_lflag &= ~ECHO; // Disable echo
    tty.c_lflag &= ~ECHOE; // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes
    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
    
    tty.c_cc[VTIME] = 0; // wait as long as needed to receive at least 1 byte
    tty.c_cc[VMIN] = 1; 
    cfsetispeed(&tty, 921600);
    cfsetospeed(&tty, 921600);

    if (tcsetattr(portSocket, TCSANOW, &tty) != 0)
    {
        close(portSocket);
        free(retVal);
        return NULL;
    }
    
    retVal->state = portSocket;
    return retVal;
}

void dongle_close_port(struct dongle_packet_handlers* handle)
{
    assert(handle != NULL);
    
    int sock = (int)handle->state;
    close(sock);
    
    free(handle);
}

int dongle_has_data_available(struct dongle_packet_handlers* hndl, int timeout_sec, int timeout_usec)
{
    struct timeval tv;
    int sock = (int)hndl->state;
    fd_set f;

    FD_ZERO(&f);
    FD_SET(sock, &f);

    tv.tv_sec = timeout_sec;
    tv.tv_usec = timeout_usec;
    return select(sock + 1, &f, NULL, NULL, &tv) > 0;
}
#endif // defined(__linux__) || defined(__APPLE__)
