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
    struct dongle_packet_handlers* retVal = malloc(sizeof(struct dongle_packet_handlers*));
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

    // Set required port options
    tty.c_cflag &= ~PARENB; // No parity
    tty.c_cflag &= ~CSTOPB; // 1 stop bit
    tty.c_cflag |= CS8; // 8 bits/byte
    tty.c_cflag &= ~CRTSCTS; // no flow control
    tty.c_cflag |= CREAD | CLOCAL; // Ignore control lines
    tty.c_lflag &= ~ICANON; // Disable canonical mode (read w/o waiting for newline)

    // Turn off any options that might interfere with our ability to send and
    // receive raw binary bytes.
    tty.c_iflag &= ~(INLCR | IGNCR | ICRNL | IXON | IXOFF);
    tty.c_oflag &= ~(ONLCR | OCRNL);
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_lflag &= ~ECHO; // Disable echo
    tty.c_lflag &= ~ECHOE; // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes
    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
    tty.c_cc[VTIME] = 0; // no wait
    tty.c_cc[VMIN] = 1; 
    cfsetispeed(&tty, 921600);
    cfsetospeed(&tty, 921600);

    if (tcsetattr(portSocket, TCSANOW, &tty) != 0)
    {
        close(portSocket);
        free(retVal);
        return NULL;
    }
    
    retVal->state = serialPort;
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