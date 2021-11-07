#include <stdio.h>
#include <stdlib.h>
#include <sys/select.h>

#include <fcntl.h>
#include <errno.h> 
#include <termios.h> 
#include <unistd.h>
#include <assert.h>

#include "dongle_protocol.h"

static int usb_has_data(struct dongle_packet_handlers* hndl)
{
    struct timeval tv;
    int sock = (int)hndl->state;
    fd_set f;

    FD_ZERO(&f);
    FD_SET(sock, &f);

    tv.tv_sec = 0;
    tv.tv_usec = 0; // 8ms (1/2 required for 16K)
    return select(sock + 1, &f, NULL, NULL, &tv) > 0;
}

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

void usb_flush_data(struct dongle_packet_handlers* hndl) { }

struct dongle_packet_handlers host_dongle_packet_handlers = {
    .read_fn = usb_read_data,
    .write_fn = usb_write_data,
    .flush_packet_fn = usb_flush_data,
};

int main(int argc, char** argv)
{
    if (argc != 4)
    {
        fprintf(stderr, "Usage: %s [port] [in] [out]\n", argv[0]);
        exit(-1);
    }

    int serialPort = open(argv[1], O_RDWR | O_NOCTTY);
    if (serialPort < 0)
    {
        fprintf(stderr, "Could not open serial port (errno %d)\n", errno);
        exit(-1);
    }

    struct termios tty;
    if(tcgetattr(serialPort, &tty) != 0) 
    {
        fprintf(stderr, "Could not get serial port attributes (errno %d)s\n", errno);
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
#if 1
    tty.c_lflag &= ~ECHO; // Disable echo
    tty.c_lflag &= ~ECHOE; // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes
    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
#endif
    tty.c_cc[VTIME] = 0; // no wait
    tty.c_cc[VMIN] = 1; 
    cfsetispeed(&tty, 921600);
    cfsetospeed(&tty, 921600);
    //cfsetispeed(&tty, 115200);
    //cfsetospeed(&tty, 115200);

    if (tcsetattr(serialPort, TCSANOW, &tty) != 0)
    {
        fprintf(stderr, "Could not save serial port attributes (errno %d)\n", errno);
        exit(-1);
    }

    // Open input and output files and send/receive from the Pico
    int inputFile = open(argv[2], O_RDONLY);
    if (inputFile < 0)
    {
        fprintf(stderr, "Could not open input file (errno %d)\n", errno);
        exit(-1);
    }

    int outputFile = open(argv[3], O_WRONLY | O_CREAT | O_APPEND | O_TRUNC, 0644);
    if (outputFile < 0)
    {
        fprintf(stderr, "Could not open output file (errno %d)\n", errno);
        exit(-1);
    }

    host_dongle_packet_handlers.state = (void*)serialPort;

    int bufSize = 128;
    short bufIn[bufSize];
    struct dongle_packet packet;

    while(read(inputFile, bufIn, bufSize*sizeof(short)) > 0)
    {
        send_audio_packet(&host_dongle_packet_handlers, bufIn);

        // Wait for at least one packet before we proceed.
again:
        while (read_packet(&host_dongle_packet_handlers, &packet) == 0) { }

        /// If it's not an ack packet, write it out and keep waiting
        if (packet.type != DONGLE_PACKET_ACK)
        {
            write(outputFile, &packet.packet_data.audio, packet.length);
            goto again;
        }
        //fprintf(stderr, "got ack: %d\n", packet.type);

        // Get anything else on the line before we send another packet
        while (usb_has_data(&host_dongle_packet_handlers))
        {
            if (read_packet(&host_dongle_packet_handlers, &packet) <= 0) break;
            fprintf(stderr, "got data: %d\n", packet.type);
            write(outputFile, &packet.packet_data.audio, packet.length);
        }
    }

    // Wait for any remaining data on the port.
    while (usb_has_data(&host_dongle_packet_handlers))
    {
        if (read_packet(&host_dongle_packet_handlers, &packet) <= 0) break;
        write(outputFile, &packet.packet_data.audio, packet.length);
    }

    close(serialPort);
    close(inputFile);
    close(outputFile);

    return 0;
}
