#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/select.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h> 
#include <termios.h> 
#include <unistd.h>
#include <assert.h>

#include "dongle_protocol.h"

static int usb_has_data(struct dongle_packet_handlers* hndl, int timeout_sec, int timeout_usec)
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

static void print_usage(char* appname)
{
    fprintf(stderr, "Usage: %s [-h] [-m mode] -r|-t -p [serial port] -i [input file] -o [output file]\n", appname);
    fprintf(stderr, "    -h: This help message\n");
    fprintf(stderr, "    -m: The FreeDV mode to use (700D, 700E or 1600). Defaults to 700D.\n");
    fprintf(stderr, "    -r: Places the dongle in receive mode (this or -t required).\n");
    fprintf(stderr, "    -t: Places the dongle in transmit mode (this or -r required).\n");
    fprintf(stderr, "    -p: The serial port to use to communicate with the dongle (required).\n");
    fprintf(stderr, "    -i: The input file to process (required; 8KHz int16 raw samples).\n");
    fprintf(stderr, "    -o: The file to write output to (required; 8KHz int16 raw samples).\n");
}

int main(int argc, char** argv)
{
    int opt;
    int in_tx = 0;
    int in_rx = 0;
    int mode = DONGLE_MODE_700D;
    char* input_file = NULL;
    char* output_file = NULL;
    char* serial_port = NULL;
    
    while ((opt = getopt(argc, argv, "hrtm:i:o:p:")) != -1) 
    {
        switch(opt)
        {
            case 'r':
                in_rx = 1;
                break;
            case 't':
                in_tx = 1;
                break;
            case 'm':
                if (!strcmp("700D", optarg))
                {
                    mode = DONGLE_MODE_700D;
                }
                else if (!strcmp("700E", optarg))
                {
                    mode = DONGLE_MODE_700E;
                }
                else if (!strcmp("1600", optarg))
                {
                    mode = DONGLE_MODE_1600;
                }
                else
                {
                    print_usage(argv[0]);
                    exit(-1);
                }
                break;
            case 'i':
                input_file = malloc(strlen(optarg) + 1);
                assert(input_file != NULL);
                strcpy(input_file, optarg);
                break;
            case 'o':
                output_file = malloc(strlen(optarg) + 1);
                assert(output_file != NULL);
                strcpy(output_file, optarg);
                break;
            case 'p':
                serial_port = malloc(strlen(optarg) + 1);
                assert(serial_port != NULL);
                strcpy(serial_port, optarg);
                break;
            case 'h':
            default:
                print_usage(argv[0]);
                exit(-1);
        }
    }
    
    if (serial_port == NULL || output_file == NULL || input_file == NULL || (!in_rx && !in_tx) || (in_rx && in_tx))
    {
        print_usage(argv[0]);
        exit(-1);
    }
    
    int serialPort = open(serial_port, O_RDWR | O_NOCTTY);
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

    if (tcsetattr(serialPort, TCSANOW, &tty) != 0)
    {
        fprintf(stderr, "Could not save serial port attributes (errno %d)\n", errno);
        exit(-1);
    }

    // Open input and output files and send/receive from the Pico
    int inputFile = open(input_file, O_RDONLY);
    if (inputFile < 0)
    {
        fprintf(stderr, "Could not open input file (errno %d)\n", errno);
        exit(-1);
    }

    int outputFile = open(output_file, O_WRONLY | O_CREAT | O_APPEND | O_TRUNC, 0644);
    if (outputFile < 0)
    {
        fprintf(stderr, "Could not open output file (errno %d)\n", errno);
        exit(-1);
    }

    host_dongle_packet_handlers.state = (void*)serialPort;

    // Place the dongle in the appropriate state first.
    if (in_tx)
    {
        send_switch_tx_mode_packet(&host_dongle_packet_handlers);
    }
    else if (in_rx)
    {
        send_switch_rx_mode_packet(&host_dongle_packet_handlers);
    }
    
    struct dongle_packet packet;
    
    // Wait for ack
    while (read_packet(&host_dongle_packet_handlers, &packet) <= 0) { }
    assert(packet.type == DONGLE_PACKET_ACK);
    
    send_set_fdv_mode_packet(&host_dongle_packet_handlers, mode);

    // Wait for ack
    while (read_packet(&host_dongle_packet_handlers, &packet) <= 0) { }
    assert(packet.type == DONGLE_PACKET_ACK);
    
    int bufSize = 128;
    short bufIn[bufSize];

    while(read(inputFile, bufIn, bufSize*sizeof(short)) > 0)
    {
        send_audio_packet(&host_dongle_packet_handlers, bufIn);

        // Get anything else on the line before we send another packet
        while (usb_has_data(&host_dongle_packet_handlers, 0, 0))
        {
            if (read_packet(&host_dongle_packet_handlers, &packet) <= 0) break;
            write(outputFile, &packet.packet_data.audio_data.audio, packet.length);
        }
    }

    // Wait for any remaining data on the port.
    while (usb_has_data(&host_dongle_packet_handlers, 1, 0))
    {
        if (read_packet(&host_dongle_packet_handlers, &packet) <= 0) break;
        write(outputFile, &packet.packet_data.audio_data.audio, packet.length);
    }

    close(serialPort);
    close(inputFile);
    close(outputFile);

    return 0;
}
