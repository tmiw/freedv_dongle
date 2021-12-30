#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h> 
#include <assert.h>
#include <time.h>

#include "dongle_protocol.h"

static void print_usage(char* appname)
{
    fprintf(stderr, "Usage: %s [-h] [-m mode] -p [serial port]\n", appname);
    fprintf(stderr, "    -h: This help message\n");
    fprintf(stderr, "    -m: The FreeDV mode to use (700D, 700E or 1600). Defaults to 700D.\n");
}

int main(int argc, char** argv)
{
    int opt;
    int mode = DONGLE_MODE_700D;
    char* serial_port = NULL;
    
    while ((opt = getopt(argc, argv, "hm:p:")) != -1) 
    {
        switch(opt)
        {
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
    
    if (serial_port == NULL)
    {
        print_usage(argv[0]);
        exit(-1);
    }
    
    while(1)
    {
        struct dongle_packet_handlers* serialPort = dongle_open_port(serial_port);
        if (serialPort == NULL)
        {
            fprintf(stderr, "Could not open serial port (errno %d)\n", errno);
            exit(-1);
        }
    
        // Place dongle in correct mode.
        send_set_fdv_mode_packet(serialPort, mode);
    
        // Wait for ack
        struct dongle_packet packet;
        while (read_packet(serialPort, &packet) <= 0) { }
        assert(packet.type == DONGLE_PACKET_ACK);
    
        dongle_close_port(serialPort);
        //usleep(1000);
    }

    return 0;
}
