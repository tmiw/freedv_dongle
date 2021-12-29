#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h> 
#include <assert.h>

#include "dongle_protocol.h"

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
    
    struct dongle_packet_handlers* serialPort = dongle_open_port(serial_port);
    if (serialPort == NULL)
    {
        fprintf(stderr, "Could not open serial port (errno %d)\n", errno);
        exit(-1);
    }

    // Open input and output files and send/receive from the Teensy
    int inputFile = open(input_file, O_RDONLY);
    if (inputFile < 0)
    {
        fprintf(stderr, "Could not open input file (errno %d)\n", errno);
        dongle_close_port(serialPort);
        exit(-1);
    }

    int outputFile = open(output_file, O_WRONLY | O_CREAT | O_APPEND | O_TRUNC, 0644);
    if (outputFile < 0)
    {
        fprintf(stderr, "Could not open output file (errno %d)\n", errno);
        close(inputFile);
        dongle_close_port(serialPort);
        exit(-1);
    }

    // Place dongle in correct mode first.
    send_set_fdv_mode_packet(serialPort, mode);

    // Wait for ack
    struct dongle_packet packet;
    while (read_packet(serialPort, &packet) <= 0) { }
    assert(packet.type == DONGLE_PACKET_ACK);
    
    int bufSize = 128;
    short bufIn[bufSize];

    while(read(inputFile, bufIn, bufSize*sizeof(short)) > 0)
    {
        send_audio_packet(serialPort, bufIn, in_tx);

        // Get anything else on the line before we send another packet
        while (dongle_has_data_available(serialPort, 0, 0))
        {
            if (read_packet(serialPort, &packet) <= 0) break;
            write(outputFile, &packet.packet_data.audio_data.audio, packet.length);
        }
    }

    // Wait for any remaining data on the port.
    while (dongle_has_data_available(serialPort, 1, 0))
    {
        if (read_packet(serialPort, &packet) <= 0) break;
        write(outputFile, &packet.packet_data.audio_data.audio, packet.length);
    }

    dongle_close_port(serialPort);
    close(inputFile);
    close(outputFile);

    return 0;
}
