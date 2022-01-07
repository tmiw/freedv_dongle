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
    fprintf(stderr, "Usage: %s [-h] [-m mode] -p [serial port] -i [input file]\n", appname);
    fprintf(stderr, "    -h: This help message\n");
    fprintf(stderr, "    -m: The FreeDV mode to use (700D, 700E or 1600). Defaults to 700D.\n");
    fprintf(stderr, "    -p: The serial port to use to communicate with the dongle (required).\n");
    fprintf(stderr, "    -i: The input file to process (required; 8KHz int16 raw samples).\n");
}

int main(int argc, char** argv)
{
    int opt;
    int mode = DONGLE_MODE_700D;
    char* input_file = NULL;
    char* serial_port = NULL;
    void* encoded_audio = NULL;
    size_t encoded_audio_size = 0;
    short random_data[8000*5]; // 5 seconds random audio
    
    while ((opt = getopt(argc, argv, "hm:i:p:")) != -1) 
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
            case 'i':
                input_file = malloc(strlen(optarg) + 1);
                assert(input_file != NULL);
                strcpy(input_file, optarg);
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
    
    if (serial_port == NULL || input_file == NULL)
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

    // Open input file and send/receive from the Teensy
    int inputFile = open(input_file, O_RDONLY);
    if (inputFile < 0)
    {
        fprintf(stderr, "Could not open input file (errno %d)\n", errno);
        dongle_close_port(serialPort);
        exit(-1);
    }

    // Generate random data to force Teensy out of sync
    srand(time(NULL));
    for (size_t i = 0; i < sizeof(random_data) / sizeof(short); i++)
    {
        random_data[i] = ((double)rand())/((double)RAND_MAX) * 65535 - 32767;
    }
    
    // Place dongle in correct mode first.
    send_set_fdv_mode_packet(serialPort, mode);

    // Wait for ack
    struct dongle_packet packet;
    while (read_packet(serialPort, &packet) <= 0) { }
    assert(packet.type == DONGLE_PACKET_ACK);
    
    int expected_audio_size = 0;
    while(1)
    {
        int bufSize = 128;
        int sleep_time = 5000; //(1000.0/8000.0)*128*1000;
        short bufIn[bufSize];

        printf("Transmitting...\n");
        while(read(inputFile, bufIn, bufSize*sizeof(short)) > 0)
        {
            send_audio_packet(serialPort, bufIn, 1);

            // Get anything else on the line before we send another packet
            while (dongle_has_data_available(serialPort, 0, 0))
            {
                if (read_packet(serialPort, &packet) <= 0) break;
                encoded_audio = realloc(encoded_audio, encoded_audio_size + packet.length);
                assert(encoded_audio);
                memcpy(encoded_audio + encoded_audio_size, &packet.packet_data.audio_data.audio, packet.length);
                encoded_audio_size += packet.length;
            }
        
            // Pause between packets
            usleep(sleep_time);
        }

        // Wait for any remaining data on the port.
        while (dongle_has_data_available(serialPort, 0, 0))
        {
            if (read_packet(serialPort, &packet) <= 0) break;
        
            encoded_audio = realloc(encoded_audio, encoded_audio_size + sizeof(short)*packet.length);
            assert(encoded_audio);
            memcpy(encoded_audio + encoded_audio_size, &packet.packet_data.audio_data.audio, sizeof(short)*packet.length);
            encoded_audio_size += sizeof(short)*packet.length;
        
            // Pause between packets
            usleep(sleep_time);
        }
        
        if (expected_audio_size != encoded_audio_size && expected_audio_size != 0)
        {
            fprintf(stderr, "XXX we stopped receiving TX audio at some point!\n");
            break;
        }
        expected_audio_size = encoded_audio_size;
        
        printf("Transmitted %d bytes\n", encoded_audio_size);
        
        printf("Receiving...\n");
        
        // Transmit the encoded audio back
        int received_audio_size = 0;
        void* ptr = encoded_audio;
        for (size_t i = 0; i < encoded_audio_size; i += sizeof(short)*128)
        {
            send_audio_packet(serialPort, ptr, 0);

            // Get anything else on the line before we send another packet
            while (dongle_has_data_available(serialPort, 0, 0))
            {
                if (read_packet(serialPort, &packet) <= 0) break;
                received_audio_size += packet.length;
            }
        
            // Pause between packets
            usleep(sleep_time);
            
            ptr += sizeof(short)*128;
        }
        
        // Force Teensy out of sync
        ptr = random_data;
        for (size_t i = 0; i < sizeof(random_data); i += sizeof(short)*128)
        {
            send_audio_packet(serialPort, ptr, 0);

            // Get anything else on the line before we send another packet
            while (dongle_has_data_available(serialPort, 0, 0))
            {
                if (read_packet(serialPort, &packet) <= 0) break;
                received_audio_size += packet.length;
            }
        
            // Pause between packets
            usleep(sleep_time);
            
            ptr += sizeof(short)*128;
        }
        
        printf("Received %d bytes\n", received_audio_size);
        
        free(encoded_audio);
        encoded_audio = NULL;
        encoded_audio_size = 0;
        
        lseek(inputFile, 0, SEEK_SET);
    }

    dongle_close_port(serialPort);
    close(inputFile);

    return 0;
}
