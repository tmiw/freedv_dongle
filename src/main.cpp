#include <Arduino.h>
#include <assert.h>

#include "dongle_protocol.h"

#include "ringbuf.h"
#include "freedv_api.h"

const int ledPin = 13;

static struct freedv* fdv = nullptr;
static ringbuf_t audio_input_buf = nullptr;
static ringbuf_t audio_output_buf = nullptr;
static bool in_transmit = false;
static int read_ctr = 1;

static int usb_read_data(struct dongle_packet_handlers* hndl, void* ptr, int size)
{
    int numRead = Serial.readBytes((char*)ptr, size);
    for (int i = 0; i < numRead; i++, read_ctr++)
    {
        SerialUSB1.printf("%x ", ((char*)ptr)[i]);
        if (!(read_ctr & 0xF)) SerialUSB1.printf("\n");
    }
    return numRead;
}

static int usb_write_data(struct dongle_packet_handlers* hdnl, void* ptr, int size)
{
    return Serial.write((char*)ptr, size);
}

static void usb_flush_data(struct dongle_packet_handlers* hndl)
{
    // empty
}

struct dongle_packet_handlers arduino_dongle_packet_handlers = {
    .read_fn = usb_read_data,
    .write_fn = usb_write_data,
    .flush_packet_fn = usb_flush_data,
};

static void open_freedv_handle(int mode)
{
    if (fdv != nullptr)
    {
        freedv_close(fdv);
        ringbuf_free(&audio_input_buf);
        ringbuf_free(&audio_output_buf);
    }
    
    fdv = freedv_open(mode);
    assert(fdv != nullptr);
    
    audio_input_buf = ringbuf_new(freedv_get_n_speech_samples(fdv) * sizeof(short) * 5);
    assert(audio_input_buf != nullptr);
    audio_output_buf = ringbuf_new(freedv_get_n_tx_modem_samples(fdv) * sizeof(short) * 5);
    assert(audio_output_buf != nullptr);
}

static void handle_incoming_messages()
{
    struct dongle_packet packet;
    bool send_ack = true;
    
    if (read_packet(&arduino_dongle_packet_handlers, &packet) > 0)
    {        
        // Handle request depending on the packet type.
        switch(packet.type)
        {
            case DONGLE_PACKET_AUDIO:
            {
                // Inbound audio to be processed. Append to ring buffer for later handling.
                ringbuf_memcpy_into(audio_input_buf, packet.packet_data.audio_data.audio, packet.length);
                send_ack = false;
                break;
            }
            case DONGLE_PACKET_SWITCH_TX_MODE:
            case DONGLE_PACKET_SWITCH_RX_MODE:
            {
                // Switch in or out of TX mode. Queued audio will also be cleared so we can
                // immediately start processing inbound data in the new mode.
                in_transmit = packet.type == DONGLE_PACKET_SWITCH_TX_MODE;
                ringbuf_reset(audio_input_buf);
                break;
            }
            case DONGLE_PACKET_SET_FDV_MODE:
            {
                // Reopen FDV handle using new mode.
                open_freedv_handle(packet.packet_data.fdv_mode_data.mode);
                ringbuf_reset(audio_input_buf);
                break;
            }
            default:
            {
                // Ignore anything we don't recognize.
                send_ack = false;
                break;
            }
        }
        
        if (send_ack) send_ack_packet(&arduino_dongle_packet_handlers);
    }
}

static void process_queued_audio()
{
    static short input_buf[2048];
    static short output_buf[2048];
    size_t input_bytes = 0;
    
    if (in_transmit)
    {
        input_bytes = freedv_get_n_speech_samples(fdv) * sizeof(short);
        
        // Initialize LED in off state before processing transmit audio.
        //digitalWrite(ledPin, LOW);
    }
    else
    {
        input_bytes = freedv_nin(fdv) * sizeof(short);
    }
    
    while(ringbuf_bytes_used(audio_input_buf) >= input_bytes)
    {
        ringbuf_memcpy_from(input_buf, audio_input_buf, input_bytes);
        if (in_transmit)
        {
            // LED in TX means that we're processing a packet of audio.
            //digitalWrite(ledPin, HIGH);
            freedv_tx(fdv, output_buf, input_buf);
            ringbuf_memcpy_into(audio_output_buf, output_buf, sizeof(short) * freedv_get_n_tx_modem_samples(fdv));
            //digitalWrite(ledPin, LOW);
        }
        else
        {
            int nout = freedv_rx(fdv, output_buf, input_buf);
            ringbuf_memcpy_into(audio_output_buf, output_buf, sizeof(short) * nout);
            input_bytes = freedv_nin(fdv) * sizeof(short);
            
            // Update LED to reflect current sync state.
            //digitalWrite(ledPin, freedv_get_sync(fdv) ? HIGH : LOW);
        }        
    }
}

static void transmit_output_audio()
{
    while (ringbuf_bytes_used(audio_output_buf) >= 256)
    {
        short buf[128];
        ringbuf_memcpy_from(buf, audio_output_buf, 256);
        send_audio_packet(&arduino_dongle_packet_handlers, buf);
    }
}

void serialEvent()
{
    handle_incoming_messages();
}

void serialEventUSB1()
{
    int x = SerialUSB1.read();
    if (x == '\r' || x == '\n')
    {
        SerialUSB1.println("still alive\r\n");
    }
}

void setup()
{ 
    pinMode(ledPin, OUTPUT);    
    open_freedv_handle(FREEDV_MODE_700D);
    
    SerialUSB1.printf("freedv_dongle debug console\n");
}

void loop() 
{
    // Each action within the loop is a separate function for readability.
    process_queued_audio();
    transmit_output_audio();
}

