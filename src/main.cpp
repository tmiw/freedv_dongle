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
static short* input_buf;
static short* output_buf;
static int last_audio_mode_rx = -1;

//static int read_ctr = 1;

static int usb_read_data(struct dongle_packet_handlers* hndl, void* ptr, int size)
{
    //SerialUSB1.printf("usb_read_data(%x, %x, %d)\r\n", hndl, ptr, size);
    int numRead = Serial.readBytes((char*)ptr, size);
    return numRead;
}

static int usb_write_data(struct dongle_packet_handlers* hdnl, void* ptr, int size)
{
    /*for (int i = 0; i < size; i++, read_ctr++)
    {
        SerialUSB1.printf("%x ", ((char*)ptr)[i]);
        if (!(read_ctr & 0xF)) SerialUSB1.printf("\n");
    }*/
    return Serial.write((char*)ptr, size);
}

static void usb_flush_data(struct dongle_packet_handlers* hndl)
{
    Serial.send_now();
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
        free(input_buf);
        free(output_buf);
    }
    
    fdv = freedv_open(mode);
    assert(fdv != nullptr);
    
    // 700D/E clipping/BPF
    if (mode == FREEDV_MODE_700D || mode == FREEDV_MODE_700E)
    {
        freedv_set_tx_bpf(fdv, 1);
        freedv_set_clip(fdv, 1);
    }
    
    // Default squelch settings
    float squelchDb = 0.0f;
    switch(mode)
    {
        case FREEDV_MODE_700D:
            squelchDb = -2.0f;
            break;
        case FREEDV_MODE_700E:
            squelchDb = 1.0f;
            break;
        case FREEDV_MODE_1600:
            squelchDb = 4.0f;
            break;
    }
    freedv_set_squelch_en(fdv, 1);
    freedv_set_snr_squelch_thresh(fdv, squelchDb);
    
    size_t max_speech_samples = freedv_get_n_max_speech_samples(fdv);
    size_t max_modem_samples = freedv_get_n_max_modem_samples(fdv);
    size_t max_sample_size = max_speech_samples > max_modem_samples ? max_speech_samples : max_modem_samples;

    audio_input_buf = ringbuf_new(max_sample_size * sizeof(short) * 10);
    assert(audio_input_buf != nullptr);
    audio_output_buf = ringbuf_new(max_sample_size * sizeof(short) * 10);
    assert(audio_output_buf != nullptr);
        
    input_buf = (short*)malloc(max_sample_size * sizeof(short));
    assert(input_buf != nullptr);
    output_buf = (short*)malloc(max_sample_size * sizeof(short));
    assert(output_buf != nullptr);
    
    last_audio_mode_rx = -1;
}

static void handle_incoming_messages()
{
    struct dongle_packet packet;
    bool send_ack = true;
    
    if (Serial.available() && read_packet(&arduino_dongle_packet_handlers, &packet) > 0)
    {
        //SerialUSB1.printf("got packet %d\r\n", packet.type);
        
        // Handle request depending on the packet type.
        switch(packet.type)
        {
            case DONGLE_PACKET_RX_AUDIO:
            case DONGLE_PACKET_TX_AUDIO:
            {
                // Inbound audio to be processed. Append to ring buffer for later handling.
                if (last_audio_mode_rx != packet.type)
                {
                    ringbuf_reset(audio_input_buf);
                    ringbuf_reset(audio_output_buf);
                    last_audio_mode_rx = packet.type;
                }
                in_transmit = packet.type == DONGLE_PACKET_TX_AUDIO;
                ringbuf_memcpy_into(audio_input_buf, packet.packet_data.audio_data.audio, packet.length);
                send_ack = false;
                break;
            }
            case DONGLE_PACKET_SET_FDV_MODE:
            {
                // Reopen FDV handle using new mode.
                open_freedv_handle(packet.packet_data.fdv_mode_data.mode);
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
        
        //SerialUSB1.printf("finished processing packet %d\r\n", packet.type);
    }
}

static void process_queued_audio()
{
    size_t input_bytes = 0;
    
    if (in_transmit)
    {
        input_bytes = freedv_get_n_speech_samples(fdv) * sizeof(short);
        
        // Initialize LED in off state before processing transmit audio.
        digitalWrite(ledPin, LOW);
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
            digitalWrite(ledPin, HIGH);
            freedv_tx(fdv, output_buf, input_buf);
            ringbuf_memcpy_into(audio_output_buf, output_buf, sizeof(short) * freedv_get_n_tx_modem_samples(fdv));
            digitalWrite(ledPin, LOW);
        }
        else
        {
            int nout = freedv_rx(fdv, output_buf, input_buf);
            ringbuf_memcpy_into(audio_output_buf, output_buf, sizeof(short) * nout);
            input_bytes = freedv_nin(fdv) * sizeof(short);
            
            // Update LED to reflect current sync state.
            digitalWrite(ledPin, freedv_get_sync(fdv) ? HIGH : LOW);
        }        
    }
}

static void transmit_output_audio()
{
    short buf[DONGLE_AUDIO_LENGTH];
    while (ringbuf_bytes_used(audio_output_buf) >= sizeof(buf))
    {
        ringbuf_memcpy_from(buf, audio_output_buf, sizeof(buf));
        send_audio_packet(&arduino_dongle_packet_handlers, buf, in_transmit);
    }
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
    
    // Let user know we've fully started up (3 fast blinks).
    digitalWrite(ledPin, HIGH);
    delay(50);
    digitalWrite(ledPin, LOW);
    delay(150);
    digitalWrite(ledPin, HIGH);
    delay(50);
    digitalWrite(ledPin, LOW);
    delay(150);
    digitalWrite(ledPin, HIGH);
    delay(50);
    digitalWrite(ledPin, LOW);
}

void loop() 
{
    // Each action within the loop is a separate function for readability.
    handle_incoming_messages();
    process_queued_audio();
    transmit_output_audio();
}

