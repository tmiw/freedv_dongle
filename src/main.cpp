#include <Arduino.h>

#include "dongle_protocol.h"

#include "ringbuf.h"
#include "freedv_api.h"

const int ledPin = 13;

static struct freedv* fdv;

static ringbuf_t audio_input_buf;
static ringbuf_t audio_output_buf;

static int usb_read_data(struct dongle_packet_handlers* hndl, void* ptr, int size)
{
    return Serial.readBytes((char*)ptr, size);
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


void setup()
{ 
    pinMode(ledPin, OUTPUT);
    
    fdv = freedv_open(FREEDV_MODE_700D);
    audio_input_buf = ringbuf_new(freedv_get_n_speech_samples(fdv) * sizeof(short) * 5);
    audio_output_buf = ringbuf_new(freedv_get_n_tx_modem_samples(fdv) * sizeof(short) * 5);
}

void loop() 
{
    static short input_buf[2048];
    static short output_buf[2048];
    int input_bytes = freedv_get_n_speech_samples(fdv) * sizeof(short);
    struct dongle_packet packet;

    if (read_packet(&arduino_dongle_packet_handlers, &packet) > 0)
    {
        send_ack_packet(&arduino_dongle_packet_handlers);
        ringbuf_memcpy_into(audio_input_buf, packet.packet_data.audio, packet.length);
    }

    while(ringbuf_bytes_used(audio_input_buf) >= input_bytes)
    {
        digitalWrite(ledPin, HIGH);   // set the LED on
        ringbuf_memcpy_from(input_buf, audio_input_buf, input_bytes);
        freedv_tx(fdv, output_buf, input_buf);
        ringbuf_memcpy_into(audio_output_buf, output_buf, sizeof(short) * freedv_get_n_tx_modem_samples(fdv));
        digitalWrite(ledPin, LOW);   // set the LED on
    }

    while (ringbuf_bytes_used(audio_output_buf) >= 256)
    {
        short buf[128];
        ringbuf_memcpy_from(buf, audio_output_buf, 256);
        send_audio_packet(&arduino_dongle_packet_handlers, buf);
    }
}

