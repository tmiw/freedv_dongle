
#include "arduino_freertos.h"
#include "avr/pgmspace.h"
#include "queue.h"
#include <assert.h>

#include "dongle_protocol.h"

#include "ringbuf.h"
#include "freedv_api.h"
#include "reliable_text.h"

const int ledPin = 13;

static struct freedv* fdv = nullptr;
static ringbuf_t audio_input_buf = nullptr;
static ringbuf_t audio_output_buf = nullptr;
static bool in_transmit = false;
static short* input_buf;
static short* output_buf;
static int last_audio_mode_rx = -1;
static reliable_text_t reliable_text_obj = nullptr;
static QueueHandle_t serialTaskQueue = nullptr;
static QueueHandle_t freedvTaskQueue = nullptr;

static void reliable_text_rx_fn(reliable_text_t rt, const char* txt_ptr, int length, void* state)
{
    reliable_text_reset(rt);
}

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
        if (reliable_text_obj != nullptr)
        {
            reliable_text_unlink_from_freedv(reliable_text_obj);
            reliable_text_destroy(reliable_text_obj);
            reliable_text_obj = nullptr;
        }
        
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
        
    input_buf = (short*)extmem_malloc(max_sample_size * sizeof(short));
    assert(input_buf != nullptr);
    output_buf = (short*)extmem_malloc(max_sample_size * sizeof(short));
    assert(output_buf != nullptr);
    
    last_audio_mode_rx = -1;
}

static bool handle_incoming_messages()
{
    struct dongle_packet packet;
    bool valid = false;
    
    if (Serial.available() && read_packet(&arduino_dongle_packet_handlers, &packet) > 0)
    {
        ::xQueueSendToBack(freedvTaskQueue, &packet, pdMS_TO_TICKS(10));
        valid = true;
    }
    
    return valid;
}

static bool process_freedv_command(struct dongle_packet* packet)
{
    bool send_ack = false;
    
    switch (packet->type)
    {
        case DONGLE_PACKET_RX_AUDIO:
        case DONGLE_PACKET_TX_AUDIO:
        {
            // Inbound audio to be processed. Append to ring buffer for later handling.
            if (last_audio_mode_rx != packet->type)
            {
                ringbuf_reset(audio_input_buf);
                ringbuf_reset(audio_output_buf);
                last_audio_mode_rx = packet->type;
            }
            in_transmit = packet->type == DONGLE_PACKET_TX_AUDIO;
            ringbuf_memcpy_into(audio_input_buf, packet->packet_data.audio_data.audio, packet->length);
            break;
        }
        case DONGLE_PACKET_SET_FDV_MODE:
        {
            // Reopen FDV handle using new mode.
            open_freedv_handle(packet->packet_data.fdv_mode_data.mode);
            send_ack = true;
            break;
        }
        case DONGLE_PACKET_SET_CALLSIGN:
        {
            if (reliable_text_obj == nullptr)
            {
                reliable_text_obj = reliable_text_create();
                assert(reliable_text_obj);
                reliable_text_use_with_freedv(reliable_text_obj, fdv, reliable_text_rx_fn, nullptr);
            }
            
            reliable_text_set_string(
                reliable_text_obj, 
                (char*)packet->packet_data.fdv_callsign_data.callsign, 
                strlen((char*)packet->packet_data.fdv_callsign_data.callsign));
            reliable_text_reset(reliable_text_obj);
            
            //SerialUSB1.printf("Callsign set to %s\n", packet.packet_data.fdv_callsign_data.callsign);
            
            send_ack = true;
            break;
        }
        default:
        {
            // Ignore anything we don't recognize.
            break;
        }
    }
    
    return send_ack;
}

static void process_queued_audio()
{
    size_t input_bytes = 0;
    
    if (in_transmit)
    {
        input_bytes = freedv_get_n_speech_samples(fdv) * sizeof(short);
        
        // Initialize LED in off state before processing transmit audio.
        ::digitalWrite(ledPin, arduino::LOW);
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
            ::digitalWrite(ledPin, arduino::HIGH);
            freedv_tx(fdv, output_buf, input_buf);
            ringbuf_memcpy_into(audio_output_buf, output_buf, sizeof(short) * freedv_get_n_tx_modem_samples(fdv));
            ::digitalWrite(ledPin, arduino::LOW);
        }
        else
        {
            int nout = freedv_rx(fdv, output_buf, input_buf);
            ringbuf_memcpy_into(audio_output_buf, output_buf, sizeof(short) * nout);
            input_bytes = freedv_nin(fdv) * sizeof(short);
            
            // Update LED to reflect current sync state.
            ::digitalWrite(ledPin, freedv_get_sync(fdv) ? arduino::HIGH : arduino::LOW);
        }        
    }
}

static void transmit_output_audio()
{
    short buf[DONGLE_AUDIO_LENGTH];
    bool flush = false;
    while (ringbuf_bytes_used(audio_output_buf) >= sizeof(buf))
    {
        flush = true;
        //digitalWrite(ledPin, HIGH);
        ringbuf_memcpy_from(buf, audio_output_buf, sizeof(buf));
        send_audio_packet(&arduino_dongle_packet_handlers, buf, in_transmit);
        //digitalWrite(ledPin, LOW);
    }
    
    if (flush)
    {
        //Serial.send_now();
    }    
}

static void serialTask(void*)
{
    while(true)
    {
        handle_incoming_messages();
        
        {
            int tmp = 0;
            ::xQueueReceive(serialTaskQueue, &tmp, pdMS_TO_TICKS(10));
            if (tmp)
            {
                send_ack_packet(&arduino_dongle_packet_handlers);
            }
        }
        
        transmit_output_audio();
    }
}

static void freedvTask(void*)
{
    while(true)
    {
        struct dongle_packet tmp;
        if (::xQueueReceive(freedvTaskQueue, &tmp, pdMS_TO_TICKS(1)) == pdTRUE)
        {
            bool send_ack = process_freedv_command(&tmp);
            process_queued_audio();
            
            int tmp = send_ack ? 1 : 0;
            ::xQueueSendToBack(serialTaskQueue, &tmp, pdMS_TO_TICKS(10));
        }
    }
}

FLASHMEM __attribute__((noinline)) void setup() {
    ::Serial.begin(115'200);
    ::pinMode(arduino::LED_BUILTIN, arduino::OUTPUT);
    
    // Let user know we've fully started up (3 fast blinks).
    ::digitalWrite(arduino::LED_BUILTIN, arduino::HIGH);
    ::delay(50);
    ::digitalWrite(arduino::LED_BUILTIN, arduino::LOW);
    ::delay(150);
    ::digitalWrite(arduino::LED_BUILTIN, arduino::HIGH);
    ::delay(50);
    ::digitalWrite(arduino::LED_BUILTIN, arduino::LOW);
    ::delay(150);
    ::digitalWrite(arduino::LED_BUILTIN, arduino::HIGH);
    ::delay(50);
    ::digitalWrite(arduino::LED_BUILTIN, arduino::LOW);
    ::delay(150);
    
    open_freedv_handle(FREEDV_MODE_700D);
    
    freedvTaskQueue = ::xQueueCreate(8, sizeof(struct dongle_packet));
    assert(freedvTaskQueue != nullptr);
        
    serialTaskQueue = ::xQueueCreate(8, sizeof(int));
    assert(serialTaskQueue != nullptr);
    
    ::xTaskCreate(serialTask, "serialTask", 32767, nullptr, 2, nullptr);
    ::xTaskCreate(freedvTask, "freedvTask", 32767, nullptr, 2, nullptr);
    
    ::vTaskStartScheduler();
}

void loop() 
{
    // empty
}