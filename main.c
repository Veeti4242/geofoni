#include "hardware/adc.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/irq.h"
#include "hardware/sync.h"
#include "kiss_fft.h"
#include "kiss_fftr.h"
#include "math.h"
#include "pico/stdio_usb.h"
#include "pico/stdlib.h"
#include <hardware/regs/dreq.h>
#include <hardware/regs/intctrl.h>
#include <hardware/timer.h>
#include <pico/stdio.h>
#include <pico/time.h>
#include <pico/types.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define I2C_PORT i2c0
#define I2C_SDA 4
#define I2C_SCL 5
#define I2C_ALRT 6

enum { BLOCK_COUNT = 6, SAMPLES_PER_BLOCK = 256 };
enum { ADS_BLOCK_COUNT = 3, ADS_SAMPLES_PER_BLOCK = 64 };
enum { BLOCK_FREE, BLOCK_READY, BLOCK_FILLING, BLOCK_READING, BLOCK_INVALID };
enum { FFT_LENGTH = 2048 };
enum { ADS_FFT_LENGTH = 1024 };

float freqs[FFT_LENGTH];        // frequency bins for fft
float window[FFT_LENGTH];       // hann window values array
float window_sum = 0;           // sum used for hann window calc
float sample_rate_hz = 8000.0f; // sampling ratei

kiss_fft_cpx fft_out[FFT_LENGTH];   // fft output array
kiss_fft_scalar fft_in[FFT_LENGTH]; // kiss_fft_scalar is a float

uint16_t blocks[BLOCK_COUNT][SAMPLES_PER_BLOCK]; // the actual ring buffed
size_t count = sizeof(blocks[0]) / sizeof(blocks[0][0]);
volatile int state[BLOCK_COUNT] = {BLOCK_FREE, BLOCK_FREE, BLOCK_FREE};

volatile uint32_t block_ids[BLOCK_COUNT]; // increments forever as run goes
volatile uint16_t write_index = 0;
volatile uint16_t read_index = 0;
uint16_t next_block_id = 0;
size_t fft_fill = 0;      // number of samples currently stored in fft_in
uint32_t first_block = 0; // fft array first index
uint32_t last_block = 0;  // fft array last index
volatile uint32_t completed_blocks = 0;

// SDA1115 stuffs
uint8_t conv = 0x00;
uint8_t buffer[2];                              // buf for 2x bytes
volatile uint16_t ads_write_index = 0;          // 0-2
volatile uint16_t ads_write_index_in_block = 0; // 0-63
volatile uint16_t ads_read_index = 0;
volatile bool ADS1115_CAPTURE_IN_PROGRESS = false;
int16_t ads_blocks[ADS_BLOCK_COUNT][ADS_SAMPLES_PER_BLOCK]; // ADS1115 ring buffer
volatile int ads_state[ADS_BLOCK_COUNT] = {BLOCK_FREE, BLOCK_FREE, BLOCK_FREE};
size_t ads_count = sizeof(ads_blocks[0]) / sizeof(ads_blocks[0][0]);
volatile uint32_t ads_block_ids[ADS_BLOCK_COUNT]; // increments forever
volatile uint32_t ads_completed_blocks = 0;       // increments forever
float ads_freqs[ADS_FFT_LENGTH];
float ads_window[ADS_FFT_LENGTH];
float ads_window_sum = 0;
float ads_sample_rate_hz = 867.0f; // module says 860 but measured 867
kiss_fft_cpx ads_fft_out[ADS_FFT_LENGTH];
kiss_fft_scalar ads_fft_in[ADS_FFT_LENGTH];
size_t ads_fft_fill = 0;
uint32_t ads_first_block = 0;
uint32_t ads_last_block = 0;

// metadata flags for main and errors
volatile bool capture_active = false;
volatile bool running = false;
volatile bool consumer_held = false;
volatile bool overflowed = false;
volatile bool fifo_overflowed = false;
volatile bool ads_overflowed = false;

int channel;

dma_channel_config c;
kiss_fftr_cfg cfg;
kiss_fftr_cfg ads_cfg;
static uint16_t raw_to_send[SAMPLES_PER_BLOCK];
static int16_t ads_raw_to_send[ADS_SAMPLES_PER_BLOCK];
static bool output_failed = false;

void handle_completion(void) {
    state[write_index] = BLOCK_READY;
    completed_blocks++;
    block_ids[write_index] = completed_blocks - 1;
    write_index == BLOCK_COUNT - 1 ? write_index = 0 : write_index++;
}

void setup_next_block(void) {                 // setup the next block by
    state[write_index] = BLOCK_FILLING;       // setting the status as filling
    capture_active = true;                    // setup irq0 interrupt flag back down
    dma_irqn_acknowledge_channel(0, channel); // re-arm DMA with settings
    dma_channel_configure(channel, &c, blocks[write_index], &adc_hw->fifo,
                          dma_encode_transfer_count(count), true);
}

void fft_run(void) {  // fft run helper function
    uint64_t sum = 0; // sum so we can get average
    for (int i = 0; i < FFT_LENGTH; i++) {
        sum += fft_in[i];
    }
    float avg = (float)sum / (FFT_LENGTH);                // average for DC offset
    for (int i = 0; i < FFT_LENGTH; i++) {                // run through FFT values
        fft_in[i] = window[i] * ((float)fft_in[i] - avg); // insert captured values with DC offset
    }
    if (cfg != NULL) {
        kiss_fftr(cfg, fft_in, fft_out); // actual fft run
    }
    if (cfg == NULL) {
        printf("{\"type\":\"error\",\"code\":\"fft_allocation_failed\","
               "\"message\":\"FFT allocation failed. Reset the Pico.\"}\n");
    }
}

void ads_fft_run(void) {
    int64_t sum = 0; // sum so we can get average
    for (int i = 0; i < ADS_FFT_LENGTH; i++) {
        sum += ads_fft_in[i];
    }
    float avg = (float)sum / (ADS_FFT_LENGTH); // average for DC offset
    for (int i = 0; i < ADS_FFT_LENGTH; i++) { // run through FFT values
        ads_fft_in[i] =
            ads_window[i] * ((float)ads_fft_in[i] - avg); // insert captured values with DC offset
    }
    if (ads_cfg != NULL) {
        kiss_fftr(ads_cfg, ads_fft_in, ads_fft_out); // actual fft run
    }
    if (ads_cfg == NULL) {
        printf("{\"type\":\"error\",\"code\":\"fft_allocation_failed\","
               "\"message\":\"FFT allocation failed. Reset the Pico.\"}\n");
    }
}

void calc_magnitudes(void) {
    float max_power = 0;
    int max_idx = 0;
    // / 2 because nyqiust
    for (int i = 1; i <= FFT_LENGTH / 2; i++) {
        float power = // calc magnitudes for each fft output value
                      // (real^2 + imaginary^2)
            fft_out[i].r * fft_out[i].r + fft_out[i].i * fft_out[i].i; //
        if (power > max_power) {
            max_power = power;
            max_idx = i;
        }
    }

    float max_freq = freqs[max_idx];
    float amplitude = sqrtf(max_power) / window_sum;

    if (max_idx > 0 && max_idx < FFT_LENGTH / 2) {
        amplitude *= 2.0f;
    }
    // fft stats
    if (overflowed || fifo_overflowed)
        return;
    printf("{\"type\":\"fft\",\"sensor\":\"adc\",\"peak_frequency_hz\":%.2f,"
           "\"peak_bin\":%d,\"peak_amplitude\":%.4f,\"values\":[",
           max_freq, max_idx, amplitude);

    // calc freq bins of ffty
    for (int i = 0; i <= FFT_LENGTH / 2; i++) {
        float power =
            fft_out[i].r * fft_out[i].r + fft_out[i].i * fft_out[i].i; // real^2 + imaginary^2
        float bin_amplitude = sqrtf(power) / window_sum;               // 1028
        if (i > 0 && i < FFT_LENGTH / 2) {
            bin_amplitude *= 2.0f;
        }
        if (i > 0)
            printf(",");
        printf("%.4f", bin_amplitude);
    }
    printf("]}\n");
}

void ads_calc_magnitudes(void) {
    float max_power = 0;
    int max_idx = 0;
    // / 2 because nyqiust
    for (int i = 1; i <= ADS_FFT_LENGTH / 2; i++) {
        float power = // calc magnitudes for each fft output value
                      // (real^2 + imaginary^2)
            ads_fft_out[i].r * ads_fft_out[i].r + ads_fft_out[i].i * ads_fft_out[i].i; //
        if (power > max_power) {
            max_power = power;
            max_idx = i;
        }
    }

    float max_freq = ads_freqs[max_idx];
    float amplitude = sqrtf(max_power) / ads_window_sum;

    if (max_idx > 0 && max_idx < ADS_FFT_LENGTH / 2) {
        amplitude *= 2.0f;
    }
    // fft stats
    if (ads_overflowed)
        return;
    printf("{\"type\":\"fft\",\"sensor\":\"ads1115\",\"peak_frequency_hz\":%.2f,"
           "\"peak_bin\":%d,\"peak_amplitude\":%.4f,\"values\":[",
           max_freq, max_idx, amplitude);

    // calc freq bins of ffty
    for (int i = 0; i <= ADS_FFT_LENGTH / 2; i++) {
        float power = ads_fft_out[i].r * ads_fft_out[i].r +
                      ads_fft_out[i].i * ads_fft_out[i].i;   // real^2 + imaginary^2
        float bin_amplitude = sqrtf(power) / ads_window_sum; // 1028
        if (i > 0 && i < ADS_FFT_LENGTH / 2) {
            bin_amplitude *= 2.0f;
        }
        if (i > 0)
            printf(",");
        printf("%.4f", bin_amplitude);
    }
    printf("]}\n");
}

void send_usb(uint32_t block_id) {
    // printf already goes to USB. Send one raw block as one JSON line.
    printf("{\"type\":\"samples\",\"sensor\":\"adc\",\"first_sample_index\":%llu,\"values\":[",
           (unsigned long long)block_id * SAMPLES_PER_BLOCK);
    for (size_t i = 0; i < SAMPLES_PER_BLOCK; i++) {
        if (i > 0)
            printf(",");
        printf("%u", (unsigned)raw_to_send[i]);
    }
    printf("]}\n");
}

void ads_send_usb(uint32_t block_id) {
    // printf already goes to USB. Send one raw block as one JSON line.
    printf("{\"type\":\"samples\",\"sensor\":\"ads1115\",\"first_sample_index\":%llu,\"values\":[",
           (unsigned long long)block_id * ADS_SAMPLES_PER_BLOCK);
    for (size_t i = 0; i < ADS_SAMPLES_PER_BLOCK; i++) {
        if (i > 0)
            printf(",");
        printf("%d", (int)ads_raw_to_send[i]);
    }
    printf("]}\n");
}

void consume_one_block(void) {
    last_block = block_ids[read_index];
    if (fft_fill == 0) {
        first_block = block_ids[read_index];
    }

    if (fft_fill + SAMPLES_PER_BLOCK <= FFT_LENGTH) {
        for (int i = 0; i < SAMPLES_PER_BLOCK; i++) { // i = offset
            fft_in[fft_fill + i] = (float)blocks[read_index][i];
            raw_to_send[i] = blocks[read_index][i];
        }
        fft_fill += SAMPLES_PER_BLOCK;
    } else {
        output_failed = true;
        return;
    }

    __dmb();
    state[read_index] = BLOCK_FREE;
    read_index == BLOCK_COUNT - 1 ? read_index = 0 : read_index++;

    if (!overflowed && !fifo_overflowed) {
        send_usb(last_block);
    }

    if (fft_fill == FFT_LENGTH) {
        if (!overflowed && !fifo_overflowed) {
            if (cfg != NULL) {
                // uint64_t start = time_us_64();
                fft_run();
                // uint64_t processing_us = time_us_64() - start;
                // printf("mean/hann/fft took %llu \n", processing_us);
                if (!overflowed && !fifo_overflowed) {
                    calc_magnitudes();
                }
            }
        }
        fft_fill = 0; // reset after full fft array
    }
}

void on_dma_complete(void) {
    if (dma_irqn_get_channel_status(0, channel)) {
        dma_irqn_acknowledge_channel(0, channel);
        if (!running || overflowed || fifo_overflowed) {
            return;
        }
        // if bit = 1 we fifo overfwlowed
        if ((adc_hw->fcs & ADC_FCS_OVER_BITS) != 0) {
            adc_run(false);
            state[write_index] = BLOCK_INVALID;
            fifo_overflowed = true;
            running = false;
            capture_active = false;
            return;
        }

        else {
            __dmb();
            handle_completion(); // block state = READY and increment write
                                 // index
            if (state[write_index] == BLOCK_FREE) {
                state[write_index] = BLOCK_FILLING;
                capture_active = true;
                setup_next_block(); // setup next block to FILLING and rearm DMA
            }

            else { // something has gone wrong
                adc_run(false);
                running = false;
                capture_active = false;
                overflowed = true;
                return;
            }
        }
    }

    else {
        return;
    }
}

void ads_handle_completion(void) {
    ads_state[ads_write_index] = BLOCK_READY;
    ads_completed_blocks++;
    ads_block_ids[ads_write_index] = ads_completed_blocks - 1;
    ads_write_index == ADS_BLOCK_COUNT - 1 ? ads_write_index = 0 : ads_write_index++;
}

void ads_setup_next_block(void) {
    ads_state[ads_write_index] = BLOCK_FILLING;
    ads_write_index_in_block = 0; // reset in block
}

void ads_consume_one_block(void) {
    ads_last_block = ads_block_ids[ads_read_index];
    if (ads_fft_fill == 0) {
        ads_first_block = ads_block_ids[ads_read_index];
    }

    if (ads_fft_fill + ADS_SAMPLES_PER_BLOCK <= ADS_FFT_LENGTH) {
        for (int i = 0; i < ADS_SAMPLES_PER_BLOCK; i++) {
            ads_fft_in[ads_fft_fill + i] = (float)ads_blocks[ads_read_index][i];
            ads_raw_to_send[i] = ads_blocks[ads_read_index][i];
        }
        ads_fft_fill += ADS_SAMPLES_PER_BLOCK;
    }

    __dmb();
    ads_state[ads_read_index] = BLOCK_FREE;
    ads_read_index == ADS_BLOCK_COUNT - 1 ? ads_read_index = 0 : ads_read_index++;

    if (!ads_overflowed) {
        ads_send_usb(ads_last_block);
    }

    if (ads_fft_fill == ADS_FFT_LENGTH) {
        if (!ads_overflowed) {
            if (ads_cfg != NULL) {
                ads_fft_run();
                ads_calc_magnitudes();
            }
        }
        ads_fft_fill = 0;
    }
}

void callback(uint gpio, uint32_t events) {
    if (ADS1115_CAPTURE_IN_PROGRESS) {

        int status = i2c_read_blocking(I2C_PORT, 0x48, buffer, 2, false);
        ads_blocks[ads_write_index][ads_write_index_in_block] = (buffer[0] << 8) | buffer[1];
        ads_write_index_in_block++;

        if (ads_write_index_in_block == (ADS_SAMPLES_PER_BLOCK)) {
            __dmb();
            ads_handle_completion();
            if (ads_state[ads_write_index] == BLOCK_FREE) {
                ads_state[ads_write_index] = BLOCK_FILLING;
                ads_setup_next_block();
            } else {
                ADS1115_CAPTURE_IN_PROGRESS = false;
                ads_overflowed = true;
                return;
            }
        }
    }
}

static void setup(void) {

    stdio_init_all();

    // ADC SIDE SETUP
    adc_init();
    adc_gpio_init(26);
    adc_select_input(0);
    adc_set_round_robin(0);
    adc_fifo_setup(true, true, 1, false, false);

    channel = dma_claim_unused_channel(true);
    irq_set_exclusive_handler(DMA_IRQ_0, on_dma_complete);
    dma_irqn_acknowledge_channel(0, channel);
    dma_channel_set_irq0_enabled(channel, true);
    irq_set_enabled(DMA_IRQ_0, true);

    float divider = 48000000.0 / sample_rate_hz - 1.0;
    adc_set_clkdiv(divider);

    c = dma_channel_get_default_config(channel);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_16);
    channel_config_set_read_increment(&c, false); // fifo register does not increment
    channel_config_set_write_increment(&c, true);
    channel_config_set_dreq(&c, DREQ_ADC);

    // ADS1115 SIDE SETUP
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_init(I2C_ALRT);
    gpio_set_dir(I2C_ALRT, GPIO_IN);
    gpio_pull_up(I2C_ALRT);

    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    uint8_t lo_thresh[3] = {0x02, 0x00, 0x00};
    uint8_t hi_thresh[3] = {0x03, 0x80, 0x00};
    uint8_t config[3] = {0x01, 0x04, 0xE0};
    i2c_write_blocking(I2C_PORT, 0x48, lo_thresh, 3, false);
    i2c_write_blocking(I2C_PORT, 0x48, hi_thresh, 3, false);
    i2c_write_blocking(I2C_PORT, 0x48, config, 3, false);
    i2c_write_blocking(I2C_PORT, 0x48, &conv, 1, false);
    gpio_set_irq_enabled_with_callback(I2C_ALRT, GPIO_IRQ_EDGE_FALL, true, &callback);
}
// one config line per sensor
void print_configs(void) {
    printf("{\"type\":\"config\",\"sensor\":\"adc\",\"sample_rate_hz\":%.1f,"
           "\"fft_length\":%d,\"spectrum_units\":\"ADC counts peak\"}\n",
           sample_rate_hz, FFT_LENGTH);
    printf("{\"type\":\"config\",\"sensor\":\"ads1115\",\"sample_rate_hz\":%.1f,"
           "\"fft_length\":%d,\"spectrum_units\":\"ADS1115 counts peak\"}\n",
           ads_sample_rate_hz, ADS_FFT_LENGTH);
}

void frequency_bins_calc() {
    const float pi = 3.14159265358979323846f;
    // insert freq values into freq bins
    float f_max = sample_rate_hz;
    float f_res = f_max / FFT_LENGTH;
    for (int i = 0; i < FFT_LENGTH; i++) {
        freqs[i] = f_res * i;
    }

    // generate hann window multipliers for each sample
    for (int i = 0; i < FFT_LENGTH; i++) {
        window[i] = 0.5 - 0.5 * cosf(2 * pi * i / FFT_LENGTH);
        window_sum += window[i];
    }
    // just copy paste ads version etc
    float ads_f_max = ads_sample_rate_hz;
    float ads_f_res = ads_f_max / ADS_FFT_LENGTH;
    for (int i = 0; i < ADS_FFT_LENGTH; i++) {
        ads_freqs[i] = ads_f_res * i;
    }
    for (int i = 0; i < ADS_FFT_LENGTH; i++) {
        ads_window[i] = 0.5 - 0.5 * cosf(2 * pi * i / ADS_FFT_LENGTH);
        ads_window_sum += ads_window[i];
    }
}

int main(void) {
    setup(); // setup ADC, DMA , interrupt
    cfg = kiss_fftr_alloc(FFT_LENGTH, false, NULL, NULL);
    ads_cfg = kiss_fftr_alloc(ADS_FFT_LENGTH, false, NULL, NULL);
    frequency_bins_calc();
    bool fault_reported = false;

    // wait till usb conncted before main loop
    while (!stdio_usb_connected())
        sleep_ms(100);
    sleep_ms(300);

    if (cfg == NULL) {
        printf("{\"type\":\"error\",\"code\":\"fft_allocation_failed\","
               "\"message\":\"FFT allocation failed. Reset the Pico.\"}\n");
        while (true)
            sleep_ms(10);
    }

    if (ads_cfg == NULL) {
        printf("{\"type\":\"error\",\"code\":\"fft_allocation_failed\","
               "\"message\":\"FFT allocation failed. Reset the Pico.\"}\n");
        while (true)
            sleep_ms(10);
    }

    print_configs();
    uint64_t last_config_us = time_us_64();

    if (!output_failed) {
        adc_fifo_drain();
        adc_hw->fcs |= ADC_FCS_OVER_BITS | ADC_FCS_UNDER_BITS; // combine 2 masks and add it to
                                                               // register (writing 1 clears them)
        running = true;
        setup_next_block();
        adc_run(true);

        // ADS starter
        ads_setup_next_block();
        ADS1115_CAPTURE_IN_PROGRESS = true;
    }

    while (true) {
        if (running && !stdio_usb_connected())
            output_failed = true;

        // output failed stop everything
        if (output_failed && running) {
            adc_run(false);
            running = false; //
            dma_channel_abort(channel);
            capture_active = false;
        }

        if (overflowed || fifo_overflowed || output_failed) {
            fft_fill = 0;
            if (!fault_reported && stdio_usb_connected()) {
                adc_fifo_drain();
                const char *code = fifo_overflowed ? "adc_fifo_overflow"
                                   : overflowed    ? "acquisition_overflow"
                                                   : "usb_output_failed";
                printf("{\"type\":\"error\",\"code\":\"%s\","
                       "\"message\":\"Measurement stopped. Reset the Pico to "
                       "restart.\"}\n",
                       code);
                fault_reported = true;
            }
            sleep_ms(1);
            continue;
        }

        if (time_us_64() - last_config_us >= 1000000) {
            print_configs();
            last_config_us = time_us_64();
        }
        if (!output_failed && !overflowed && !fifo_overflowed && state[read_index] == BLOCK_READY) {
            __dmb();
            consume_one_block();
        }
        if (!ads_overflowed && ads_state[ads_read_index] == BLOCK_READY) {
            __dmb();
            ads_consume_one_block();
        }
    }
}
