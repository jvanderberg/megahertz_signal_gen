#include "pico/stdlib.h"
#include "pico/platform.h"
#include <stdio.h>
#include <math.h>
#include "hardware/pwm.h"
#include "hardware/adc.h"

#include <stdint.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "dac_high.pio.h"
#include "hardware/clocks.h"
#include "hardware/vreg.h"
#include "dadadadum.h"
#include "hardware/timer.h"
#include "hardware/irq.h"

#define TS_US 125
#define SYS_CLK 300000000 // System clock frequency (300 MHz)
static uint32_t sys_clk = SYS_CLK;
#define PIO_CLK_DIV 1         // PIO clock divider
#define WAVE_BUFFER_SIZE 4096 // Size of the waveform buffer
static uint32_t save_dma_buffer[WAVE_BUFFER_SIZE];
static uint32_t dma_buffer[WAVE_BUFFER_SIZE];
static uint32_t buffer_size = WAVE_BUFFER_SIZE;

static float frequency = 1.7e6;

#define MAX_TABLE_LEN 16          // Maximum length of the waveform lookup table
uint8_t wav_table[MAX_TABLE_LEN]; // Waveform lookup table

uint32_t table_len = MAX_TABLE_LEN; // Actual length of the waveform lookup table

// PIO variables
static PIO pio;
static uint sm;
static uint offset;

static int index = 0; // Index for dadadadum_wav

#define BASE_PIN 1 // first GPIO of your 8-bit R-2R DAC
static pio_sm_config c;
// Initialize PIO
void pio_init()
{

    bool success = pio_claim_free_sm_and_add_program_for_gpio_range(&dac_high_program, &pio, &sm, &offset, BASE_PIN, 8, false);

    hard_assert(success);
    printf("PIO Success: %d\n", success);
    for (int i = 0; i < 8; i++)
    {
        pio_gpio_init(pio, BASE_PIN + i);
    }

    // Load default config for our program
    c = dac_high_program_get_default_config(offset);

    // Map 8 pins for DAC output
    sm_config_set_out_pins(&c, BASE_PIN, 8);
    int result = pio_sm_set_consecutive_pindirs(pio, sm, BASE_PIN, 8, true);

    // Configure shift: autopull, shift right, 32 bits at a time
    sm_config_set_out_shift(&c, true, true, 32);

    // Clock divider run at SYS_CLK / PIO_CLK_DIV
    sm_config_set_clkdiv(&c, (float)SYS_CLK / (float)sys_clk);

    // Initialize and enable state machine
    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);
}

// Fill the DMA buffer with a repeating pattern of one or more waveforms
// One of the issues is that DMA is word aligned, so the pattern has to repeat on WORD,
// not byte boundaries.
int fill_dma_buffer(uint32_t *buf, float freq, float fs,
                    const uint8_t *table, int table_len, uint8_t amplitude)
{

    int idx_bits = __builtin_ctz(table_len); // Get the number of bits needed to represent the table index
    uint32_t idx_mask = (uint32_t)table_len - 1;

    uint32_t phase = 0; // Current phase accumulator

    // Calculate phase increment for each sample
    uint32_t phase_inc = (uint32_t)(((uint64_t)freq << 32) / (uint64_t)fs);

    int i = 0;
    uint32_t index;
    int first_index = -1;
    while (i < WAVE_BUFFER_SIZE && (i < table_len || (i >= table_len && index != first_index)))
    {

        uint32_t packed = 0;
        for (int j = 0; j < 4; j++)
        {
            phase += phase_inc;
            index = phase >> (32 - idx_bits) & idx_mask;

            packed |= table[index] * amplitude / 255 << (j * 8);
        }
        if (first_index == -1)
        {
            first_index = index; // remember the first index
        }
        buf[i] = packed;

        i++;
    }
    // throw away the last word, it's a repeat
    i--;

    return i; // number of 32-bit words for DMA
}
// One LUT per amplitude setting would be 256×256 entries (~64 KB).
// Too big. Instead, compute per amplitude when it changes.
static uint8_t scale_lut[256];

// Call whenever amplitude changes (costly only once per change).
void build_scale_lut(uint8_t amplitude)
{
    for (int i = 0; i < 256; i++)
    {
        scale_lut[i] = (uint8_t)((i * amplitude) / 255u);
    }
}

void build_scale_lut2(uint8_t amplitude)
{
    for (int i = 0; i < 256; i++)
    {
        int16_t centered = i - 128; // shift to signed range [-128,127]
        int16_t scaled = (centered * amplitude) / 255;
        int16_t result = scaled + 128;

        // clamp to 0..255
        if (result < 0)
            result = 0;
        else if (result > 255)
            result = 255;

        scale_lut[i] = (uint8_t)result;
    }
}
void amplify_dma_buffer(const uint32_t *orig, uint32_t *buf, uint32_t size)
{
    for (uint32_t i = 0; i < size; i++)
    {
        uint32_t w = orig[i];
        uint8_t s0 = scale_lut[(w >> 0) & 0xFF];
        uint8_t s1 = scale_lut[(w >> 8) & 0xFF];
        uint8_t s2 = scale_lut[(w >> 16) & 0xFF];
        uint8_t s3 = scale_lut[(w >> 24) & 0xFF];
        buf[i] = (uint32_t)s0 |
                 ((uint32_t)s1 << 8) |
                 ((uint32_t)s2 << 16) |
                 ((uint32_t)s3 << 24);
    }
}

void generate_sine_table(uint8_t *table, int table_len)
{
    for (int n = 0; n < table_len; n++)
    {
        // angle from 0 to 2π
        double theta = (2.0 * M_PI * n) / table_len;
        // scale sine from [-1, 1] to [0, 255]
        double s = (sin(theta) + 1.0) * 126.5;
        table[n] = (uint8_t)lrint(s); // round to nearest integer
    }
}

int dma_chan_a, dma_chan_b;

/**
 * @brief Start chained DMA to continuously stream a waveform buffer into a PIO state machine.
 *
 * This function sets up two DMA channels:
 *   - Channel A transfers `length` bytes from `buffer[]` into the PIO TX FIFO (8-bit per transfer).
 *   - Channel B re-arms Channel A by writing the buffer’s start address back into Channel A’s
 *     read pointer when Channel A completes.
 *
 * Together, the two channels form a self-looping DMA chain that continuously outputs the buffer
 * into the PIO pins with no CPU intervention. The transfer count exactly matches `length`, so
 * the buffer length determines the output frequency.
 *
 * @param pio     PIO instance (pio0 or pio1).
 * @param sm      State machine index within the PIO block.
 * @param buffer  Pointer to the waveform buffer containing pre-expanded 8-bit samples.
 * @param length  Number of samples in the buffer; must match samples-per-cycle for desired frequency.
 */
void start_dma(uint32_t *buffer, int length)
{
    printf("Starting DMA: buffer %p length %d\n", buffer, length);

    // --- Channel A: stream buffer to PIO ---
    dma_chan_a = dma_claim_unused_channel(true);
    dma_chan_b = dma_claim_unused_channel(true);
    dma_channel_config c = dma_channel_get_default_config(dma_chan_a);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, true); // <-- advance
    channel_config_set_write_increment(&c, false);
    channel_config_set_dreq(&c, pio_get_dreq(pio, sm, true));
    channel_config_set_chain_to(&c, dma_chan_b);
    dma_channel_configure(
        dma_chan_a,
        &c,
        &pio->txf[sm], // dest
        buffer,        // src
        length,        // count
        false);

    //--- Channel B: reset channel A’s read_addr ---

    dma_channel_config c2 = dma_channel_get_default_config(dma_chan_b);
    channel_config_set_transfer_data_size(&c2, DMA_SIZE_32);
    channel_config_set_read_increment(&c2, false);
    channel_config_set_write_increment(&c2, false);
    channel_config_set_chain_to(&c2, dma_chan_a);

    static uint32_t buf_addr; // keep alive in RAM
    buf_addr = (uint32_t)buffer;

    dma_channel_configure(
        dma_chan_b,
        &c2,
        &dma_hw->ch[dma_chan_a].read_addr, // dest
        &buf_addr,                         // src = variable holding the buffer start address
        1,
        false);

    // Kick it off
    dma_channel_start(dma_chan_a);
}

// Clock speed and other init
void initialize()
{
    stdio_init_all();
    // Overclock the Pico
    vreg_set_voltage(VREG_VOLTAGE_1_30);
    sleep_ms(500);
    setup_default_uart();
    set_sys_clock_khz(SYS_CLK / 1000, true);
    setup_default_uart();
    // Initialize the Pico SDK standard I/O (USB/UART)
    stdio_usb_init();
    sleep_ms(1000);
}

int count = 0;
void __isr __time_critical_func(timer0_irq)()
{
    // Ack alarm 0
    timer_hw->intr = 1u << 0;

    uint8_t ampl = dadadadum_wav[index] / 2;

    // Output next sample
    build_scale_lut(ampl);                                        // build scale LUT for current amplitude
    amplify_dma_buffer(save_dma_buffer, dma_buffer, buffer_size); // start_dma(dma_buffer, length);
    index++;
    count++;
    if (index >= sizeof(dadadadum_wav) / sizeof(dadadadum_wav[0]))
        index = 0;

    // Next tick
    timer_hw->alarm[0] = timer_hw->timerawl + TS_US;
}

int main()
{
    initialize();

    generate_sine_table(wav_table, table_len);

    pio_init();
    buffer_size = fill_dma_buffer(save_dma_buffer, frequency, sys_clk, wav_table, table_len, 255);
    build_scale_lut(255); // build scale LUT for full amplitude
    amplify_dma_buffer(save_dma_buffer, dma_buffer, buffer_size);
    start_dma(dma_buffer, buffer_size);

    timer_hw->alarm[0] = timer_hw->timerawl + TS_US;
    irq_set_exclusive_handler(TIMER0_IRQ_0, timer0_irq);
    irq_set_enabled(TIMER0_IRQ_0, true);
    timer_hw->inte = 1u << 0;

    while (1)
    {

        tight_loop_contents();
        sleep_ms(1000);
        printf("Count: %d, Index: %d, Amplitude: %d\n", count, index, dadadadum_wav[index]);
    }

    return 0;
}
