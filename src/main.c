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

#define SYS_CLK 300000000     // System clock frequency (300 MHz)
#define WAVE_BUFFER_SIZE 4096 // Size of the waveform buffer
static uint32_t dma_buffer[WAVE_BUFFER_SIZE];

#define MAX_TABLE_LEN 256         // Maximum length of the waveform lookup table
uint8_t wav_table[MAX_TABLE_LEN]; // Waveform lookup table

uint32_t table_len = MAX_TABLE_LEN; // Actual length of the waveform lookup table

// PIO variables
static PIO pio;
static uint sm;
static uint offset;

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

    sm_config_set_clkdiv(&c, 1);

    // Initialize and enable state machine
    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);
}

// Fill the DMA buffer with a repeating pattern of one or more waveforms
// One of the issues is that DMA is word aligned, so the pattern has to repeat on WORD,
// not byte boundaries.
int fill_dma_buffer(uint32_t *buf, float freq, float fs,
                    const uint8_t *table, int table_len)
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

            packed |= table[index] << (j * 8);
            printf("Index: %d, i: %d\n", index, 4 * i + j);
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
    printf("Generated %d words for DMA\n", i);
    printf("Phase increment: %d\n", phase_inc);
    printf("Index: %d\n", index);
    printf("First index: %d\n", first_index);

    return i; // number of 32-bit words for DMA
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

void generate_triangle_table(uint8_t *table, int table_len)
{
    for (int i = 0; i < table_len; i++)
    {
        if (i < table_len / 2)
        {
            // Rising edge: 0 → 255
            table[i] = (uint8_t)((i * 255) / (table_len / 2 - 1));
        }
        else
        {
            // Falling edge: 255 → 0
            int j = i - table_len / 2;
            table[i] = (uint8_t)(255 - (j * 255) / (table_len / 2 - 1));
        }
    }
}

void generate_square_table(uint8_t *table, int table_len)
{
    for (int i = 0; i < table_len; i++)
    {
        if (i < table_len / 2)
        {
            // Rising edge: 0 → 255
            table[i] = 255;
        }
        else
        {
            // Falling edge: 255 → 0
            table[i] = 0;
        }
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

int main()
{
    initialize();

    float frequency = 150e6;
    generate_sine_table(wav_table, table_len);
    generate_triangle_table(wav_table, table_len);
    //  generate_square_table(wav_table, table_len);
    pio_init();
    int length = fill_dma_buffer(dma_buffer, frequency, SYS_CLK, wav_table, table_len);
    start_dma(dma_buffer, length);

    while (1)
    {

        tight_loop_contents();
    }

    return 0;
}
