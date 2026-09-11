/*
 * spi_sim.c — Userspace SPI protocol simulator
 *
 * Goal: learn SPI at the *signal* level (SCLK, MOSI, MISO, CS) without any
 * real hardware. Nothing here talks to a device driver or a physical bus —
 * it's plain C structs and functions standing in for wires, stepped one
 * clock edge at a time so you can watch exactly when data is sampled and
 * when it's shifted, for each of the 4 SPI modes (CPOL/CPHA combinations).
 *
 * Build:   gcc -Wall -Wextra -o spi_sim spi_sim.c
 * Run:     ./spi_sim
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* ---------------------------------------------------------------------
 * Bus signals
 *
 * A real SPI bus is exactly these four wires (for a single slave, no
 * daisy-chaining). We model them as plain variables that master and
 * slave both read/write, the same way physical GPIO pins would be
 * shared between two chips.
 * ------------------------------------------------------------------- */
typedef struct {
    bool sclk;   /* serial clock, driven by master           */
    bool mosi;   /* master-out-slave-in, driven by master     */
    bool miso;   /* master-in-slave-out, driven by slave       */
    bool cs;     /* chip select, active LOW, driven by master */
} spi_bus_t;

/* SPI mode = (CPOL, CPHA)
 *   CPOL: clock polarity -> idle level of SCLK (0 = idle low, 1 = idle high)
 *   CPHA: clock phase    -> which edge data is SAMPLED on
 *
 *  CPOL = 0 -> clock idle state is low, leading edge is rising, trailing edge is falling
 *  CPOL = 1 -> clock idle state is high, leading edge is falling, trailing edge is rising
 *  CPHA = 0 -> data is sampled on the leading edge (first edge after idle)
 *  CPHA = 1 -> data is sampled on the trailing edge (second edge after idle)
 * 
 * Mode 0 (CPOL=0,CPHA=0): idle low,  sample on rising edge,  shift on falling
 * Mode 1 (CPOL=0,CPHA=1): idle low,  sample on falling edge, shift on rising
 * Mode 2 (CPOL=1,CPHA=0): idle high, sample on falling edge, shift on rising
 * Mode 3 (CPOL=1,CPHA=1): idle high, sample on rising edge,  shift on falling
 */
typedef struct {
    int cpol;
    int cpha;
} spi_mode_t;

static spi_mode_t MODE0 = {0, 0};
static spi_mode_t MODE1 = {0, 1};
static spi_mode_t MODE2 = {1, 0};
static spi_mode_t MODE3 = {1, 1};

/* ---------------------------------------------------------------------
 * Master
 * ------------------------------------------------------------------- */
typedef struct {
    spi_mode_t mode;
    bool msb_first;
} spi_master_t;

/* ---------------------------------------------------------------------
 * Slave — a tiny simulated "register file" device.
 *
 * Protocol (made up, but modeled on real SPI EEPROM/sensor conventions):
 *   Byte 0 from master: command
 *     0x03 = READ  (followed by: address byte, then master clocks a
 *                   dummy byte out while slave shifts back reg[addr])
 *     0x02 = WRITE (followed by: address byte, then data byte)
 *   While CS is high (deselected), the slave ignores the bus entirely —
 *   this is what lets multiple slaves share MOSI/SCLK on real hardware.
 * ------------------------------------------------------------------- */
#define SLAVE_NUM_REGS 8

typedef struct {
    uint8_t regs[SLAVE_NUM_REGS];
    uint8_t shift_out;   /* byte currently being clocked onto MISO */
    uint8_t shift_in;    /* byte currently being assembled from MOSI */
    int     bit_count;
    int     phase;       /* 0 = expecting command, 1 = expecting address,
                             2 = expecting data (write) or clocking data (read) */
    int     cmd;
    int     addr;
} spi_slave_t;

static void slave_reset(spi_slave_t *s) {
    s->shift_in  = 0;
    s->bit_count = 0;
    s->phase     = 0;
    s->cmd       = 0;
    s->addr      = 0;
}

static void slave_init(spi_slave_t *s) {
    memset(s->regs, 0, sizeof(s->regs));
    slave_reset(s);
}

/* Called once per bit, at the moment the slave should SAMPLE mosi and
 * drive its next output bit onto miso. Mirrors what a real slave's
 * shift register does in hardware. */
static void slave_bit(spi_slave_t *s, bool mosi_bit, bool *miso_bit_out) {
    /* Drive miso with the current top bit of shift_out (MSB-first) */
    *miso_bit_out = (s->shift_out >> 7) & 1;

    /* Shift the incoming bit into shift_in, MSB first */
    s->shift_in = (uint8_t)((s->shift_in << 1) | (mosi_bit ? 1 : 0));
    s->shift_out = (uint8_t)(s->shift_out << 1);
    s->bit_count++;

    if (s->bit_count == 8) {
        s->bit_count = 0;
        uint8_t byte = s->shift_in;
        s->shift_in = 0;

        switch (s->phase) {
        case 0: /* just received command byte */
            s->cmd = byte;
            s->phase = 1;
            break;
        case 1: /* just received address byte */
            s->addr = byte % SLAVE_NUM_REGS;
            if (s->cmd == 0x02) {
                s->phase = 2; /* WRITE: next byte is data */
            } else if (s->cmd == 0x03) {
                s->phase = 2; /* READ: pre-load shift_out with reg value */
                s->shift_out = s->regs[s->addr];
            }
            break;
        case 2:
            if (s->cmd == 0x02) {
                s->regs[s->addr] = byte; /* WRITE data received */
            }
            /* READ: master already received the byte via miso during
               this same phase, nothing more to store */
            slave_reset(s);
            break;
        default:
            slave_reset(s);
            break;
        }
    }
}

/* ---------------------------------------------------------------------
 * The actual bus transaction: clock one full byte in both directions
 * simultaneously (SPI is always full-duplex — every clock edge moves a
 * bit each way whether or not you "care" about one direction).
 * ------------------------------------------------------------------- */
static uint8_t spi_transfer_byte(spi_bus_t *bus, spi_master_t *m,
                                  spi_slave_t *s, uint8_t tx_byte,
                                  bool verbose) {
    uint8_t rx_byte = 0;

    for (int i = 0; i < 8; i++) {
        int bit_index = m->msb_first ? (7 - i) : i;
        bool tx_bit = (tx_byte >> bit_index) & 1;

        /* --- First edge of this bit cell --- */
        bus->sclk = !bus->sclk;
        bool first_edge_is_sample =
            (m->mode.cpol == 0 && m->mode.cpha == 0) ||
            (m->mode.cpol == 1 && m->mode.cpha == 1);
        /* CPHA=0 samples on the leading edge; CPHA=1 shifts (sets up
           data) on the leading edge and samples on the trailing edge. */

        if (m->mode.cpha == 0) {
            /* leading edge = sample edge */
            bus->mosi = tx_bit;              /* master already set up data before clocking */
            bool miso_bit;
            slave_bit(s, bus->mosi, &miso_bit);
            bus->miso = miso_bit;
            if (bit_index == (m->msb_first ? 7 : 0)) {
                /* nothing extra; kept for clarity */
            }
            rx_byte = (uint8_t)((rx_byte << 1) | (bus->miso ? 1 : 0));
            if (verbose)
                printf("  bit %d: SCLK=%d MOSI=%d MISO=%d  (sample on leading edge)\n",
                       i, bus->sclk, bus->mosi, bus->miso);
            bus->sclk = !bus->sclk; /* trailing edge: shift/setup next bit */
        } else {
            /* leading edge = setup edge, trailing edge = sample edge */
            bus->mosi = tx_bit;
            if (verbose)
                printf("  bit %d: SCLK=%d MOSI=%d (setup on leading edge)\n",
                       i, bus->sclk, bus->mosi);
            bus->sclk = !bus->sclk; /* trailing edge */
            bool miso_bit;
            slave_bit(s, bus->mosi, &miso_bit);
            bus->miso = miso_bit;
            rx_byte = (uint8_t)((rx_byte << 1) | (bus->miso ? 1 : 0));
            if (verbose)
                printf("          SCLK=%d MISO=%d  (sample on trailing edge)\n",
                       bus->sclk, bus->miso);
        }
        (void)first_edge_is_sample; /* documents the general rule above */
    }

    return rx_byte;
}

static void spi_select(spi_bus_t *bus, spi_master_t *m, bool select) {
    bus->cs = !select; /* active low */
    bus->sclk = (m->mode.cpol == 1); /* clock idles at CPOL level while selected */
}

/* ---------------------------------------------------------------------
 * Demo
 * ------------------------------------------------------------------- */
static void run_transaction(spi_master_t *m, spi_slave_t *s,
                             const char *label,
                             uint8_t cmd, uint8_t addr, uint8_t data,
                             bool is_read) {
    spi_bus_t bus = {0};
    printf("\n=== %s ===\n", label);

    spi_select(&bus, m, true);
    printf("CS asserted (LOW), SCLK idles at %d\n", bus.sclk);

    printf("-- clocking command byte 0x%02X --\n", cmd);
    spi_transfer_byte(&bus, m, s, cmd, true);

    printf("-- clocking address byte 0x%02X --\n", addr);
    spi_transfer_byte(&bus, m, s, addr, true);

    if (is_read) {
        printf("-- clocking dummy byte, capturing slave's reply --\n");
        uint8_t reply = spi_transfer_byte(&bus, m, s, 0x00, true);
        printf("Master received: 0x%02X\n", reply);
    } else {
        printf("-- clocking data byte 0x%02X --\n", data);
        spi_transfer_byte(&bus, m, s, data, true);
    }

    spi_select(&bus, m, false);
    printf("CS deasserted (HIGH) — transaction complete\n");
}

int main(void) {
    spi_master_t master = { .mode = MODE0, .msb_first = true };
    spi_slave_t slave;
    slave_init(&slave);

    printf("SPI protocol simulator — Mode %d (CPOL=%d, CPHA=%d)\n",
           0, master.mode.cpol, master.mode.cpha);

    /* Write 0xAB into register 3 */
    run_transaction(&master, &slave, "WRITE reg[3] = 0xAB",
                     0x02, 0x03, 0xAB, true);

    /* Read it back */
    run_transaction(&master, &slave, "READ reg[3]",
                     0x03, 0x03, 0x00, true);

    printf("\nFinal register file: ");
    for (int i = 0; i < SLAVE_NUM_REGS; i++)
        printf("[%d]=0x%02X ", i, slave.regs[i]);
    printf("\n");

    return 0;
}
