/*
 * Board interface for the Musée Bolo CBM teensy 2 
 * Copyright (c) 2021 Cédric Gaudin <cedric.gaudin@museebolo.ch>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */
#ifndef _BOARD_MB_CBM_TEENSY2_H
#define _BOARD_MB_CBM_TEENSY2_H

// Initialize the board (timer, indicators, UART)
void board_init(void);
// Initialize the IO ports for IEC mode
void board_init_iec(void);

// Mapping of IEC lines to IO port output signals.
#define IO_DATA         _BV(0) // 
#define IO_CLK          _BV(1) // 
#define IO_ATN          _BV(2) //
#define IO_RESET        _BV(3) // 
#define LED_MASK        _BV(5) // 
#define YLED_MASK       _BV(6) // 
#define IO_SRQ          _BV(7) // 

#define IEC_CLK_O       _BV(7)  // D7
#define IEC_DATA_O      _BV(5)  // D5
#define IEC_SRQ_O       _BV(4)  // F4
#define IEC_RESET_O     _BV(6)  // C6
#define IEC_ATN_O       _BV(6)  // F6
#define LED_O           _BV(6)  // E6
#define YLED_O          _BV(6)  // D6

// Input signals
//#define IO_CLK_IN       _BV(6) // D6
//#define IO_CLK_IN       _BV(1) // D1
//#define IO_DATA_IN      _BV(4) // D4
//#define IO_SRQ_IN       _BV(5) // F5
//#define IO_RESET_IN     _BV(0) // C7
//#define IO_ATN_IN       _BV(7) // F7

//#define IEC_CLK_I       _BV(6)  
#define IEC_CLK_I       _BV(1)  // D1  
#define IEC_DATA_I      _BV(4)  // D4
#define IEC_SRQ_I       _BV(5)  // F5
#define IEC_RESET_I     _BV(7)  // C7 
#define IEC_ATN_I       _BV(7)  // F7

// Masks for setting data direction registers.
#define IEC_MASK_C      (IEC_RESET_O )
#define IEC_MASK_D      (IEC_CLK_O | IEC_DATA_O | YLED_O)
#define IEC_MASK_E      (LED_O)
#define IEC_MASK_F      (IEC_SRQ_O | IEC_ATN_O)

// IEC and parallel port accessors
#define PAR_PORT_PORT   PORTB
#define PAR_PORT_DDR    DDRB
#define PAR_PORT_PIN    PINB

//
// IEEE-488 Pinout  (Port,bit) OR (Input Port,Bit,Output Port,Bit)
//
#define IEEE_SUPPORT    1
#define SRQ_NIB_SUPPORT 1

#define IEEE_EOI_IO     0xf0 // 
#define IEEE_ATN_I      0xf7 // 
#define IEEE_ATN_O      0xf6 // 
#define IEEE_DAV_IO     0xf1 //
#define IEEE_IFC_I      0xc7 //
#define IEEE_IFC_O      0xc6 //
#define IEEE_SRQ_I      0xf5 //
#define IEEE_SRQ_O      0xf4 //
#define IEEE_NDAC_I     0xd4 //
#define IEEE_NDAC_O     0xd5 //
//#define IEEE_NRFD_I     0xd6 //
#define IEEE_NRFD_I     0xd1 //
#define IEEE_NRFD_O     0xd7 //
#define IEEE_REN_IO     0xd0 //
#define IEEE_DATA_IO    0xb0 // Port B - Data
#define IEEE_LED_O      0xe6

// Masks for setting Pull-up
#define PUP_MASK_C      (IEC_RESET_I)
#define PUP_MASK_D      (IEC_DATA_I|IEC_CLK_I|(1<<0))
#define PUP_MASK_F      (IEC_ATN_I|(1<<1)|(1<<0)|IEC_SRQ_I)


/*
 * Use always_inline to override gcc's -Os option. Since we measured each
 * inline function's disassembly and verified the size decrease, we are
 * certain when we specify inline that we really want it.
 */
#define INLINE          static inline __attribute__((always_inline))

/*
 * Routines for getting/setting individual IEC lines and parallel port.
 *
 * We no longer add a short delay after changing line(s) state, even though
 * it takes about 0.5 us for the line to stabilize (measured with scope).
 * This is because we need to toggle SRQ quickly to send data to the 1571
 * and the delay was breaking our deadline.
 *
 * These are all inlines and this was incrementally measured that each
 * decreases the firmware size. Some (set/get) compile into a single
 * instruction (say, sbis). This works because the "line" argument is
 * almost always a constant.
 */

INLINE uint8_t
iec_get(uint8_t line)
{
    uint8_t ret;

    switch (line) 
    {
    case IO_SRQ:
        ret = PINF & IEC_SRQ_I;
        break;
    case IO_CLK:
        ret = PIND & IEC_CLK_I;
        break;
    case IO_DATA:
        ret = PIND & IEC_DATA_I;
        break;
    case IO_ATN:
        ret = PINF & IEC_ATN_I;
        break;
    case IO_RESET:
        ret = PINC & IEC_RESET_I;
        break;
    default:
        // Invalid set of requested signals, trigger WD reset
        for (;;) ;
    }

    return !ret;
}

INLINE void
iec_set(uint8_t line)
{
    if (line & IO_CLK)
    {
        PORTD |= IEC_CLK_O;
    }
    if (line & IO_DATA)
    {
        PORTD |= IEC_DATA_O;
    }
    if (line & IO_ATN)
    {
        PORTF |= IEC_ATN_O;
    }
    if (line & IO_SRQ)
    {
        PORTF |= IEC_SRQ_O;
    }
    if (line & IO_RESET)
    {
        PORTC |= IEC_RESET_O;
    }
    if (line & YLED_MASK)
    {
        PORTD |= YLED_O;
    }
    if (line & LED_MASK)
    {
        PORTE |= LED_O;
    }    
}

INLINE void
iec_release(uint8_t line)
{
    if (line & IO_CLK)
    {
        PORTD &= ~IEC_CLK_O;
    }
    if (line & IO_DATA)
    {
        PORTD &= ~IEC_DATA_O;
    }
    if (line & IO_ATN)
    {
        PORTF &= ~IEC_ATN_O;
    }
    if (line & IO_SRQ)
    {
        PORTF &= ~IEC_SRQ_O;
    }
    if (line & IO_RESET)
    {
        PORTC &= ~IEC_RESET_O;
    }
    if (line & LED_MASK)
    {
        PORTE &= ~LED_O;
    }    
    if (line & YLED_MASK)
    {
        PORTD &= ~YLED_O;
    }    
}

INLINE void
iec_set_release(uint8_t s, uint8_t r)
{
    iec_set(s);
    iec_release(r);
}

// Make 8-bit port all inputs and read parallel value
INLINE uint8_t
iec_pp_read(void)
{
    PAR_PORT_DDR = 0;
    PAR_PORT_PORT = 0;
    return PAR_PORT_PIN;
}

// Make 8-bits of port output and write out the parallel data
INLINE void
iec_pp_write(uint8_t val)
{
    PAR_PORT_DDR = 0xff;
    PAR_PORT_PORT = val;
}

INLINE uint8_t
iec_srq_read(void)
{
    uint8_t i, data;

    data = 0;
    for (i = 8; i != 0; --i) {
        // Wait for the drive to pull IO_SRQ.
        while (!iec_get(IO_SRQ))
            ;

        // Wait for drive to release SRQ, then delay another 375 ns for DATA
        // to stabilize before reading it.
        while (iec_get(IO_SRQ))
            ;
        DELAY_US(0.375);

        // Read data bit
        data = (data << 1) | (iec_get(IO_DATA) ? 0 : 1);
   }

   return data;
}

/*
 * Write out a byte by sending each bit on the DATA line (inverted) and
 * clocking the CIA with SRQ. We don't want clock jitter so the body of
 * the loop must not have any branches. At 500 Kbit/sec, each loop iteration
 * should take 2 us or 32 clocks per bit at 16 MHz.
 */
INLINE void
iec_srq_write(uint8_t data)
{
    uint8_t i;

    for (i = 8; i != 0; --i) {
        /*
         * Take the high bit of the data byte. Shift it down to the IO_DATA
         * pin for the ZF board. Combine it (inverted) with the IO_SRQ line
         * being set. Write both of these to port D at the same time.
         *
         * This is 7 clock cycles with gcc 9.1.0 at both -Os and -O2.
         */
        PORTD = (((data >> 3) & IEC_DATA_O) ^ IEC_DATA_O) | IEC_SRQ_O;
        data <<= 1;          // get next bit: 1 clock
        DELAY_US(0.3);       // (nibtools relies on this timing, do not change)
        iec_release(IO_SRQ); // release SRQ: 2 clocks
        DELAY_US(0.935);     // (nibtools relies on this timing, do not change)

        // Decrement i and loop: 3 clock cycles when branch taken
        // Total: 13 clocks per loop (minus delays); 19 clocks left.
    }
}

// Since this is called with a runtime-specified mask, inlining doesn't help.
uint8_t iec_poll_pins(void);

// Status indicators (LEDs)
uint8_t board_get_status(void);
void board_set_status(uint8_t status);
void board_update_display(void);
bool board_timer_fired(void);

#endif // _BOARD_MB_CBM_TEENSY2_H
