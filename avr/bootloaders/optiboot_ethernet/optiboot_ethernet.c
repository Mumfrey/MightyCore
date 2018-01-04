/**********************************************************/
/* 2011-09-18: Extension to support W5100 by              */
/* William R Sowerbutts <will@sowerbutts.com>             */
/* See http://sowerbutts.com/optiboot-w5100/              */
/**********************************************************/

#define FUNC_READ 1
#define FUNC_WRITE 1
/**********************************************************/
/* Majek's Optiboot bootloader for Arduino                */
/*                                                        */
/* http://github.com/majekw/optiboot                      */
/*                                                        */
/*  It is the intent that changes not relevant to the     */
/*  Arduino production envionment get moved from the      */
/*  optiboot project to the arduino project in "lumps."   */
/*                                                        */
/* Heavily optimised bootloader that is faster and        */
/* smaller than the Arduino standard bootloader           */
/*                                                        */
/* Enhancements:                                          */
/*   Fits in 512 bytes, saving 1.5K of code space         */
/*   Higher baud rate speeds up programming               */
/*   Written almost entirely in C                         */
/*   Customisable timeout with accurate timeconstant      */
/*   Optional virtual UART. No hardware UART required.    */
/*   Optional virtual boot partition for devices without. */
/*   Supports "write to flash" in application!            */
/*                                                        */
/* What you lose:                                         */
/*   Implements a skeleton STK500 protocol which is       */
/*     missing several features including EEPROM          */
/*     programming and non-page-aligned writes            */
/*   High baud rate breaks compatibility with standard    */
/*     Arduino flash settings                             */
/*                                                        */
/*  Supported microcontrollers:                           */
/*   ATmega1284                                           */
/*   ATmega644                                            */
/*   ATmega324                                            */
/*   ATmega164                                            */
/*   ATmega32                                             */
/*   ATmega16                                             */
/*   ATmega8535                                           */
/*                                                        */
/*                                                        */
/* Assumptions:                                           */
/*   The code makes several assumptions that reduce the   */
/*   code size. They are all true after a hardware reset, */
/*   but may not be true if the bootloader is called by   */
/*   other means or on other hardware.                    */
/*     No interrupts can occur                            */
/*     UART and Timer 1 are set to their reset state      */
/*     SP points to RAMEND                                */
/*                                                        */
/* Code builds on code, libraries and optimisations from: */
/*   stk500boot.c          by Jason P. Kyle               */
/*   Arduino bootloader    http://arduino.cc              */
/*   Spiff's 1K bootloader http://spiffie.org/know/arduino_1k_bootloader/bootloader.shtml */
/*   avr-libc project      http://nongnu.org/avr-libc     */
/*   Adaboot               http://www.ladyada.net/library/arduino/bootloader.html */
/*   AVR305                Atmel Application Note         */
/*                                                        */

/* Copyright 2013-2015 by Bill Westfield.                 */
/* Copyright 2010 by Peter Knight.                        */
/*                                                        */
/* This program is free software; you can redistribute it */
/* and/or modify it under the terms of the GNU General    */
/* Public License as published by the Free Software       */
/* Foundation; either version 2 of the License, or        */
/* (at your option) any later version.                    */
/*                                                        */
/* This program is distributed in the hope that it will   */
/* be useful, but WITHOUT ANY WARRANTY; without even the  */
/* implied warranty of MERCHANTABILITY or FITNESS FOR A   */
/* PARTICULAR PURPOSE.  See the GNU General Public        */
/* License for more details.                              */
/*                                                        */
/* You should have received a copy of the GNU General     */
/* Public License along with this program; if not, write  */
/* to the Free Software Foundation, Inc.,                 */
/* 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA */
/*                                                        */
/* Licence can be viewed at                               */
/* http://www.fsf.org/licenses/gpl.txt                    */
/*                                                        */
/**********************************************************/


/**********************************************************/
/*                                                        */
/* Optional defines:                                      */
/*                                                        */
/**********************************************************/
/*                                                        */
/* BIGBOOT:                                               */
/* Build a 1k bootloader, not 512 bytes. This turns on    */
/* extra functionality.                                   */
/*                                                        */
/* BAUD_RATE:                                             */
/* Set bootloader baud rate.                              */
/*                                                        */
/* SOFT_UART:                                             */
/* Use AVR305 soft-UART instead of hardware UART.         */
/*                                                        */
/* LED_START_FLASHES:                                     */
/* Number of LED flashes on bootup.                       */
/*                                                        */
/* LED_DATA_FLASH:                                        */
/* Flash LED when transferring data. For boards without   */
/* TX or RX LEDs, or for people who like blinky lights.   */
/*                                                        */
/* SUPPORT_EEPROM:                                        */
/* Support reading and writing from EEPROM. This is not   */
/* used by Arduino, so off by default.                    */
/*                                                        */
/* TIMEOUT_MS:                                            */
/* Bootloader timeout period, in milliseconds.            */
/* 500,1000,2000,4000,8000 supported.                     */
/*                                                        */
/* UART:                                                  */
/* UART number (0..n) for devices with more than          */
/* one hardware uart (644P, 1284P, etc)                   */
/*                                                        */
/**********************************************************/

/**********************************************************/
/* Version Numbers!                                       */
/*                                                        */
/* Arduino Optiboot now includes this Version number in   */
/* the source and object code.                            */
/*                                                        */
/* Version 3 was released as zip from the optiboot        */
/*  repository and was distributed with Arduino 0022.     */
/* Version 4 starts with the arduino repository commit    */
/*  that brought the arduino repository up-to-date with   */
/*  the optiboot source tree changes since v3.            */
/* Version 5 was created at the time of the new Makefile  */
/*  structure (Mar, 2013), even though no binaries changed*/
/* It would be good if versions implemented outside the   */
/*  official repository used an out-of-seqeunce version   */
/*  number (like 104.6 if based on based on 4.5) to       */
/*  prevent collisions.                                   */
/*                                                        */
/**********************************************************/

/**********************************************************/
/* Edit History:                                          */
/*                                                        */
/* Aug 2014                                               */
/* 6.2 WestfW: make size of length variables dependent    */
/*              on the SPM_PAGESIZE.  This saves space    */
/*              on the chips where it's most important.   */
/* 6.1 WestfW: Fix OPTIBOOT_CUSTOMVER (send it!)          */
/*             Make no-wait mod less picky about          */
/*               skipping the bootloader.                 */
/*             Remove some dead code                      */
/* Jun 2014                                               */
/* 6.0 WestfW: Modularize memory read/write functions     */
/*             Remove serial/flash overlap                */
/*              (and all references to NRWWSTART/etc)     */
/*             Correctly handle pagesize > 255bytes       */
/*             Add EEPROM support in BIGBOOT (1284)       */
/*             EEPROM write on small chips now causes err */
/*             Split Makefile into smaller pieces         */
/*             Add Wicked devices Wildfire                */
/*         Move UART=n conditionals into pin_defs.h       */
/*         Remove LUDICOUS_SPEED option                   */
/*         Replace inline assembler for .version          */
/*              and add OPTIBOOT_CUSTOMVER for user code  */
/*             Fix LED value for Bobuino (Makefile)       */
/*             Make all functions explicitly inline or    */
/*              noinline, so we fit when using gcc4.8     */
/*             Change optimization options for gcc4.8     */
/*             Make ENV=arduino work in 1.5.x trees.      */
/* May 2014                                               */
/* 5.0 WestfW: Add support for 1Mbps UART                 */
/* Mar 2013                                               */
/* 5.0 WestfW: Major Makefile restructuring.              */
/*             See Makefile and pin_defs.h                */
/*             (no binary changes)                        */
/*                                                        */
/* 4.6 WestfW/Pito: Add ATmega32 support                  */
/* 4.6 WestfW/radoni: Don't set LED_PIN as an output if   */
/*                    not used. (LED_START_FLASHES = 0)   */
/* Jan 2013                                               */
/* 4.6 WestfW/dkinzer: use autoincrement lpm for read     */
/* 4.6 WestfW/dkinzer: pass reset cause to app in R2      */
/* Mar 2012                                               */
/* 4.5 WestfW: add infrastructure for non-zero UARTS.     */
/* 4.5 WestfW: fix SIGNATURE_2 for m644 (bad in avr-libc) */
/* Jan 2012:                                              */
/* 4.5 WestfW: fix NRWW value for m1284.                  */
/* 4.4 WestfW: use attribute OS_main instead of naked for */
/*             main().  This allows optimizations that we */
/*             count on, which are prohibited in naked    */
/*             functions due to PR42240.  (keeps us less  */
/*             than 512 bytes when compiler is gcc4.5     */
/*             (code from 4.3.2 remains the same.)        */
/* 4.4 WestfW and Maniacbug:  Add m1284 support.  This    */
/*             does not change the 328 binary, so the     */
/*             version number didn't change either. (?)   */
/* June 2011:                                             */
/* 4.4 WestfW: remove automatic soft_uart detect (didn't  */
/*             know what it was doing or why.)  Added a   */
/*             check of the calculated BRG value instead. */
/*             Version stays 4.4; existing binaries are   */
/*             not changed.                               */
/* 4.4 WestfW: add initialization of address to keep      */
/*             the compiler happy.  Change SC'ed targets. */
/*             Return the SW version via READ PARAM       */
/* 4.3 WestfW: catch framing errors in getch(), so that   */
/*             AVRISP works without HW kludges.           */
/*  http://code.google.com/p/arduino/issues/detail?id=368n*/
/* 4.2 WestfW: reduce code size, fix timeouts, change     */
/*             verifySpace to use WDT instead of appstart */
/* 4.1 WestfW: put version number in binary.              */
/**********************************************************/

#define OPTIBOOT_MAJVER 6
#define OPTIBOOT_MINVER 2

/*
 * OPTIBOOT_CUSTOMVER should be defined (by the makefile) for custom edits
 * of optiboot.  That way you don't wind up with very different code that
 * matches the version number of a "released" optiboot.
 */

#if !defined(OPTIBOOT_CUSTOMVER)
#define OPTIBOOT_CUSTOMVER 0
#endif

unsigned const int __attribute__((section(".version"))) 
optiboot_version = 256*(OPTIBOOT_MAJVER + OPTIBOOT_CUSTOMVER) + OPTIBOOT_MINVER;


#include <inttypes.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <util/delay.h>

/*
 * Note that we use our own version of "boot.h"
 * <avr/boot.h> uses sts instructions, but this version uses out instructions
 * This saves cycles and program memory.  Sorry for the name overlap.
 */
#include "boot.h"


// We don't use <avr/wdt.h> as those routines have interrupt overhead we don't need.

/*
 * pin_defs.h
 * This contains most of the rather ugly defines that implement our
 * ability to use UART=n and LED=D3, and some avr family bit name differences.
 */
#include "pin_defs.h"

/*
 * stk500.h contains the constant definitions for the stk500v1 comm protocol
 */
#include "stk500.h"

/*
 * EEPROM layout:
 * ADDRESS _0 _1 _2 _3 _4 _5 _6 _7 _8 _9 _A _B _C _D _E _F
 * 0x0000  -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --  / -- = unused (0xFF)
 * 0x0010  GG GG GG GG SS SS SS SS MM MM MM MM MM MM II II  / MM = MAC address; QQ = bootloader flag
 * 0x0020  II II QQ -- -- -- -- -- -- -- -- -- -- -- -- --  / II = IPv4 address; SS = subnet mask; GG = gateway
 *
 * OFFSET LENGTH DESCRIPTION
 *  0x10  4      IPv4 gateway address
 *  0x14  4      IPv4 subnet mask
 *  0x18  6      Ethernet MAC address
 *  0x1E  4      IPv4 address
 *  0x22  1      Bootloader flag
 *
 *  Example:
 *
 *  ADDRESS _0 _1 _2 _3 _4 _5 _6 _7 _8 _9 _A _B _C _D _E _F
 *  0x0010  c0 a8 64 01 ff ff ff 00 00 16 36 de 58 f6 c0 a8 
 *  0x0020  64 e9 55 
 *
 *  This configures IP address 192.168.100.233, subnet mask 255.255.255.0,
 *  gateway 192.168.100.1, and MAC address 00:16:36:de:58:f6. Note that the
 *  bootloader flag is set in this example.
 *
 */

#define EEPROM_GATEWAY_OFFSET                 0x10
#define EEPROM_SNMASK_OFFSET                  0x14
#define EEPROM_MAC_OFFSET                     0x18
#define EEPROM_IP_ADDR_OFFSET                 0x1E
#define EEPROM_BOOTLOADER_FLAG_OFFSET         0x22
#define EEPROM_BOOTLOADER_MAGIC_VALUE         0x55
#define EEPROM_BOOTLOADER_BORING_VALUE        0xFF

// W5100 registers
#define  W5100_MR               0x0000      /* Mode Register */
#define  W5100_GAR              0x0001      /* Gateway Address: 0x0001 to 0x0004 */
#define  W5100_SUBR             0x0005      /* Subnet mask Address: 0x0005 to 0x0008 */
#define  W5100_SHAR             0x0009      /* Source Hardware Address (MAC): 0x0009 to 0x000E */
#define  W5100_SIPR             0x000F      /* Source IP Address: 0x000F to 0x0012 */
#define  W5100_IMR              0x0016      /* Interrupt Mask Register */
#define  W5100_SKT_REG_BASE     0x0400      /* start of socket registers */
#define  W5100_SKT_OFFSET       0x0100      /* offset to each socket regester set */
#define  W5100_SKT_BASE(n)      (W5100_SKT_REG_BASE+(n*W5100_SKT_OFFSET))

// socket register offsets
#define  W5100_MR_OFFSET        0x0000      /* socket Mode Register offset */
#define  W5100_CR_OFFSET        0x0001      /* socket Command Register offset */
#define  W5100_IR_OFFSET        0x0002      /* socket Interrupt Register offset */
#define  W5100_SR_OFFSET        0x0003      /* socket Status Register offset */
#define  W5100_PORT_OFFSET      0x0004      /* socket Port Register offset (2 bytes) */
#define  W5100_DHAR_OFFSET      0x0006      /* socket Destination Hardware Address Register (MAC, 6 bytes) */
#define  W5100_DIPR_OFFSET      0x000C      /* socket Destination IP Address Register (IP, 4 bytes) */
#define  W5100_DPORT_OFFSET     0x0010      /* socket Destination Port Register (2 bytes) */
#define  W5100_MSS_OFFSET       0x0012      /* socket Maximum Segment Size (2 bytes) */
#define  W5100_PROTO_OFFSET     0x0014      /* socket IP Protocol Register */
#define  W5100_TOS_OFFSET       0x0015      /* socket Type Of Service Register */
#define  W5100_TTL_OFFSET       0x0016      /* socket Time To Live Register */
#define  W5100_TX_FSR_OFFSET    0x0020      /* socket Transmit Free Size Register (2 bytes) */
#define  W5100_TX_RR_OFFSET     0x0022      /* socket Transmit Read Pointer Register (2 bytes) */
#define  W5100_TX_WR_OFFSET     0x0024      /* socket Transmit Write Pointer Register (2 bytes) */
#define  W5100_RX_RSR_OFFSET    0x0026      /* socket Receive Received Size Register (2 bytes) */
#define  W5100_RX_RD_OFFSET     0x0028      /* socket Receive Read Pointer Register (2 bytes) */

// Device Mode Register
#define  W5100_MR_SOFTRST       (1<<7)      /* soft-reset */
#define  W5100_MR_PINGBLK       (1<<4)      /* block responses to ping request */
#define  W5100_MR_PPPOE         (1<<3)      /* enable PPPoE */
#define  W5100_MR_AUTOINC       (1<<1)      /* address autoincrement (indirect interface ONLY!) */
#define  W5100_MR_INDINT        (1<<0)      /* use indirect interface (parallel interface ONLY!) */

// Socket mode register
#define  W5100_SKT_MR_CLOSE     0x00        /* Unused socket */
#define  W5100_SKT_MR_TCP       0x01        /* TCP */
#define  W5100_SKT_MR_UDP       0x02        /* UDP */
#define  W5100_SKT_MR_IPRAW     0x03        /* IP LAYER RAW SOCK */
#define  W5100_SKT_MR_MACRAW    0x04        /* MAC LAYER RAW SOCK */
#define  W5100_SKT_MR_PPPOE     0x05        /* PPPoE */
#define  W5100_SKT_MR_ND        0x20        /* No Delayed Ack(TCP) flag */
#define  W5100_SKT_MR_MULTI     0x80        /* support multicasting */

// Socket command register
#define  W5100_SKT_CR_OPEN      0x01        /* open the socket */
#define  W5100_SKT_CR_LISTEN    0x02        /* wait for TCP connection (server mode) */
#define  W5100_SKT_CR_CONNECT   0x04        /* listen for TCP connection (client mode) */
#define  W5100_SKT_CR_DISCON    0x08        /* close TCP connection */
#define  W5100_SKT_CR_CLOSE     0x10        /* mark socket as closed (does not close TCP connection) */
#define  W5100_SKT_CR_SEND      0x20        /* transmit data in TX buffer */
#define  W5100_SKT_CR_SEND_MAC  0x21        /* SEND, but uses destination MAC address (UDP only) */
#define  W5100_SKT_CR_SEND_KEEP 0x22        /* SEND, but sends 1-byte packet for keep-alive (TCP only) */
#define  W5100_SKT_CR_RECV      0x40        /* receive data into RX buffer */

// Socket status register
#define  W5100_SKT_SR_CLOSED      0x00      /* closed */
#define  W5100_SKT_SR_INIT        0x13      /* init state */
#define  W5100_SKT_SR_LISTEN      0x14      /* listen state */
#define  W5100_SKT_SR_SYNSENT     0x15      /* connection state */
#define  W5100_SKT_SR_SYNRECV     0x16      /* connection state */
#define  W5100_SKT_SR_ESTABLISHED 0x17      /* success to connect */
#define  W5100_SKT_SR_FIN_WAIT    0x18      /* closing state */
#define  W5100_SKT_SR_CLOSING     0x1A      /* closing state */
#define  W5100_SKT_SR_TIME_WAIT   0x1B      /* closing state */
#define  W5100_SKT_SR_CLOSE_WAIT  0x1C      /* closing state */
#define  W5100_SKT_SR_LAST_ACK    0x1D      /* closing state */
#define  W5100_SKT_SR_UDP         0x22      /* UDP socket */
#define  W5100_SKT_SR_IPRAW       0x32      /* IP raw mode socket */
#define  W5100_SKT_SR_MACRAW      0x42      /* MAC raw mode socket */
#define  W5100_SKT_SR_PPPOE       0x5F      /* PPPOE socket */

// TX and RX buffers
#define  W5100_TXBUFADDR        0x4000      /* W5100 Send Buffer Base Address */
#define  W5100_RXBUFADDR        0x6000      /* W5100 Read Buffer Base Address */
#define  W5100_BUFSIZE          0x0800      /* W5100 buffers are sized 2K */
#define  W5100_TX_BUF_MASK      0x07FF      /* Tx 2K Buffer Mask */
#define  W5100_RX_BUF_MASK      0x07FF      /* Rx 2K Buffer Mask */

#define W5100_WRITE_OPCODE          0xF0
#define W5100_READ_OPCODE           0x0F

#ifndef LED_START_FLASHES
#define LED_START_FLASHES 0
#endif

/* set the UART baud rate defaults */
#ifndef BAUD_RATE
#if F_CPU >= 8000000L
#define BAUD_RATE   115200L // Highest rate Avrdude win32 will support
#elsif F_CPU >= 1000000L
#define BAUD_RATE   9600L   // 19200 also supported, but with significant error
#elsif F_CPU >= 128000L
#define BAUD_RATE   4800L   // Good for 128kHz internal RC
#else
#define BAUD_RATE 1200L     // Good even at 32768Hz
#endif
#endif

#ifndef UART
#define UART 0
#endif

#define BAUD_SETTING (( (F_CPU + BAUD_RATE * 4L) / ((BAUD_RATE * 8L))) - 1 )
#define BAUD_ACTUAL (F_CPU/(8 * ((BAUD_SETTING)+1)))
#define BAUD_ERROR (( 100*(BAUD_RATE - BAUD_ACTUAL) ) / BAUD_RATE)

#if BAUD_ERROR >= 5
#error BAUD_RATE error greater than 5%
#elif BAUD_ERROR <= -5
#error BAUD_RATE error greater than -5%
#elif BAUD_ERROR >= 2
#warning BAUD_RATE error greater than 2%
#elif BAUD_ERROR <= -2
#warning BAUD_RATE error greater than -2%
#endif

#if (F_CPU + BAUD_RATE * 4L) / (BAUD_RATE * 8L) - 1 > 250
#error Unachievable baud rate (too slow) BAUD_RATE 
#endif // baud rate slow check
#if (F_CPU + BAUD_RATE * 4L) / (BAUD_RATE * 8L) - 1 < 3
#if BAUD_ERROR != 0 // permit high bitrates (ie 1Mbps@16MHz) if error is zero
#error Unachievable baud rate (too fast) BAUD_RATE 
#endif
#endif // baud rate fastn check

/* Watchdog settings */
#define WATCHDOG_OFF    (0)
#define WATCHDOG_16MS   (_BV(WDE))
#define WATCHDOG_32MS   (_BV(WDP0) | _BV(WDE))
#define WATCHDOG_64MS   (_BV(WDP1) | _BV(WDE))
#define WATCHDOG_125MS  (_BV(WDP1) | _BV(WDP0) | _BV(WDE))
#define WATCHDOG_250MS  (_BV(WDP2) | _BV(WDE))
#define WATCHDOG_500MS  (_BV(WDP2) | _BV(WDP0) | _BV(WDE))
#define WATCHDOG_1S     (_BV(WDP2) | _BV(WDP1) | _BV(WDE))
#define WATCHDOG_2S     (_BV(WDP2) | _BV(WDP1) | _BV(WDP0) | _BV(WDE))
#ifndef __AVR_ATmega8__
#define WATCHDOG_4S     (_BV(WDP3) | _BV(WDE))
#define WATCHDOG_8S     (_BV(WDP3) | _BV(WDP0) | _BV(WDE))
#endif


/*
 * We can never load flash with more than 1 page at a time, so we can save
 * some code space on parts with smaller pagesize by using a smaller int.
 */
#if SPM_PAGESIZE > 255
typedef uint16_t pagelen_t ;
#define GETLENGTH(len) len = getch()<<8; len |= getch()
#else
typedef uint8_t pagelen_t;
#define GETLENGTH(len) (void) getch() /* skip high byte */; len = getch()
#endif


/* Function Prototypes
 * The main() function is in init9, which removes the interrupt vector table
 * we don't need. It is also 'OS_main', which means the compiler does not
 * generate any entry or exit code itself (but unlike 'naked', it doesn't
 * supress some compile-time options we want.)
 */

void pre_main(void) __attribute__ ((naked)) __attribute__ ((section (".init8")));
int main(void) __attribute__ ((OS_main)) __attribute__ ((section (".init9")));

void ethernet_init(void);

void __attribute__((noinline)) putch(char);
#ifdef UART1_DEBUG
void __attribute__((noinline)) debug(char);
#endif
uint8_t __attribute__((noinline)) getch(void);
void __attribute__((noinline)) verifySpace();
void __attribute__((noinline)) watchdogConfig(uint8_t x);

static inline void getNch(uint8_t);
static inline void flash_led(uint8_t);
static inline void watchdogReset();
static inline void writebuffer(int8_t memtype, uint8_t *mybuff,
                   uint16_t address, pagelen_t len);
static inline void read_mem(uint8_t memtype,
                   uint16_t address, pagelen_t len);
static void __attribute__((noinline)) do_spm(uint16_t address, uint8_t command, uint16_t data);

#ifdef SOFT_UART
void uartDelay() __attribute__ ((naked));
#endif
void appStart(uint8_t rstFlags) __attribute__ ((naked));

/*
 * RAMSTART should be self-explanatory.  It's bigger on parts with a
 * lot of peripheral registers.  Let 0x100 be the default
 * Note that RAMSTART (for optiboot) need not be exactly at the start of RAM.
 */
#if !defined(RAMSTART)  // newer versions of gcc avr-libc define RAMSTART
#define RAMSTART 0x100
#if defined (__AVR_ATmega644P__)
// correct for a bug in avr-libc
#undef SIGNATURE_2
#define SIGNATURE_2 0x0A
#elif defined(__AVR_ATmega1280__)
#undef RAMSTART
#define RAMSTART (0x200)
#endif
#endif


// Make sure all variants gets the correct device signature
#if defined(__AVR_ATmega1284__)
  #undef SIGNATURE_2
  #define SIGNATURE_2 0x06
#elif defined(__AVR_ATmega1284P__)
  #undef SIGNATURE_2
  #define SIGNATURE_2 0x05
#elif defined(__AVR_ATmega644__)
  #undef SIGNATURE_2
  #define SIGNATURE_2 0x09
#elif defined(__AVR_ATmega644P__)
  #undef SIGNATURE_2
  #define SIGNATURE_2 0x0A
#elif defined(__AVR_ATmega324A__)
  #undef SIGNATURE_2
  #define SIGNATURE_2 0x15
#elif defined(__AVR_ATmega324P__)
  #undef SIGNATURE_2
  #define SIGNATURE_2 0x08
  #elif defined(__AVR_ATmega324PA__)
  #undef SIGNATURE_2
  #define SIGNATURE_2 0x11
#elif defined(__AVR_ATmega164A__)
  #undef SIGNATURE_2
  #define SIGNATURE_2 0x0F
#elif defined(__AVR_ATmega164P__)
  #undef SIGNATURE_2
  #define SIGNATURE_2 0x0A
#endif


/* C zero initialises all global variables. However, that requires */
/* These definitions are NOT zero initialised, but that doesn't matter */
/* This allows us to drop the zero init code, saving us memory */
#define buff    ((uint8_t*)(RAMSTART))

/* Virtual boot partition support */
#ifdef VIRTUAL_BOOT_PARTITION
#define rstVect0_sav (*(uint8_t*)(RAMSTART+SPM_PAGESIZE*2+4))
#define rstVect1_sav (*(uint8_t*)(RAMSTART+SPM_PAGESIZE*2+5))
#define wdtVect0_sav (*(uint8_t*)(RAMSTART+SPM_PAGESIZE*2+6))
#define wdtVect1_sav (*(uint8_t*)(RAMSTART+SPM_PAGESIZE*2+7))
// Vector to save original reset jump:
//   SPM Ready is least probably used, so it's default
//   if not, use old way WDT_vect_num,
//   or simply set custom save_vect_num in Makefile using vector name
//   or even raw number.
#if !defined (save_vect_num)
#if defined (SPM_RDY_vect_num)
#define save_vect_num (SPM_RDY_vect_num)
#elif defined (SPM_READY_vect_num)
#define save_vect_num (SPM_READY_vect_num)
#elif defined (WDT_vect_num)
#define save_vect_num (WDT_vect_num)
#else
#error Cant find SPM or WDT interrupt vector for this CPU
#endif
#endif //save_vect_num
// check if it's on the same page (code assumes that)
#if (SPM_PAGESIZE <= save_vect_num)
#error Save vector not in the same page as reset!
#endif
#if FLASHEND > 8192
// AVRs with more than 8k of flash have 4-byte vectors, and use jmp.
//  We save only 16 bits of address, so devices with more than 128KB
//  may behave wrong for upper part of address space.
#define rstVect0 2
#define rstVect1 3
#define wdtVect0 (WDT_vect_num*4+2)
#define wdtVect1 (WDT_vect_num*4+3)
#define appstart_vec (WDT_vect_num*2)
#else
// AVRs with up to 8k of flash have 2-byte vectors, and use rjmp.
#define rstVect0 0
#define rstVect1 1
#define wdtVect0 (WDT_vect_num*2)
#define wdtVect1 (WDT_vect_num*2+1)
#define appstart_vec (WDT_vect_num)
#endif
#else
#define appstart_vec (0)
#endif // VIRTUAL_BOOT_PARTITION

#define ethernet_mode  (*(uint8_t*)(RAMSTART+SPM_PAGESIZE*2+8)) // mode flag. non-zero for ethernet mode, zero for serial mode.
#define sync_sink  (*(uint8_t*)(RAMSTART+SPM_PAGESIZE*2+9)) // sync sink

/* everything that needs to run VERY early */
void pre_main(void) {
  // Allow convenient way of calling do_spm function - jump table,
  //   so entry to this function will always be here, indepedent of compilation,
  //   features etc
  asm volatile (
    "   rjmp    1f\n"
    "   rjmp    do_spm\n"
    "1:\n"
  );
}


/* main program starts here */
int main(void) {
  uint8_t ch;

  /*
   * Making these local and in registers prevents the need for initializing
   * them, and also saves space because code no longer stores to memory.
   * (initializing address keeps the compiler happy, but isn't really
   *  necessary, and uses 4 bytes of flash.)
   */
  register uint16_t address = 0;
  register pagelen_t  length;

  // After the zero init loop, this is the first code to run.
  //
  // This code makes the following assumptions:
  //  No interrupts will execute
  //  SP points to RAMEND
  //  r1 contains zero
  //
  // If not, uncomment the following instructions:
  // cli();
  asm volatile ("clr __zero_reg__");
#if defined(__AVR_ATmega8__) || defined(__AVR_ATmega8535__) || defined (__AVR_ATmega16__) || defined (__AVR_ATmega32__)
  SP=RAMEND;  // This is done by hardware reset
#endif

  /*
   * modified Adaboot no-wait mod.
   * Pass the reset reason to app.  Also, it appears that an Uno poweron
   * can leave multiple reset flags set; we only want the bootloader to
   * run on an 'external reset only' status
   */

#if defined(__AVR_ATmega8535__) || defined(__AVR_ATmega16__)
   ch = MCUCSR;
   MCUCSR = 0;
#else
  ch = MCUSR;
  MCUSR = 0;
#endif

#ifdef UART1_DEBUG
  UART1_SRA = _BV(U2X0); //Double speed mode USART0
  UART1_SRB = _BV(RXEN0) | _BV(TXEN0);
  UART1_SRC = _BV(UCSZ00) | _BV(UCSZ01);
  UART1_SRL = (uint8_t)( (F_CPU + BAUD_RATE * 4L) / (BAUD_RATE * 8L) - 1 );

  debug(10);debug(13);
#endif

  // Check whether we should enter ethernet mode
  if (eeprom_read_byte((uint8_t*)EEPROM_BOOTLOADER_FLAG_OFFSET) == EEPROM_BOOTLOADER_MAGIC_VALUE) {
    eeprom_write_byte((uint8_t*)EEPROM_BOOTLOADER_FLAG_OFFSET, EEPROM_BOOTLOADER_BORING_VALUE); // unset flag so we boot normally next time around
#ifdef UART1_DEBUG
    debug('N');debug('E');debug('T');
#endif
    
    watchdogConfig(WATCHDOG_8S);
    ethernet_init();

    ethernet_mode = 1;
    sync_sink = 0;
    ch = 0;
  } else {
#ifdef UART1_DEBUG
    debug('S');debug('T');debug('D');
#endif
    ethernet_mode = 0;
  }
 
  if (ch & (_BV(WDRF) | _BV(BORF) | _BV(PORF)))
  {
#ifdef UART1_DEBUG
    debug('>');debug('>');
#endif
    appStart(ch);
  }

#if LED_START_FLASHES > 0
  // Set up Timer 1 for timeout counter
  TCCR1B = _BV(CS12) | _BV(CS10); // div 1024
#endif

#ifndef SOFT_UART
#if defined(__AVR_ATmega8__) || defined(__AVR_ATmega8535__) || defined (__AVR_ATmega16__) || defined (__AVR_ATmega32__)
  UCSRA = _BV(U2X); //Double speed mode USART
  UCSRB = _BV(RXEN) | _BV(TXEN);  // enable Rx & Tx
  UCSRC = _BV(URSEL) | _BV(UCSZ1) | _BV(UCSZ0);  // config USART; 8N1
  UBRRL = (uint8_t)( (F_CPU + BAUD_RATE * 4L) / (BAUD_RATE * 8L) - 1 );
#else
  UART_SRA = _BV(U2X0); //Double speed mode USART0
  UART_SRB = _BV(RXEN0) | _BV(TXEN0);
  UART_SRC = _BV(UCSZ00) | _BV(UCSZ01);
  UART_SRL = (uint8_t)( (F_CPU + BAUD_RATE * 4L) / (BAUD_RATE * 8L) - 1 );
#endif
#endif

  // Set up watchdog to trigger after 1s
  watchdogConfig(WATCHDOG_1S);

#if (LED_START_FLASHES > 0) || defined(LED_DATA_FLASH)
  /* Set LED pin as output */
  LED_DDR |= _BV(LED);
#endif

#ifdef SOFT_UART
  /* Set TX pin as output */
  UART_DDR |= _BV(UART_TX_BIT);
#endif

#if LED_START_FLASHES > 0
  /* Flash onboard LED to signal entering of bootloader */
  flash_led(LED_START_FLASHES * 2);
#endif

  /* Forever loop: exits by causing WDT reset */
  for (;;) {
    /* get character from UART */
    ch = getch();

    if(ch == STK_GET_PARAMETER) {
#ifdef UART1_DEBUG
      debug('G');
#endif
      unsigned char which = getch();
      verifySpace();
      /*
       * Send optiboot version as "SW version"
       * Note that the references to memory are optimized away.
       */
      if (which == 0x82) {
        putch(optiboot_version & 0xFF);
      } else if (which == 0x81) {
        putch(optiboot_version >> 8);
      } else {
        /*
         * GET PARAMETER returns a generic 0x03 reply for
         * other parameters - enough to keep Avrdude happy
         */
        putch(0x03);
      }
    }
    else if(ch == STK_SET_DEVICE) {
#ifdef UART1_DEBUG
      debug('D');debug(' ');
#endif
      // SET DEVICE is ignored
      getNch(20);
    }
    else if(ch == STK_SET_DEVICE_EXT) {
#ifdef UART1_DEBUG
      debug('X');debug(' ');
#endif
      // SET DEVICE EXT is ignored
      getNch(5);
    }
    else if(ch == STK_LOAD_ADDRESS) {
#ifdef UART1_DEBUG
      debug('L');
#endif
      // LOAD ADDRESS
      uint16_t newAddress;
      newAddress = getch();
      newAddress = (newAddress & 0xff) | (getch() << 8);
#ifdef RAMPZ
      // Transfer top bit to RAMPZ
      RAMPZ = (newAddress & 0x8000) ? 1 : 0;
#endif
      newAddress += newAddress; // Convert from word address to byte address
      address = newAddress;
      verifySpace();
    }
    else if(ch == STK_UNIVERSAL) {
#ifdef UART1_DEBUG
      debug('U');debug(' ');
#endif
      // UNIVERSAL command is ignored
      getNch(4);
      putch(0x00);
    }
    /* Write memory, length is big endian and is in bytes */
    else if(ch == STK_PROG_PAGE) {
#ifdef UART1_DEBUG
      debug('#');
#endif
      // PROGRAM PAGE - we support flash programming only, not EEPROM
      uint8_t desttype;
      uint8_t *bufPtr;
      pagelen_t savelength;

      GETLENGTH(length);
      savelength = length;
      desttype = getch();

#ifdef UART1_DEBUG
      debug(desttype);
#endif

      // read a page worth of contents
      bufPtr = buff;
      do *bufPtr++ = getch();
      while (--length);

      // Read command terminator, start reply
      verifySpace();

#ifdef VIRTUAL_BOOT_PARTITION
#if FLASHEND > 8192
/*
 * AVR with 4-byte ISR Vectors and "jmp"
 */
      if (address == 0) {
        // This is the reset vector page. We need to live-patch the
        // code so the bootloader runs first.
        //
        // Save jmp targets (for "Verify")
        rstVect0_sav = buff[rstVect0];
        rstVect1_sav = buff[rstVect1];
        wdtVect0_sav = buff[wdtVect0];
        wdtVect1_sav = buff[wdtVect1];

        // Move RESET jmp target to WDT vector
        buff[wdtVect0] = rstVect0_sav;
        buff[wdtVect1] = rstVect1_sav;

        // Add jump to bootloader at RESET vector
        buff[rstVect0] = ((uint16_t)main) & 0xFF;
        buff[rstVect1] = ((uint16_t)main) >> 8;
      }

#else
/*
 * AVR with 2-byte ISR Vectors and rjmp
 */
      if ((uint16_t)(void*)address == rstVect0) {
        // This is the reset vector page. We need to live-patch
        // the code so the bootloader runs first.
        //
        // Move RESET vector to WDT vector
        // Save jmp targets (for "Verify")
        rstVect0_sav = buff[rstVect0];
        rstVect1_sav = buff[rstVect1];
        wdtVect0_sav = buff[wdtVect0];
        wdtVect1_sav = buff[wdtVect1];

        // Instruction is a relative jump (rjmp), so recalculate.
        uint16_t vect=rstVect0_sav+(rstVect1_sav<<8);
        vect -= WDT_vect_num;
        // Move RESET jmp target to WDT vector
        buff[wdtVect0] = vect & 0xff;
        buff[wdtVect1] = vect >> 8;
        // Add rjump to bootloader at RESET vector
        buff[0] = (((uint16_t)main) & 0xFFF) & 0xFF; // rjmp 0x1d00 instruction
        buff[1] = ((((uint16_t)main) & 0xFFF) >> 8) | 0xC0;
      }
#endif // FLASHEND
#endif // VBP

      writebuffer(desttype, buff, address, savelength);

#ifdef UART1_DEBUG
      debug('!');debug(10);debug(13);
#endif

    }
    /* Read memory block mode, length is big endian.  */
    else if(ch == STK_READ_PAGE) {
#ifdef UART1_DEBUG
      debug('<');
#endif
      uint8_t desttype;
      GETLENGTH(length);

      desttype = getch();

      verifySpace();

      read_mem(desttype, address, length);
    }

    /* Get device signature bytes  */
    else if(ch == STK_READ_SIGN) {
#ifdef UART1_DEBUG
      debug('R');debug('s');debug(10);debug(13);
#endif
      // READ SIGN - return what Avrdude wants to hear
      verifySpace();
      putch(SIGNATURE_0);
      putch(SIGNATURE_1);
      putch(SIGNATURE_2);
    }
    else if (ch == STK_LEAVE_PROGMODE) { /* 'Q' */
#ifdef UART1_DEBUG
      debug('Q');debug('t');debug(10);debug(13);
#endif
      // Adaboot no-wait mod
      watchdogConfig(WATCHDOG_16MS);
      verifySpace();
    } 
    else if (ch == STK_GET_SYNC && ethernet_mode) {
#ifdef UART1_DEBUG
      debug('$');
#endif
      // avrdude sends 3 sync requests but will begin processing on
      // the first reply it receives, make sure we only reply with
      // one STK_OK+STK_INSYNC ack otherwise avrdude will treat the
      // other replies as the start of the normal reply payload and
      // immediately fail!
      if (sync_sink++ == 2) {
#ifdef UART1_DEBUG
        debug('~');debug(10);debug(13);
#endif
        verifySpace();
        putch(STK_OK);
        continue;
      }
      getch();
      continue;
    }
    else {
      // This covers the response to commands like STK_ENTER_PROGMODE
#ifdef UART1_DEBUG
      debug('?');debug(ch);debug(10);debug(13);
#endif
      verifySpace();
    }
    putch(STK_OK);
    
    // For all other requests rather than STK_GET_SYNC, reset sync req state
    sync_sink = 0;
  }
}

uint8_t W51_xfer(uint16_t addr, uint8_t data, uint8_t opcode)
{
  PORTB &= ~(_BV(PINB4));        // Make SPI SS low
  SPDR = opcode;
  while (!(SPSR & _BV(SPIF))); // wait for SPI
  SPDR = addr >> 8;
  while (!(SPSR & _BV(SPIF))); // wait for SPI
  SPDR = addr & 0xff;
  while (!(SPSR & _BV(SPIF))); // wait for SPI
  SPDR = data;
  while (!(SPSR & _BV(SPIF))); // wait for SPI
  PORTB |= (_BV(PINB4));         // Make SPI SS high
  return SPDR;
}

void W51_write(uint16_t addr, uint8_t data)
{
  W51_xfer(addr, data, W5100_WRITE_OPCODE);
}

uint8_t W51_read(uint16_t addr)
{
  return W51_xfer(addr, 0, W5100_READ_OPCODE);
}

void W51_write16(uint16_t addr, uint16_t data)
{
  W51_write(addr, data >> 8);     // write MSB
  W51_write(addr+1, data & 0xFF); // write LSB
}

uint16_t W51_read16(uint16_t addr)
{
  uint16_t val;

  val = W51_read(addr) << 8;  // read MSB (must be read first)
  val |= W51_read(addr+1);    // read LSB

  return val;
}

void W51_execute(uint16_t cmd)
{
  W51_write(W5100_SKT_BASE(0) + W5100_CR_OFFSET, cmd);
  while(W51_read(W5100_SKT_BASE(0) + W5100_CR_OFFSET));
}

void ethernet_init(void)
{
  // prepare SPI
  PORTB |= _BV(PORTB4);                         // make sure SS is high
  DDRB = _BV(PORTB4) | _BV(PORTB5) | _BV(PORTB7);   // set MOSI, SCK and SS as output, others as input
  SPCR = _BV(SPE) | _BV(MSTR);                    // enable SPI, master mode 0
  // omitting this saves 6 bytes, and it's not like we're a high performance system...
  SPSR |= _BV(SPI2X);                           // enable SPI double speed clock

  // wait for ethernet chip to initialise
  _delay_ms(10); // datasheet says max 10ms but I have seen figures up to 300ms elsewhere.

  W51_write(W5100_MR, W5100_MR_SOFTRST);      // force the w5100 to soft-reset
  _delay_ms(10);                               // wait for chip to reset

  // use 2K buffers for RX/TX for each socket.
  // W51_write(W5100_RMSR, 0x55);  -- pointless; W5100_MR_SOFTRST does this
  // W51_write(W5100_TMSR, 0x55);  -- pointless; W5100_MR_SOFTRST does this
  
  // I laid out the config in EEPROM to match the registers in the W5100.
  // doing it this way saves a bunch of instructions!
  uint8_t r, w;

  r = EEPROM_GATEWAY_OFFSET;
  w = W5100_GAR;
  do{
    W51_write(w, eeprom_read_byte((uint8_t*)((uint16_t)r)));
    w++;
    r++;
  }while(r < EEPROM_BOOTLOADER_FLAG_OFFSET);

  // put socket 0 into listen mode
  W51_write(W5100_SKT_BASE(0) + W5100_MR_OFFSET, W5100_SKT_MR_TCP);
  W51_write(W5100_SKT_BASE(0) + W5100_PORT_OFFSET + 0, 0x11); // port 0x11d0 = 4560
  W51_write(W5100_SKT_BASE(0) + W5100_PORT_OFFSET + 1, 0xd0);
  W51_execute(W5100_SKT_CR_OPEN);
  // ... assume success
  W51_execute(W5100_SKT_CR_LISTEN);
  // ... assume success

  // now wait for an incoming connection -- may take a while; watchdog will reset us if
  // no connection arrives.
  uint16_t a = 0xffff;
  while(a) {
    watchdogReset();
    if (W51_read(W5100_SKT_BASE(0) + W5100_SR_OFFSET) == W5100_SKT_SR_ESTABLISHED)
      break;
    _delay_ms(9); // 0xffff * 9 == 589 sec
    a--;
  }
}

#ifdef UART1_DEBUG
void debug(char ch) {
  while (!(UART1_SRA & _BV(UDRE0)));
  UART1_UDR = ch;
}
#endif

void putch(char ch) {
#ifndef SOFT_UART
  if (ethernet_mode) {
    // write to w5100
    // wait for space in buffer
    while(W51_read16(W5100_SKT_BASE(0) + W5100_TX_FSR_OFFSET) == 0);
    
    uint16_t tx_ptr;
    tx_ptr = W51_read16(W5100_SKT_BASE(0) + W5100_TX_WR_OFFSET);
    
    W51_write(W5100_TXBUFADDR + (tx_ptr & W5100_TX_BUF_MASK), ch);
    tx_ptr++;
    
    W51_write16(W5100_SKT_BASE(0) + W5100_TX_WR_OFFSET, tx_ptr);
    W51_execute(W5100_SKT_CR_SEND);
  } else {
  
  // read from uart
  while (!(UART_SRA & _BV(UDRE0)));
  UART_UDR = ch;

  }
#else
  __asm__ __volatile__ (
    "   com %[ch]\n" // ones complement, carry set
    "   sec\n"
    "1: brcc 2f\n"
    "   cbi %[uartPort],%[uartBit]\n"
    "   rjmp 3f\n"
    "2: sbi %[uartPort],%[uartBit]\n"
    "   nop\n"
    "3: rcall uartDelay\n"
    "   rcall uartDelay\n"
    "   lsr %[ch]\n"
    "   dec %[bitcnt]\n"
    "   brne 1b\n"
    :
    :
      [bitcnt] "d" (10),
      [ch] "r" (ch),
      [uartPort] "I" (_SFR_IO_ADDR(UART_PORT)),
      [uartBit] "I" (UART_TX_BIT)
    :
      "r25"
  );
#endif
}

uint8_t getch(void) {
  uint8_t ch;

#ifdef LED_DATA_FLASH
#if defined(__AVR_ATmega8__) ||defined (__AVR_ATmega8535__) || defined (__AVR_ATmega16__) || defined (__AVR_ATmega32__)
  LED_PORT ^= _BV(LED);
#else
  LED_PIN |= _BV(LED);
#endif
#endif

#ifdef SOFT_UART
    watchdogReset();
  __asm__ __volatile__ (
    "1: sbic  %[uartPin],%[uartBit]\n"  // Wait for start edge
    "   rjmp  1b\n"
    "   rcall uartDelay\n"              // Get to middle of start bit
    "2: rcall uartDelay\n"              // Wait 1 bit period
    "   rcall uartDelay\n"              // Wait 1 bit period
    "   clc\n"
    "   sbic  %[uartPin],%[uartBit]\n"
    "   sec\n"
    "   dec   %[bitCnt]\n"
    "   breq  3f\n"
    "   ror   %[ch]\n"
    "   rjmp  2b\n"
    "3:\n"
    :
      [ch] "=r" (ch)
    :
      [bitCnt] "d" (9),
      [uartPin] "I" (_SFR_IO_ADDR(UART_PIN)),
      [uartBit] "I" (UART_RX_BIT)
    :
      "r25"
);
#else
  if (ethernet_mode) {
    while (W51_read16(W5100_SKT_BASE(0) + W5100_RX_RSR_OFFSET) == 0);
    // load single byte from ethernet device
    uint16_t rx_ptr;
    rx_ptr = W51_read16(W5100_SKT_BASE(0) + W5100_RX_RD_OFFSET);
    
    ch = W51_read(W5100_RXBUFADDR + (rx_ptr & W5100_RX_BUF_MASK));
    
    W51_write16(W5100_SKT_BASE(0) + W5100_RX_RD_OFFSET, rx_ptr+1);
    W51_execute(W5100_SKT_CR_RECV);
    
#ifdef UART1_DEBUG
    //debug('.');
#endif
    watchdogReset();
  } else {
  
  while(!(UART_SRA & _BV(RXC0)))
    ;
#ifdef UART1_DEBUG
  //debug('.');
#endif
  if (!(UART_SRA & _BV(FE0))) {
      /*
       * A Framing Error indicates (probably) that something is talking
       * to us at the wrong bit rate.  Assume that this is because it
       * expects to be talking to the application, and DON'T reset the
       * watchdog.  This should cause the bootloader to abort and run
       * the application "soon", if it keeps happening.  (Note that we
       * don't care that an invalid char is returned...)
       */
    watchdogReset();
  }

  ch = UART_UDR;
  
  }

#endif

#ifdef LED_DATA_FLASH
#if defined(__AVR_ATmega8__) || defined(__AVR_ATmega8535__) || defined (__AVR_ATmega16__) || defined (__AVR_ATmega32__)
  LED_PORT ^= _BV(LED);
#else
  LED_PIN |= _BV(LED);
#endif
#endif

  return ch;
}

#ifdef SOFT_UART
// AVR305 equation: #define UART_B_VALUE (((F_CPU/BAUD_RATE)-23)/6)
// Adding 3 to numerator simulates nearest rounding for more accurate baud rates
#define UART_B_VALUE (((F_CPU/BAUD_RATE)-20)/6)
#if UART_B_VALUE > 255
#error Baud rate too slow for soft UART
#endif

void uartDelay() {
  __asm__ __volatile__ (
    "ldi r25,%[count]\n"
    "1:dec r25\n"
    "brne 1b\n"
    "ret\n"
    ::[count] "M" (UART_B_VALUE)
  );
}
#endif

void getNch(uint8_t count) {
  do getch(); while (--count);
  verifySpace();
}

void verifySpace() {
  if (getch() != CRC_EOP) {
    watchdogConfig(WATCHDOG_16MS);    // shorten WD timeout
    while (1)                         // and busy-loop so that WD causes
      ;                               //  a reset and app start.
  }
  putch(STK_INSYNC);
}

#if LED_START_FLASHES > 0
void flash_led(uint8_t count) {
  do {
    TCNT1 = -(F_CPU/(1024*16));
    TIFR1 = _BV(TOV1);
    while(!(TIFR1 & _BV(TOV1)));
#if defined(__AVR_ATmega8__) || defined(__AVR_ATmega8535__) || defined (__AVR_ATmega16__) || defined (__AVR_ATmega32__)
    LED_PORT ^= _BV(LED);
#else
    LED_PIN |= _BV(LED);
#endif
    watchdogReset();
  } while (--count);
}
#endif

// Watchdog functions. These are only safe with interrupts turned off.
void watchdogReset() {
  __asm__ __volatile__ (
    "wdr\n"
  );
}

void watchdogConfig(uint8_t x) {
  WDTCSR = _BV(WDCE) | _BV(WDE);
  WDTCSR = x;
}

void appStart(uint8_t rstFlags) {
  // save the reset flags in the designated register
  //  This can be saved in a main program by putting code in .init0 (which
  //  executes before normal c init code) to save R2 to a global variable.
  __asm__ __volatile__ ("mov r2, %0\n" :: "r" (rstFlags));

  watchdogConfig(WATCHDOG_OFF);
  // Note that appstart_vec is defined so that this works with either
  // real or virtual boot partitions.
  __asm__ __volatile__ (
    // Jump to WDT or RST vector
    "ldi r30,%[rstvec]\n"
    "clr r31\n"
    "ijmp\n"::[rstvec] "M"(appstart_vec)
  );
}

/*
 * void writebuffer(memtype, buffer, address, length)
 */
static inline void writebuffer(int8_t memtype, uint8_t *mybuff,
                   uint16_t address, pagelen_t len)
{
    switch (memtype) {
    case 'E': // EEPROM
#if defined(SUPPORT_EEPROM) || defined(BIGBOOT)
        while(len--) {
        eeprom_write_byte((uint8_t *)(address++), *mybuff++);
        }
#else
    /*
     * On systems where EEPROM write is not supported, just busy-loop
     * until the WDT expires, which will eventually cause an error on
     * host system (which is what it should do.)
     */
    while (1)
        ; // Error: wait for WDT
#endif
    break;
    default:  // FLASH
    /*
     * Default to writing to Flash program memory.  By making this
     * the default rather than checking for the correct code, we save
     * space on chips that don't support any other memory types.
     */
    {
        // Copy buffer into programming buffer
        uint8_t *bufPtr = mybuff;
        uint16_t addrPtr = (uint16_t)(void*)address;

        /*
         * Start the page erase and wait for it to finish.  There
         * used to be code to do this while receiving the data over
         * the serial link, but the performance improvement was slight,
         * and we needed the space back.
         */
        do_spm((uint16_t)(void*)address,__BOOT_PAGE_ERASE,0);

        /*
         * Copy data from the buffer into the flash write buffer.
         */
        do {
            uint16_t a;
            a = *bufPtr++;
            a |= (*bufPtr++) << 8;
            do_spm((uint16_t)(void*)addrPtr,__BOOT_PAGE_FILL,a);
            addrPtr += 2;
        } while (len -= 2);

        /*
         * Actually Write the buffer to flash (and wait for it to finish.)
         */
        do_spm((uint16_t)(void*)address,__BOOT_PAGE_WRITE,0);
    } // default block
    break;
    } // switch
}

static inline void read_mem(uint8_t memtype, uint16_t address, pagelen_t length)
{
    uint8_t ch;

    switch (memtype) {

#if defined(SUPPORT_EEPROM) || defined(BIGBOOT)
    case 'E': // EEPROM
    do {
        putch(eeprom_read_byte((uint8_t *)(address++)));
    } while (--length);
    break;
#endif
    default:
    do {
#ifdef VIRTUAL_BOOT_PARTITION
        // Undo vector patch in bottom page so verify passes
        if (address == rstVect0) ch = rstVect0_sav;
        else if (address == rstVect1) ch = rstVect1_sav;
        else if (address == wdtVect0) ch = wdtVect0_sav;
        else if (address == wdtVect1) ch = wdtVect1_sav;
        else ch = pgm_read_byte_near(address);
        address++;
#elif defined(RAMPZ)
        // Since RAMPZ should already be set, we need to use EPLM directly.
        // Also, we can use the autoincrement version of lpm to update "address"
        //      do putch(pgm_read_byte_near(address++));
        //      while (--length);
        // read a Flash and increment the address (may increment RAMPZ)
        __asm__ ("elpm %0,Z+\n" : "=r" (ch), "=z" (address): "1" (address));
#else
        // read a Flash byte and increment the address
        __asm__ ("lpm %0,Z+\n" : "=r" (ch), "=z" (address): "1" (address));
#endif
        putch(ch);
    } while (--length);
    break;
    } // switch
}

/*
 * Separate function for doing spm stuff
 * It's needed for application to do SPM, as SPM instruction works only
 * from bootloader.
 *
 * How it works:
 * - do SPM
 * - wait for SPM to complete
 * - if chip have RWW/NRWW sections it does additionaly:
 *   - if command is WRITE or ERASE, AND data=0 then reenable RWW section
 *
 * In short:
 * If you play erase-fill-write, just set data to 0 in ERASE and WRITE
 * If you are brave, you have your code just below bootloader in NRWW section
 *   you could do fill-erase-write sequence with data!=0 in ERASE and
 *   data=0 in WRITE
 */
static void do_spm(uint16_t address, uint8_t command, uint16_t data) {
    // Do spm stuff
    asm volatile (
    "    movw  r0, %3\n"
    "    out %0, %1\n"
    "    spm\n"
    "    clr  r1\n"
    :
    : "i" (_SFR_IO_ADDR(__SPM_REG)),
        "r" ((uint8_t)command),
        "z" ((uint16_t)address),
        "r" ((uint16_t)data)
    : "r0"
    );

    // wait for spm to complete
    //   it doesn't have much sense for __BOOT_PAGE_FILL,
    //   but it doesn't hurt and saves some bytes on 'if'
    boot_spm_busy_wait();
#if defined(RWWSRE)
    // this 'if' condition should be: (command == __BOOT_PAGE_WRITE || command == __BOOT_PAGE_ERASE)...
    // but it's tweaked a little assuming that in every command we are interested in here, there
    // must be also SELFPRGEN set. If we skip checking this bit, we save here 4B
    if ((command & (_BV(PGWRT)|_BV(PGERS))) && (data == 0) ) {
      // Reenable read access to flash
      boot_rww_enable();
    }
#endif
}
