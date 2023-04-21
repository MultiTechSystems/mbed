/*  CAT24S128 Driver
 *
 *  CAT24S128:
 *      size:           128 Kb
 *                      16,384 words (16KB, 0x4000)
 *                      256 pages, 64 B each
 *      7-bit i2c:      0x51    (0b1010001)
 *      byte write:     write a single byte
 *      page write:     write a page
 *                      256 pages of 64 bytes each
 *      write protect:  
 *                      |  7  |  6  |  5  |  4  |  3  |  2  |  1  |  0  |
 *                      |  X  |  X  |  X  |  X  |WPEN | BP1 | BP0 | WPL |
 *                      WPEN:   Write Protect Enable
 *                      BP1:    Block Protect 1
 *                      BP0:    Block Protect 0
 *                      WPL:    Write Protect Lock (Permanent)
 */

#ifndef __XDOT_EEPROM__H
#define __XDOT_EEPROM__H
#include "mbed.h"

#define ADDRESS_7   0x51
#define PAGE_SIZE   64
#define EEPROM_SIZE 16384
#define NUM_PAGES   (EEPROM_SIZE)/(PAGE_SIZE)

// Write Protect Bits
#define WPL         0   // Permanently Lock EEPROM
#define BP0         1   // Block Protect 0, used for upper half and full array protection
#define BP1         2   // Block Protect 1, used for upper 3/4 array protection and full array protection
#define WPEN        3   // Write Protect Enable, when '0' all memory can be written

// Implementations of L151CC WriteBuffer and ReadBuffer functions
int xdot_eeprom_write_buf(uint32_t addr, uint8_t *buf, uint32_t size);
int xdot_eeprom_read_buf(uint32_t addr, uint8_t *buf, uint32_t size);

//
int PageWrite(uint16_t addr, uint8_t *buf, uint8_t size);

// 
int SequentialRead(uint16_t addr, uint8_t *buf, uint32_t size);

// Debug
void DumpPage(uint8_t page);

// Write a byte to 16-bit memory address on EEPROM
void WriteByte(uint16_t addr, uint8_t *buf);

// Read a byte from 16-bit memory address on EEPROM
void ReadByte(uint16_t addr, uint8_t *buf);

// Read a byte from the next memory address on EEPROM
void ReadByteImmediate(uint16_t addr, uint8_t *buf);

// Set memory address pointer on EEPROM, this is required for selective read anyway.
void SetReadAddress(uint16_t readAddr);

// Read EEPROM I2C address until ACK is returned
void WaitForChip();

// Enable write operations by configuring write protect register
void WriteEnable();

// Disable write operations by configuring write protect register
void WriteDisable();

// Erase chip (used for formatting factory data)
int EraseChip(void);

#endif // __XDOT_EEPROM__H