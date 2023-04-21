#include "xdot_eeprom.h"
#include "mbed.h"

#define address     (0x51 << 1)     // I2C Address of CAT24S128

extern I2C i2c0;
extern DigitalOut mpe;

void WriteByte(uint16_t addr, uint8_t* buf)
{
    char seq[3]; 
    seq[0] = char(addr >> 8);       // Upper Address Byte
    seq[1] = char(addr & 0xFF);     // Lower Address Byte
    seq[2] = buf[0];                // Data Byte

    // mpe.write(0);                   // MEM_PWR_EN active low, turn on EEPROM
    WriteEnable();                  // Turn off write protection

    i2c0.write(address, seq, 3);    // Write data

    WriteDisable();                 // Turn on write protection

    // mpe.write(1);                   // Turn off EEPROM
}

void ReadByte(uint16_t addr, uint8_t* buf)
{
    char data;

    // mpe.write(0);                   // MEM_PWR_EN active low

    SetReadAddress(addr);
    // printf("Reading a byte..\r\n");
    i2c0.read(address, &data, 1);
    buf[0] = data;

    // mpe.write(1);
}

void WaitForChip()
{
    // printf("WaitForChip(): Writing mpe low\r\n");
    mpe.write(0);
    // printf("WaitForChip(): MXC_GPIO0->out: %d\r\n", (((MXC_GPIO0->out) >> 23) & 1));
    // printf("WaitForChip(): Waiting 100ms\r\n");
    // ThisThread::sleep_for(3ms);
    while(i2c0.read(address, 0x00, 1, false) != 0)
    {
        // printf("WaitForChip(): Ack not received..\r\n");
        // printf("WaitForChip(): Writing mpe low\r\n");
        // printf("WaitForChip(): MXC_GPIO0->out: %d\r\n", (((MXC_GPIO0->out) >> 23) & 1));
        mpe.write(0);
        // printf("WaitForChip(): Waiting 100ms\r\n");
        // ThisThread::sleep_for(3ms);
        // printf("Waiting\r\n");
    }
    // printf("WaitForChip(): Ack received, continuing\r\n");
}

void SetReadAddress(uint16_t readAddr)
{
    char seq[2];
    seq[0] = char(readAddr >> 8);
    seq[1] = char(readAddr & 0xFF);
    WaitForChip();
    // printf("Setting read address..\r\n");
    i2c0.write(address, seq, 2, true);
    // WaitForChip();
}

void WriteEnable()
{
    WaitForChip();
    char wpSettings = 0x00;
    char wpReg[] = {0xFF, 0xFF, wpSettings};
    i2c0.write(address, wpReg, 3, false);     // repeated = false, send stop
    WaitForChip();
}

void WriteDisable()
{
    WaitForChip();
    char wpSettings = 0x00;
    wpSettings |= (1 << BP0) | (1 << BP1) | (1 << WPEN);    // Enable write protection for full array
    char wpReg[] = {0xFF, 0xFF, wpSettings};
    i2c0.write(address, wpReg, 3, false);
    WaitForChip();
}

int PageWrite(uint16_t addr, uint8_t *buf, uint8_t size)
{
    // A13 - A6 is the page, want page as input or just address? 
    // Find location in page from address
    uint8_t offset = addr & 0x3F; // Offset in page

    // Check that size isn't going to wrap around page
    if((size + offset) > 64) return -1001; // wrap around error

    // Copy buffer
    char* temp;
    temp = new char[size + 2];
    temp[0] = addr >> 8;
    temp[1] = addr & 0xFF;

    memcpy(temp + 2, buf, sizeof(char)*size);

    // Begin Page Write
    mpe.write(0);
    WriteEnable();
    i2c0.write(address, temp, size + 2);
    WriteDisable();
    mpe.write(1);

    delete temp;

    return 0;
}

int SequentialRead(uint16_t addr, uint8_t *buf, uint32_t size)
{
    mpe.write(0);

    SetReadAddress(addr);
    i2c0.read(address, (char*)buf, size, false);

    mpe.write(1);

    return 0;
}

int xdot_eeprom_write_buf(uint32_t addr, uint8_t *buf, uint32_t size)
{
    // Implement read before write, as in L151CC eeprom driver
    // No 32 bit word alignment needed
    //logInfo("Writing data from 0x%08X to 0x%08X", addr, addr + size);
    mpe.write(0);
    WriteEnable();
    uint16_t bytes_written = 0;
    uint8_t data_read;
    while(bytes_written < size)
    {
        ReadByte(addr + bytes_written, &data_read);
        if(data_read != buf[bytes_written]) 
        {
            uint16_t position = addr + bytes_written;
            WriteByte(position, buf + bytes_written);
        }
        bytes_written += 1;
    }
    WriteDisable();
    mpe.write(1);

    return 0;
}

int xdot_eeprom_read_buf(uint32_t addr, uint8_t *buf, uint32_t size)
{
    // logInfo("Begin - Reading memory from 0x%08X to 0x%08X", addr, addr + size);
    if (addr + size > EEPROM_SIZE )
    {
        return -1;
    }
    mpe.write(0);
    uint16_t bytes_read = 0;
    while(bytes_read < size)
    {
        ReadByte(addr + bytes_read, buf + bytes_read);
        bytes_read += 1;
    }
    mpe.write(1);

    return 0;
}

int EraseChip(void)
{
    uint8_t* buffer;
    buffer = new uint8_t[PAGE_SIZE];

    memset(buffer, 0, PAGE_SIZE);
    for(uint16_t i = 0; i < NUM_PAGES; i++)
    {
        PageWrite(i*PAGE_SIZE, buffer, PAGE_SIZE);
    }
	delete buffer;

    return 0;
}