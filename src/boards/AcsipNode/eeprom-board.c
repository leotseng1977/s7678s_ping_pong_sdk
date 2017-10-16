/*
    AcSiP(C)2016
*/

#include "board.h"
#include "eeprom-board.h"

#define WORD_LENGTH 4
#define BYTE_LENGTH 1
#define ERROR_PATTERN_EEPROM 0xDEADBEAF

#if defined (STM32L073xx)
#define MAX_LENGTH_EEPROM (6 * 1024)
#else
#define MAX_LENGTH_EEPROM (0)
#endif

static int32_t eeprom_written_length = 0;

void WriteOneWordByAccumulatedAddrEEPROM(uint32_t word)
{
    //Check not to exceed the ceiling of EEPROM length.
    if ((eeprom_written_length + WORD_LENGTH) > MAX_LENGTH_EEPROM)
        return;

    //Align Word Boundary
    if ((eeprom_written_length % 4) != 0) {
        eeprom_written_length = eeprom_written_length
            - (eeprom_written_length % 4) + WORD_LENGTH;
    }

    if ( HAL_OK == HAL_FLASHEx_DATAEEPROM_Unlock() ) {
        HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_WORD,
                                       DATA_EEPROM_BASE + eeprom_written_length,
                                       word);
        eeprom_written_length += WORD_LENGTH;
        HAL_FLASHEx_DATAEEPROM_Lock();
    }
}

void WriteSectionByAccumulatedAddrEEPROM(uint32_t* src, uint32_t length_word)
{
    uint32_t word = 0;

    //Check not to exceed the ceiling of EEPROM length.
    if ((eeprom_written_length + WORD_LENGTH) > MAX_LENGTH_EEPROM)
        return;

    //Align Word Boundary
    if ((eeprom_written_length % 4) != 0) {
        eeprom_written_length = eeprom_written_length
            - (eeprom_written_length % 4) + WORD_LENGTH;
    }

    if ( HAL_OK == HAL_FLASHEx_DATAEEPROM_Unlock() ) {
        for ( uint32_t len = 0; len < length_word; len++ ) {
            word = *(src + len);
            HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_WORD,
                                           DATA_EEPROM_BASE + eeprom_written_length,
                                           word);

            eeprom_written_length += WORD_LENGTH;
        }
    }
    HAL_FLASHEx_DATAEEPROM_Lock();
}

void WriteOneByteByAccumulatedAddrEEPROM(uint32_t byte)
{
    //Check not to exceed the ceiling of EEPROM length.
    if ((eeprom_written_length + BYTE_LENGTH) > MAX_LENGTH_EEPROM)
        return;

    if ( HAL_OK == HAL_FLASHEx_DATAEEPROM_Unlock() ) {
        HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_BYTE,
                                       DATA_EEPROM_BASE + eeprom_written_length,
                                       byte);
        eeprom_written_length += BYTE_LENGTH;
        HAL_FLASHEx_DATAEEPROM_Lock();
    }
}

uint32_t ReadOneWordByDecreasedAddrEEPROM(void)
{
    eeprom_written_length -= WORD_LENGTH;

    if ( eeprom_written_length >= MAX_LENGTH_EEPROM ) {
        eeprom_written_length = MAX_LENGTH_EEPROM;
        return ERROR_PATTERN_EEPROM;
    } else if ( eeprom_written_length < 0 ) {
        eeprom_written_length = 0;
        return ERROR_PATTERN_EEPROM;
    } else {
        uint32_t addr = DATA_EEPROM_BASE + eeprom_written_length;
        return  *(uint32_t*)addr;
    }
}

uint32_t ReadOneWordBySerialNoEEPROM(int32_t SerialNo)
{
    // Only Allow to access from 0 to (1536 -1) words (for L073)
    if(SerialNo < 0 || SerialNo >= (MAX_LENGTH_EEPROM / WORD_LENGTH))
        return ERROR_PATTERN_EEPROM;

    return *(uint32_t*)(DATA_EEPROM_BASE + (SerialNo * WORD_LENGTH));
}

void EraseAllDataEEPROM(void)
{
    // Erase All Data (6 * 1024, L073) in EEPROM
    if ( HAL_OK == HAL_FLASHEx_DATAEEPROM_Unlock() ) {
        for ( uint32_t addr = DATA_EEPROM_BASE;
              addr < (DATA_EEPROM_BASE + MAX_LENGTH_EEPROM);
              addr += WORD_LENGTH )
            HAL_FLASHEx_DATAEEPROM_Erase(addr);

        HAL_FLASHEx_DATAEEPROM_Lock();
    }
}

void EraseByLengthEEPROM(uint32_t length)
{
    if (length > MAX_LENGTH_EEPROM)
        length = MAX_LENGTH_EEPROM;

    if ( HAL_OK == HAL_FLASHEx_DATAEEPROM_Unlock() ) {
        for ( uint32_t addr = DATA_EEPROM_BASE;
              addr < (DATA_EEPROM_BASE + length);
              addr += WORD_LENGTH )
            HAL_FLASHEx_DATAEEPROM_Erase(addr);

        HAL_FLASHEx_DATAEEPROM_Lock();
    }
}

void DEMO_EEPROM(void) {
    uint32_t val = 0;
    uint32_t valArray[7];
    EraseAllDataEEPROM();

    WriteOneWordByAccumulatedAddrEEPROM(0x12345678); // Word0
    WriteOneWordByAccumulatedAddrEEPROM(0x9ACDEF01); // Word1
    WriteOneWordByAccumulatedAddrEEPROM(0xAAAA5555); // Word2
    WriteOneByteByAccumulatedAddrEEPROM(0x33);       // Word3, Byte0
    WriteOneByteByAccumulatedAddrEEPROM(0x66);       // Word3, Byte1
    WriteOneByteByAccumulatedAddrEEPROM(0x99);       // Word3, Byte2
    WriteOneWordByAccumulatedAddrEEPROM(0x5555AAAA); // Word4

    UartPrint("EEPROM Content (By Sequence):\n");
    for ( int32_t len = eeprom_written_length; len > 0; len -= WORD_LENGTH )
    {
        val = ReadOneWordByDecreasedAddrEEPROM();
        UartPrint("0x%X, ", val);
    }

    valArray[0] = ReadOneWordBySerialNoEEPROM(3);    // Output Word3
    valArray[1] = ReadOneWordBySerialNoEEPROM(2);    // Output Word2
    valArray[2] = ReadOneWordBySerialNoEEPROM(4);    // Output Word4
    valArray[3] = ReadOneWordBySerialNoEEPROM(0);    // Output Word0
    valArray[4] = ReadOneWordBySerialNoEEPROM(1);    // Output Word1

    //Only allow to read word 0 to word 1535 (6K / 4 = 1536)
    valArray[5] = ReadOneWordBySerialNoEEPROM(-1);   // Out of Boundary, Output "0xDEADBEAF"
    valArray[6] = ReadOneWordBySerialNoEEPROM(1540); // Out of Boundary, Output "0xDEADBEAF"

    UartPrint("\nEEPROM Content (Ramdonly):\n");
    for ( int32_t i = 0; i < 7; i++ )
    {
        UartPrint("0x%X, ", valArray[i]);
    }
    UartPrint("\n");
}
