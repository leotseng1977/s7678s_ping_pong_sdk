/*
    AcSiP(C)2016
*/
#ifndef __EEPROM_MCU_H__
#define __EEPROM_MCU_H__

/*!
 * \brief Write a word-wide data into EEPROM, the address would be accumulated 4 bytes after writing.
 *
 * \param [IN] word  A 32-bit word data
 */
void WriteOneWordByAccumulatedAddrEEPROM(uint32_t word);

/*!
 * \brief Write a word-wide data into EEPROM, the address would be accumulated 4 bytes after writing.
 *
 * \param [IN] src  the source address of what it is ready to be written into EEPROM.
 * \param [IN] length_word  the length of writting data
 */
void WriteSectionByAccumulatedAddrEEPROM(uint32_t* src, uint32_t length_word);

/*!
 * \brief Write a word-wide data into EEPROM, the address would be accumulated 4 bytes after writing.
 *
 * \param [IN] word  A 32-bit byte data (Only lower 8-bit data written)
 */
void WriteOneByteByAccumulatedAddrEEPROM(uint32_t byte);

/*!
 * \brief Read a word-wide data from EEPROM, the address would be decreased 4 bytes after reading.
 *
 * \retval value  Current 32-bit EEPROM value,
 */
uint32_t ReadOneWordByDecreasedAddrEEPROM(void);

/*!
 * \brief Read a word-wide data from EEPROM, the address would not change.
 *
 * \param [IN] SerialNo  Serial Number of EEPROM from Word0 to Word1535 (Total 1536W = 6KB)
 * \retval value  A 32-bit EEPROM value for the input SN,
 */
uint32_t ReadOneWordBySerialNoEEPROM(int32_t SerialNo);

/*!
 * \brief Erase all the EEPROM data from Word0 to Word1535 (6KB) to 0x00
 */
void EraseAllDataEEPROM(void);

/*!
 * \brief Erase the requested EEPROM data
 * \param [IN] len  Word0 to Word((len/4)-1) would be erased
 */
void EraseByLengthEEPROM(uint32_t length);

/*!
 * \brief EEPROM Erase, Read and Write Demonstration.
 */
void DEMO_EEPROM(void);

#endif // __EEPROM_MCU_H__
