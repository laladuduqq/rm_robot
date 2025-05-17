#include "bsp_flash.h"
#include "string.h"
#include "main.h"
//第二个参数 pages 传递要擦除的扇区数量，而不是字节数

#define LOG_TAG "bsp_flash"
#include "elog.h"
Flash_Status BSP_Flash_Erase(uint32_t sector, uint32_t pages) {
    FLASH_EraseInitTypeDef erase;
    uint32_t sector_error;

    log_i("Start erasing flash sector %lu, pages: %lu", sector, pages);
    erase.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase.Sector = sector;
    erase.NbSectors = pages;
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    HAL_FLASH_Unlock();

    if (HAL_FLASHEx_Erase(&erase, &sector_error) != HAL_OK) {
        log_e("Flash erase failed at sector: %lu, error sector: %lu", sector, sector_error);
        HAL_FLASH_Lock();
        return FLASH_ERR_ERASE;
    }

    HAL_FLASH_Lock();
    log_i("Flash erase completed successfully");
    return FLASH_OK;
}

Flash_Status BSP_Flash_Write(uint32_t addr, uint8_t *data, uint16_t size) {
    if (size % 4 != 0 || addr % 4 != 0){
        log_e("Invalid parameters: addr=0x%08lX, size=%u (both must be 4-byte aligned)", addr, size);
        return FLASH_ERR_PARAM;
    }
    log_i("Start writing %u bytes to flash address 0x%08lX", size, addr);
    HAL_FLASH_Unlock();

    for (uint16_t i = 0; i < size; i += 4) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,addr + i,*(uint32_t*)(data + i)) != HAL_OK) {
            log_e("Flash write failed at address 0x%08lX", addr + i);
            HAL_FLASH_Lock();
            return FLASH_ERR_WRITE;
        }
    }
    HAL_FLASH_Lock();
    log_i("Flash write completed successfully");
    return FLASH_OK;
}

Flash_Status BSP_Flash_Read(uint32_t addr, uint8_t *buf, uint16_t size) {
    if (!IS_FLASH_ADDRESS(addr) || !buf || size == 0){
        log_e("Invalid parameters for flash read: addr=0x%08lX, buf=%p, size=%u", addr, buf, size); 
        return FLASH_ERR_PARAM;
    }
    log_i("Reading %u bytes from flash address 0x%08lX", size, addr);
    memcpy(buf, (void*)addr, size);
    return FLASH_OK;
}
