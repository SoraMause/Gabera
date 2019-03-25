#ifndef __FLASH_H
#define __FLASH_H

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdint.h>

// flash use address ( sector11 )
extern const uint32_t start_address; //sentor11 start address
extern const uint32_t end_adress;

void eraseFlash( void );
void writeFlash(uint32_t address, uint8_t *data, uint32_t size );
void loadFlash(uint32_t address, uint8_t *data, uint32_t size );

#ifdef __cplusplus
 }
#endif

#endif /* __FLASH_H */