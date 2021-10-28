#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "stdio.h"
#include "stm32_lcd.h"
#include "stm32h747i_discovery.h"
#include "stm32h747i_discovery_camera.h"
#include "stm32h747i_discovery_conf.h"
#include "stm32h747i_discovery_lcd.h"
#include "stm32h747i_discovery_qspi.h"
#include "stm32h747i_discovery_sdram.h"
#include "stm32h747i_discovery_ts.h"
#include "stm32h7xx_hal.h"
#include "string.h"

/* Exported variables --------------------------------------------------------*/
extern __IO uint32_t SRAMTest;
#ifndef USE_FULL_ASSERT
extern uint32_t ErrorCounter;
#endif
extern uint32_t JoyStickDemo;
extern __IO uint32_t ButtonState;
extern __IO uint32_t JoystickStates;
extern __IO uint32_t CameraTest;
extern __IO uint32_t SdramTest;
extern __IO uint32_t SdmmcTest;

/* Exported constants --------------------------------------------------------*/
/**
 * @brief  SDRAM Write read buffer start address after CAM Frame buffer
 * Assuming Camera frame buffer is of size 800x480 and format ARGB8888 (32 bits
 * per pixel).
 */
#define SDRAM_WRITE_READ_ADDR_OFFSET ((uint32_t)0x0800)

#define LED_GREEN LED1
#define LED_ORANGE LED2
#define LED_RED LED3
#define LED_BLUE LED4

#define CAMERA_FRAME_TMP 0xD020D000

#define SDRAM_WRITE_READ_ADDR 0xD0177000

void error_handler(void);

#endif /* __MAIN_H */
