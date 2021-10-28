#pragma once

#include "stm32_lcd.h"
#include "stm32h747i_discovery.h"
#include "stm32h747i_discovery_conf.h"
#include "stm32h747i_discovery_lcd.h"
#include "stm32h747i_discovery_qspi.h"
#include "stm32h747i_discovery_sdram.h"
#include "stm32h747i_discovery_ts.h"
#include "stm32h7xx_hal.h"

class Board {
 public:
  Board(){};
  void init();
  static void error_handler();

 private:
  void clock_config();
  void MPU_config();
  void cache_enable();
};
