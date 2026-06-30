#ifndef MY_INC_H
#define MY_INC_H

/*=========显示类==========*/
// #define CD_RST_PIN  PIN_51
// #define LCD_DC_PIN   PIN_50
// #define LCD_BL_PIN   PIN_74

#define CD_RST_PIN  PIN_50
#define LCD_DC_PIN   PIN_74
#define LCD_BL_PIN   PIN_51
#include <iostream>
#include <unistd.h>
#include "NcnnDetector.h"
#include "Camera.h"
#include "Pid.h"
#include "Motor.h"
#include "All_control.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "LS2K0300_DRV_INC.h"
#include "Encoder.h"


#ifdef __cplusplus
}
#endif

#endif /* MY_INC_H */
