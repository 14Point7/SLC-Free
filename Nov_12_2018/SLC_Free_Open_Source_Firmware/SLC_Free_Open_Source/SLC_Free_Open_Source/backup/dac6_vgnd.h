//*****************************************************************************
//*****************************************************************************
//  FILENAME: DAC6_VGND.h
//   Version: 4.3, Updated on 2015/3/4 at 22:25:9
//  Generated by PSoC Designer 5.4.3191
//
//  DESCRIPTION:  DAC6 User Module C Language interface file.
//
//-----------------------------------------------------------------------------
//      Copyright (c) Cypress Semiconductor 2015. All Rights Reserved.
//*****************************************************************************
//*****************************************************************************
#ifndef DAC6_VGND_INCLUDE
#define DAC6_VGND_INCLUDE

#include <M8C.h>

//-------------------------------------------------
// Defines for DAC6_VGND API's.
//-------------------------------------------------
#define DAC6_VGND_OFF         0
#define DAC6_VGND_LOWPOWER    1
#define DAC6_VGND_MEDPOWER    2
#define DAC6_VGND_HIGHPOWER   3
#define DAC6_VGND_FULLPOWER   3

#pragma fastcall16 DAC6_VGND_Start
#pragma fastcall16 DAC6_VGND_SetPower
#pragma fastcall16 DAC6_VGND_WriteBlind
#pragma fastcall16 DAC6_VGND_WriteStall
#pragma fastcall16 DAC6_VGND_Stop

//-------------------------------------------------
// Prototypes of the DAC6_VGND API.
//-------------------------------------------------
extern void DAC6_VGND_Start(BYTE bPowerSetting);
extern void DAC6_VGND_SetPower(BYTE bPowerSetting);
extern void DAC6_VGND_WriteBlind(CHAR cOutputValue);
extern void DAC6_VGND_WriteStall(CHAR cOutputValue);
extern void DAC6_VGND_Stop(void);


//-------------------------------------------------
// Register Addresses for DAC6_VGND
//-------------------------------------------------
#pragma ioport  DAC6_VGND_CR0:  0x080
BYTE            DAC6_VGND_CR0;
#pragma ioport  DAC6_VGND_CR1:  0x081
BYTE            DAC6_VGND_CR1;
#pragma ioport  DAC6_VGND_CR2:  0x082
BYTE            DAC6_VGND_CR2;
#pragma ioport  DAC6_VGND_CR3:  0x083
BYTE            DAC6_VGND_CR3;

#endif
// end of file DAC6_VGND.h
