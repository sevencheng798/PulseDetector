/*DTSensingDA55M.h*/
#ifndef __DTSensing_DA55M_H
#define __DTSensing_DA55M_H
#include "main.h"
#define DTSWriteAddr     0xDA
#define DTSReadAddr      0xDB
/*-----------osr define-------------*/
#define OSR_1024X        0
#define OSR_2048X        1
#define OSR_4096X        2
#define OSR_8192X        3
#define OSR_256X         4
#define OSR_512X         5
#define OSR_16384X       6
#define OSR_32768X       7

/*------------dls define-------------*/
#define OSR_1024X_dls    3//2.5
#define OSR_2048X_dls    4//3.78
#define OSR_4096X_dls    7//6.34
#define OSR_8192X_dls    12//11.46
#define OSR_256X_dls     2//1.54
#define OSR_512X_dls     2//1.86
#define OSR_16384X_dls   22//21.7
#define OSR_32768X_dls   43//42.18

/*-----------read commond define-------*/
#define Read_Temperature 0x08
#define Read_Pressure    0x09
#define Read_P_T         0x0A
#define Sleep_Read       0x0B

HAL_StatusTypeDef SendRegCommand(uint8_t ComRegAddr, uint8_t ComData);
uint8_t ReadRegStatus( uint8_t StatRegAddr );
void ReadPressTemp(uint8_t *PreTemData);

#endif
