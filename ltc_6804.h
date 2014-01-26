/*! \file 
 * File:        ltc_6804.h
 * Author:      martin
 * Created:     July 27, 2013, 1:38 AM
 */
#pragma once

#include "ltc6804_regs.h"
#include <sys/types.h>



/***                Definitions                 ***/

/***          Public Global Variables           ***/





/***              Public Functions              ***/

int ltc6804_init(u_int8_t address, const char *SPI_interface, u_int32_t SPI_speed);
void ltc6804_close(void);
int ltc6804_readStatA(u_int8_t address, struct ltc6804_stat_reg_a_s *g);
int ltc6804_readStatB(u_int8_t address, struct ltc6804_stat_reg_b_s *g);
int ltc6804_readConfig(u_int8_t address, struct ltc6804_cfg_reg_s *g);
int ltc6804_writeConfig(u_int8_t address, struct ltc6804_cfg_reg_s *g);
int ltc6804_readCellVoltageGroup(u_int8_t address, char group, struct ltc6804_cell_voltage_reg_s *g);
int ltc6804_readCellVoltageGroups(u_int8_t address, struct ltc6804_cell_voltage_s *c);
int ltc6804_readAuxGroupA(u_int8_t address, struct ltc6804_aux_reg_a_s *g);
int ltc6804_readAuxGroupB(u_int8_t address, struct ltc6804_aux_reg_b_s *g);
int ltc6804_startCellVoltageConversion(u_int8_t address,
        enum ltc6804_ADC_modes_e mode,
        enum ltc6804_CH_e cells);
int ltc6804_startGpioVoltageConversion(u_int8_t address,
        enum ltc6804_ADC_modes_e mode,
        enum ltc6804_CHG_e chans);
int ltc6804_startStatVoltageConversion(u_int8_t address,
        enum ltc6804_ADC_modes_e mode,
        enum ltc6804_CHST_e chans);
int ltc6804_startMuxDiag(u_int8_t address);
int ltc6804_startCellGpioVoltageConversion(u_int8_t address,
        enum ltc6804_ADC_modes_e mode);

