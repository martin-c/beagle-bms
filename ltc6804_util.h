/*! \file 
 * File:        ltc6804_util.h
 * Author:      Martin Clemons
 * Created:     August 5, 2013
 */
#pragma once

#include "ltc6804_regs.h"
#include <sys/types.h>



/***                Definitions                 ***/

/***          Public Global Variables           ***/

/***              Public Functions              ***/

void ltc6804util_initPec(void);
u_int16_t ltc6804util_calcPec(u_int8_t *data, int len);
double ltc6804util_convertSoc(u_int16_t soc);
double ltc6804util_convertItmp(u_int16_t itmp);
double ltc6804util_convertVa_Vd(u_int16_t v);
double ltc6804util_convertV(u_int16_t v);
void ltc6804util_encodeCommand(u_int8_t addr, enum ltc6804_command_codes_e cmd, u_int8_t *b);
void ltc6804util_decodeStatA(void *g, u_int8_t *b);
void ltc6804util_decodeStatB(void *g, u_int8_t *b);
void ltc6804util_decodeCfg(void *g, u_int8_t *b);
void ltc6804util_encodeCfg(u_int8_t *b, void *g);
void ltc6804util_decodeCV(void *g, u_int8_t *b);
void ltc6804util_decodeAuxA(void *g, u_int8_t *b);
void ltc6804util_decodeAuxB(void *g, u_int8_t *b);


