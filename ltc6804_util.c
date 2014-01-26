
/*! \file 
 * File:        ltc6804_util.c
 * Author:      Martin Clemons
 * Created:     August 5, 2013
 * 
 * Basic utility functions for communication with the LTC6804 Battery Stack Monitor.
 * Created for Extended Lab, ECEN5623 Summer 2013.
 * 
 * Note: The PEC15 calculation is based on LTC example code with the following
 * copyright information:
 * Copyright 2012 Linear Technology Corp. (LTC)
 * Permission to freely use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies:
 * THIS SOFTWARE IS PROVIDED “AS IS” AND LTC DISCLAIMS ALL WARRANTIES
 * INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO
 * EVENT SHALL LTC BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL
 * DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM ANY USE OF SAME, INCLUDING
 * ANY LOSS OF USE OR DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE
 * OR OTHER TORTUOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR
 * PERFORMANCE OF THIS SOFTWARE.
 * 
 * All other portions (C) Martin Clemons 2013.
 * 
 */

#include "ltc6804_regs.h"
#include "ltc6804_util.h"
//#include "beagle_spi.h"
#include <stdio.h>
//#include <unistd.h>
//#include <stdlib.h>
//#include <stdbool.h>
//#include <fcntl.h>
//#include <string.h>

//#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>

//#include <linux/types.h>
//#include <linux/spi/spidev.h>



/***                Definitions                 ***/

/***         Private Global Variables           ***/

// PEC calculation table
static int16_t pec15Table[256];



/***          Public Global Variables           ***/

/***        Private Function Prototypes         ***/

static inline void decodeReg(u_int8_t *buf, u_int16_t *reg1, u_int16_t *reg2, u_int16_t *reg3);
static enum ltc6804_PEC_e checkGroupPec(u_int8_t *b);



/***             Private Functions              ***/

/*! Decode a generic 6 register group into 3 16-bit registers.
 *  \param buf Pointer to 6-byte register group buffer.
 *  \param reg1 Pointer to first 16 bit register.
 *  \param reg2 Pointer to second 16 bit register.
 *  \param reg3 Pointer to third 16 bit register.
 */
static inline void decodeReg(u_int8_t *buf, u_int16_t *reg1, u_int16_t *reg2, u_int16_t *reg3) {

   *reg1 = buf[0] | ((u_int16_t)buf[1]) << 8;
   *reg2 = buf[2] | ((u_int16_t)buf[3]) << 8;
   *reg3 = buf[4] | ((u_int16_t)buf[5]) << 8;
}

/*! Check PEC for a register group.
 *  \param b Buffer containing received data, 6 data bytes + 2 PEC bytes.
 *  \return Function returns PEC_OK if calculated PEC from 6 data bytes matches
 *  PEC received in 7th and 8th byte. Function returns PEC_ERROR on mis-match.
 */
static enum ltc6804_PEC_e checkGroupPec(u_int8_t *b) {
    
    u_int16_t pec = ltc6804util_calcPec(b, 6);              // PEC of data bytes
    u_int16_t rxPec = ((u_int16_t)b[6]) << 8 | b[7];        // received PEC
    if (pec != rxPec) {
        return PEC_ERROR;
    }
    return PEC_OK;
}



/***              Public Functions              ***/

/*! Pre-compute PEC15 checksum table for fast checksumming.
 *  Function is based on code supplied in the LTC6804 datasheet.
 */
void ltc6804util_initPec(void)
{
    const int16_t CRC15_POLY = 0x4599;
    int i;
    for (i = 0; i < 256; i++) {
        int remainder = i << 7;
        int bit;
	for (bit = 8; bit > 0; bit--) {
            if (remainder & 0x4000) {
                    remainder <<= 1;
                    remainder ^= CRC15_POLY;
            } else {
                    remainder <<= 1;
            }
        }
        pec15Table[i] = remainder & 0xFFFF;
    }
}

/*! Perform PEC15 checksum calculation based on pre-initialized table.
 *  \param data Pointer to data which should be checksummed.
 *  \param len Length in bytes of data to be checksummed.
 *  \return Function returns PEC15 checksum of data, << by 1 bit.
 */
u_int16_t ltc6804util_calcPec(u_int8_t *data, int len)
{
    u_int16_t remainder, address;
    int i;
    remainder = 16;                     // PEC seed
    for (i = 0; i < len; i++) {
        //calculate PEC table address
        address = ((remainder >> 7) ^ data[i]) & 0xff;
	remainder = (remainder << 8 ) ^ pec15Table[address];
    }
    // The CRC15 has a 0 in the LSB so the final value must be << by 1.
    return remainder << 1;
}

/*! Convert raw ADC SOC value to voltage.
 *  \param soc SOC ADC value.
 *  \return Sum of Cells voltage.
 */
double ltc6804util_convertSoc(u_int16_t soc) {
    
    return soc * 100E-6 * 20;
}

/*! Convert raw ADC ITMP value to temperature.
 *  \param itmp ADC value.
 *  \return Internal Die Temperature in deg. C.
 */
double ltc6804util_convertItmp(u_int16_t itmp) {
    
    return itmp * 100E-6 / 7.5E-3 - 273.0;
}

/*! Convert raw ADC VA or VD value to voltage.
 *  \param v VA or VD ADC value.
 *  \return Analog (VA) or digital (VD) supply voltage.
 */
double ltc6804util_convertVa_Vd(u_int16_t v) {
    
    return v * 100E-6;
}

/*! Convert raw ADC V_cell or V_GPIO value to voltage.
 *  \param v V_cell or V_GPIO ADC value.
 *  \return Analog voltage sensed by ADC.
 */
double ltc6804util_convertV(u_int16_t v) {
    
    return v * 100E-6;
}

/*! Encode a command + PEC into buffer specified.
 *  \param addr LTC6804-2 device address (0-F).
 *  \param cmd Command to encode
 *  \param b buffer into which encoded command + PEC should be written.
 *  Buffer needs to be at least 4 bytes long.
 */
void ltc6804util_encodeCommand(u_int8_t addr, enum ltc6804_command_codes_e cmd, u_int8_t *b) {
    
    if (b == NULL) {
        return;
    }
    cmd &= 0x7FF;                               // mask unused bits in cmd
    addr &= 0x0F;                               // mask unused bits in addr
    u_int8_t cmdHi = (u_int8_t)(cmd >> 8);      // CMD hi byte
    u_int8_t cmdLo = (u_int8_t)cmd;             // CMD lo byte
    // see table 33 in '6804 datasheet for more information on bit layout.
    b[0] = 0x80 | (addr << 3) | cmdHi;
    b[1] = cmdLo;
    u_int16_t pec = ltc6804util_calcPec(b, 2);  // PEC of command bytes
    b[2] = (u_int8_t)(pec >> 8);                // pec hi
    b[3] = (u_int8_t)pec;                       // pec lo
}

/*! Decode the '6804 Status Register Group A, check data PEC.
 *  \param g Pointer to register group struct into which register data will be copied.
 *  \param b Pointer to buffer containing register data to be decoded.
 */
void ltc6804util_decodeStatA(void *g, u_int8_t *b) {
    
    struct ltc6804_stat_reg_a_s *r = g;
    // test PEC
    if (checkGroupPec(b) == PEC_ERROR) {
        r->pecOk = PEC_ERROR;
    }
    // copy bytes
    r->soc = b[0] | (u_int16_t)b[1] << 8;
    r->itmp = b[2] | (u_int16_t)b[3] << 8;
    r->va = b[4] | (u_int16_t)b[5] << 8;
}

/*! Decode the '6804 Status Register Group B, check data PEC.
 *  \param g Pointer to register group struct into which register data will be copied.
 *  \param b Pointer to buffer containing register data to be decoded.
 */
void ltc6804util_decodeStatB(void *g, u_int8_t *b) {
    
    struct ltc6804_stat_reg_b_s *r = g;
    // test PEC
    if (checkGroupPec(b) == PEC_ERROR) {
        r->pecOk = PEC_ERROR;
    }
    // copy bytes
    r->vd = b[0] | (u_int16_t)b[1] << 8;
    // copy flags for each cell, packed as 4 cells per byte spanning 3 bytes.
    int i, byte;
    enum ltc6804_cell_flags_e *currFlag = r->cell_flags;
    for (byte = 0; byte < 3; byte++) {
        // for each byte in rx buffer containing cell flags (3)
        for (i = 0; i < 4; i++) {
            // for each flag pair in a register
            *currFlag = (b[2 + byte] >> (i * 2)) & 0x03;
            currFlag++;
        }
    }
    r->rev = b[5] >> 4;
    r->muxfail = (b[5] >> 1) & 0x01;
    r->thsd = b[5] & 0x01;
}

/*! Decode the '6804 Configuration Register Group, check data PEC.
 *  \param g Pointer to register group struct into which register data will be copied.
 *  \param b Pointer to buffer containing register data to be decoded.
 */
void ltc6804util_decodeCfg(void *g, u_int8_t *b) {
    
    struct ltc6804_cfg_reg_s *r = g;
    // test PEC
    if (checkGroupPec(b) == PEC_ERROR) {
        r->pecOk = PEC_ERROR;
    }
    // copy bytes
    r->gpio = b[0] >> 3;
    r->refon = (b[0] >> 2) & 0x01;
    r->swtrd = (b[0] >> 1) & 0x01;
    r->adcopt = b[0] & 0x01;
    r->vuv = b[1] | ((u_int16_t)(b[2] & 0x0F)) << 8;
    r->vov = (b[2] & 0xF0) >> 4 | ((u_int16_t)b[3]) << 4;
    r->dcc = b[4] | (b[5] & 0x0F) << 8;
    r->dcto = b[5] >> 4;
}

/*! Encode the '6804 Configuration Register Group, calculate data PEC.
 *  \param g Pointer to register group struct from which register data will be copied.
 *  \param b Pointer to buffer into which register data will be encoded. Must be
 *  at least 8 bytes in size.
 */
void ltc6804util_encodeCfg(u_int8_t *b, void *g) {
    
    struct ltc6804_cfg_reg_s *r = g;
    // copy bytes
    b[0] = r->gpio << 3 | (r->refon & 0x01) << 2 |
            (r->swtrd & 0x01) << 1 | (r->adcopt & 0x01);
    b[1] = (u_int8_t)r->vuv;                    // VUV lo
    b[2] = (u_int8_t)(r->vuv >> 8) |            // VUV hi
            ((u_int8_t)r->vov) << 4;            // VOV lo nibble
    b[3] = (u_int8_t)(r->vov >> 4);             // VOV hi
    b[4] = (u_int8_t)r->dcc;
    b[5] = (u_int8_t)(r->dcc >> 8) | (r->dcto & 0x0F) << 4; 
    // calculate and encode PEC
    u_int16_t pec = ltc6804util_calcPec(b, 6);  // PEC of data bytes
    b[6] = (u_int8_t)(pec >> 8);                // data pec hi
    b[7] = (u_int8_t)pec;                       // data pec lo
}

/*! Decode a '6804 Cell Voltage Register Group, check data PEC.
 *  \param g Pointer to register group struct into which register data will be copied.
 *  \param b Pointer to buffer containing register data to be decoded.
 */
void ltc6804util_decodeCV(void *g, u_int8_t *b) {
    
    struct ltc6804_cell_voltage_reg_s *r = g;
    // test PEC
    if (checkGroupPec(b) == PEC_ERROR) {
        r->pecOk = PEC_ERROR;
    }
    // copy bytes
    decodeReg(b, &r->offset[0], &r->offset[1], &r->offset[2]);
}

/*! Decode the '6804 Aux. Register Group A, check data PEC.
 *  \param g Pointer to register group struct into which register data will be copied.
 *  \param b Pointer to buffer containing register data to be decoded.
 */
void ltc6804util_decodeAuxA(void *g, u_int8_t *b) {
    
    struct ltc6804_aux_reg_a_s *r = g;
    // test PEC
    if (checkGroupPec(b) == PEC_ERROR) {
        r->pecOk = PEC_ERROR;
    }
    // copy bytes
    decodeReg(b, &r->g1v, &r->g2v, &r->g3v);
}

/*! Decode the '6804 Aux. Register Group B, check data PEC.
 *  \param g Pointer to register group struct into which register data will be copied.
 *  \param b Pointer to buffer containing register data to be decoded.
 */
void ltc6804util_decodeAuxB(void *g, u_int8_t *b) {
    
    struct ltc6804_aux_reg_b_s *r = g;
    // test PEC
    if (checkGroupPec(b) == PEC_ERROR) {
        r->pecOk = PEC_ERROR;
    }
    // copy bytes
    decodeReg(b, &r->g4v, &r->g5v, &r->ref);
}

