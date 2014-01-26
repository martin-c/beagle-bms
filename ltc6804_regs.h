/*! \file 
 * File:        ltc6804_regs.h
 * Author:      Martin Clemons
 * Created:     August 5, 2013
 * 
 * LTC6804 register bit definitions.
 * 
 */
#pragma once

#include <sys/types.h>



/***                Definitions                 ***/

// number of battery cells per '6804 IC
#define     CELL_COUNT_PER_IC   12



/***          Public Global Variables           ***/

/*! LTC6804 command codes. Datasheet table 34.
 */
enum ltc6804_command_codes_e {
    CMD_WRCFG   = 0x001,
    CMD_RDCFG   = 0x002,
    CMD_RDCVA   = 0x004,
    CMD_RDCVB   = 0x006,
    CMD_RDCVC   = 0x008,
    CMD_RDCVD   = 0x00A,
    CMD_RDAUXA  = 0x00C,
    CMD_RDAUXB  = 0x00E,
    CMD_RDSTATA = 0x010,
    CMD_RDSTATB = 0x012,
    CMD_ADCV    = 0x260,
    CMD_ADOW    = 0x228,
    CMD_CVST    = 0x207,
    CMD_ADAX    = 0x460,
    CMD_AXST    = 0x407,
    CMD_ADSTAT  = 0x468,
    CMD_STATST  = 0x40F,
    CMD_ADCVAX  = 0x46F,
    CMD_CLRCELL = 0x711,
    CMD_CLRAUX  = 0x712,
    CMD_CLRSTAT = 0x713,
    CMD_PLADC   = 0x714,
    CMD_DIAGN   = 0x715,
    CMD_WRCOMM  = 0x721,
    CMD_RDCOMM  = 0x722,
    CMD_STCOMM  = 0x723,
};

/*! LTC6804 Discharge permitted bits
 */
enum ltc6804_DCP_e {
    DCP_DSCHG_NOT_PERMITTED_gc  = 0x0 << 4,
    DCP_DSCHG_PERMITTED_gc      = 0x1 << 4,
};

/*! LTC6804 pull up/pull down current for open-wire conversions
 */
enum ltc6804_PUP_e {
    PUP_PULL_DN_gc              = 0x0 << 6,
    PUP_PULL_UP_gc              = 0x1 << 6,
};

/*! LTC6804 Self-test mode selection
 */
enum ltc6804_ST_e {
    ST_SELF_TEST_1_gc           = 0x1 << 5,
    ST_SELF_TEST_2_gc           = 0x2 << 5,
};

/*! LTC6804 Self test 1 results */
enum ltc6804_ST1_res_e {
    ST1_RES_27k                 = 0x9565,
    ST1_RES_14k                 = 0x9553,
    ST1_RES_7k                  = 0x9555,
    ST1_RES_3k                  = 0x9555,
    ST1_RES_2k                  = 0x9555,
    ST1_RES_26                  = 0x9555,
};

/*! LTC6804 Self test 2 results */
enum ltc6804_ST2_res_e {
    ST2_RES_27k                 = 0x6A9A,
    ST2_RES_14k                 = 0x6AAC,
    ST2_RES_7k                  = 0x6AAA,
    ST2_RES_3k                  = 0x6AAA,
    ST2_RES_2k                  = 0x6AAA,
    ST2_RES_26                  = 0x6AAA,
};

/*! LTC6804 Discharge Timeout values
 */
enum ltc6804_DCTO_e {
    DCTO_DISABLED_gc            = 0x0,
    DCTO_30_SEC_gc              = 0x1,
    DCTO_1_MIN_gc               = 0x2,
    DCTO_2_MIN_gc               = 0x3,
    DCTO_3_MIN_gc               = 0x4,
    DCTO_4_MIN_gc               = 0x5,
    DCTO_5_MIN_gc               = 0x6,
    DCTO_10_MIN_gc              = 0x7,
    DCTO_15_MIN_gc              = 0x8,
    DCTO_20_MIN_gc              = 0x9,
    DCTO_30_MIN_gc              = 0xA,
    DCTO_40_MIN_gc              = 0xB,
    DCTO_60_MIN_gc              = 0xC,
    DCTO_75_MIN_gc              = 0xD,
    DCTO_90_MIN_gc              = 0xE,
    DCTO_120_MIN_gc             = 0xF,
};

/*! LTC6804 Cell selection for ADC conversion
 */
enum ltc6804_CH_e {
    CH_ALL_CELLS_gc             = 0x0,
    CH_CELL_1_7_gc              = 0x1,
    CH_CELL_2_8_gc              = 0x2,
    CH_CELL_3_9_gc              = 0x3,
    CH_CELL_4_10_gc             = 0x4,
    CH_CELL_5_11_gc             = 0x5,
    CH_CELL_6_12_gc             = 0x6,
};

/*! LTC6804  ADC Modes
 */
enum ltc6804_ADC_modes_e {
    ADC_MD_FAST_gc      = 0x001 << 7,
    ADC_MD_NORM_gc      = 0x002 << 7,
    ADC_MD_FILTERED_gc  = 0x003 << 7,
};

/*! LTC6804 GPIO selection for ADC conversion
 */
enum ltc6804_CHG_e {
    CHG_GPIO_ALL_gc             = 0x0,
    CHG_GPIO_1_gc               = 0x1,
    CHG_GPIO_2_gc               = 0x2,
    CHG_GPIO_3_gc               = 0x3,
    CHG_GPIO_4_gc               = 0x4,
    CHG_GPIO_5_gc               = 0x5,
    CHG_GPIO_REF_2_gc           = 0x6,
};

/*! LTC6804 Status group selection for ADC conversion
 */
enum ltc6804_CHST_e {
    CHST_ALL_gc                 = 0x0,
    CHST_SOC_gc                 = 0x1,
    CHST_ITMP_gc                = 0x2,
    CHST_VA_gc                  = 0x3,
    CHST_VD_gc                  = 0x4,
};

/*! LTC6804 PEC status for received data
 */
enum ltc6804_PEC_e {
    PEC_ERROR   = 0,
    PEC_OK      = 1,
};

/*! LTC6804 Configuration Registers
 */
struct ltc6804_cfg_reg_s {
    u_int8_t            gpio;   ///< GPIO Pin Control (bit 0->pin1, bit 4->pin5)
    u_int8_t            refon;  ///< Reference Powered Up
    u_int8_t            swtrd;  ///< SWTEN Pin Status
    u_int8_t            adcopt; ///< ADC Mode Option Bit
    u_int16_t           vuv;    ///< Undervoltage Comparison Voltage
    u_int16_t           vov;    ///< Overvoltage Comparison Voltage
    u_int16_t           dcc;    ///< Discharge Cell x flag (1-12) 
    enum ltc6804_DCTO_e dcto;   ///< discharge timeout value bits
    enum ltc6804_PEC_e  pecOk;  ///< PEC status for received register data
};

/*! LTC6804 Cell Voltage Registers
 *  Note: This is a generic data type for register groups A-D
 */
struct ltc6804_cell_voltage_reg_s {
    u_int16_t           offset[3];      ///< Group offset 0-3
    enum ltc6804_PEC_e  pecOk;          ///< PEC status for received register data
};

/*! LTC6804 Cell Voltages for all register groups
 *  Note: Cell array starts with cell[0] which represents '6804 cell 1.
 *  cell[11] represents 6804 cell 12.
 */
struct ltc6804_cell_voltage_s {
    u_int16_t           cell[CELL_COUNT_PER_IC];    ///< Cells 0-11 (1-12 on '6804)
    enum ltc6804_PEC_e  pecOk;                      ///< PEC status for received register data
};

/*! LTC6804 Auxiliary Register Group A
 */
struct ltc6804_aux_reg_a_s {
    u_int16_t           g1v;            ///< GPIO 1 Voltage
    u_int16_t           g2v;            ///< GPIO 2 Voltage
    u_int16_t           g3v;            ///< GPIO 3 Voltage
    enum ltc6804_PEC_e  pecOk;          ///< PEC status for received register data
};

/*! LTC6804 Auxiliary Register Group B
 */
struct ltc6804_aux_reg_b_s {
    u_int16_t           g4v;            ///< GPIO 4 Voltage
    u_int16_t           g5v;            ///< GPIO 5 Voltage
    u_int16_t           ref;            ///< Second reference voltage
    enum ltc6804_PEC_e  pecOk;          ///< PEC status for received register data
};

/*! LTC6804 Status Register Group A
 */
struct ltc6804_stat_reg_a_s {
    u_int16_t           soc;            ///< Sum of Cells Measurement
    u_int16_t           itmp;           ///< Internal IC Die Temperature
    u_int16_t           va;             ///< Analog Power Supply Voltage
    enum ltc6804_PEC_e  pecOk;          ///< PEC status for received register data
};

/*! LTC6804 Cell Flags
 */
enum ltc6804_cell_flags_e {
    CF_UNDERVOLTAGE     = 0x1,          ///< Cell Undervoltage Flag
    CF_OVERVOLTAGE      = 0x2,          ///< Cell Overvoltage Flag
};

/*! LTC6804 Status Register Group B
 */
struct ltc6804_stat_reg_b_s {
    u_int16_t           vd;             ///< Digital Supply Voltage
    enum ltc6804_cell_flags_e cell_flags[CELL_COUNT_PER_IC];    ///< Cell UV and OV flags
    u_int8_t            rev;            ///< Device Revision Code
    u_int8_t            muxfail;        ///< Multiplexer Self-Test Result
    u_int8_t            thsd;           ///< Thermal Shutdown Status
    enum ltc6804_PEC_e  pecOk;          ///< PEC status for received register data
};



/***              Public Functions              ***/
