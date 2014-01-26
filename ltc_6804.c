
/*! \file 
 * File:        ltc_6804.c
 * Author:      martin
 * Created:     July 27, 2013, 1:39 AM
 * 
 * A basic driver for the LTC6804 battery stack monitor using SPI communication.
 * Created for Extended Lab, ECEN5623 Summer 2013.
 * (C) Martin Clemons 2013.
 * 
 */

#include "ltc_6804.h"
#include "ltc6804_regs.h"
#include "beagle_spi.h"
#include "ltc6804_util.h"
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
//#include <stdbool.h>
//#include <fcntl.h>
#include <string.h>

//#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>

//#include <linux/types.h>
//#include <linux/spi/spidev.h>



/***                Definitions                 ***/

// SPI tx/rx buffer size
#define SPI_BUFFER_SIZE         16



/***         Private Global Variables           ***/

// SPI file descriptor
static int spiFd;

u_int8_t txB[SPI_BUFFER_SIZE];
u_int8_t rxB[SPI_BUFFER_SIZE];



/***          Public Global Variables           ***/

/***        Private Function Prototypes         ***/

static int dummyIo(int fd, unsigned int bytes);
static int sendCmd(int fd, u_int8_t address, enum ltc6804_command_codes_e cmd);
static int addressWriteCmd(int fd,
        u_int8_t address,
        enum ltc6804_command_codes_e cmd,
        void (*encode)(u_int8_t *, void *),
        void *data_s);
static int addressReadCmd(int fd,
        u_int8_t address,
        enum ltc6804_command_codes_e cmd,
        void (*decode)(void *, u_int8_t *),
        void *data_s);



/***             Private Functions              ***/

/*! Perform a dummy IO wake isoSPI devices.
 *  See '6820 datasheet fig. 15
 *  \param fd SPI bus fd.
 *  \param bytes Number of dummy bytes to send. Typically 1.
 *  \return function returns 0 on success, -1 on failure.
 */
static int dummyIo(int fd, unsigned int bytes) {
    
    u_int8_t b[bytes];          // buffer for dummy IO
    
    memset(b, 0, bytes);
    int stat = spiIo(fd, b, b, bytes);
    if (stat < 0) {
        printf("dummyIo(): ioctl() returned %d\r\n", stat);
        return -1;
    }
    return 0;
}

/*! Send a series of 2 byte commands with 2 byte PECs.
 *  \param fd SPI bus fd.
 *  \param address Array of addresses of device on isoSPI bus (LTC6804-2).
 *  \param cmd Array of commands to send.
 *  \param cnt Number of commands to send.
 *  \return returns 0 on success, -1 on failure.
 */
static int sendCmd(int fd, u_int8_t address, enum ltc6804_command_codes_e cmd) {
    
    // assemble TX buffer
    if (address > 0x0F) {
        printf("sendCmd() with invalid 6804-2 address:%u\r\n", (unsigned int)address);
        return -1;
    }
    ltc6804util_encodeCommand(address, cmd, txB);
    // perform IO
    int stat = spiIo(fd, txB, rxB, 4);
    if (stat < 0) {
        printf("sendCmd(): ioctl() returned %d\r\n", stat);
        return -1;
    }
    return 0;
}

/*! Perform an Address Write Command
 *  For more information see datasheet table 29.
 *  \param fd SPI bus fd.
 *  \param address Array of addresses of device on isoSPI bus (LTC6804-2).
 *  \param cmd Array of commands to send.
 *  \param encode Array of functions to encode register group structs.
 *  \param data_s Array of struct parameters for encode function.
 *  \param cnt Number of commands to send.
 *  \return returns 0 on success, -1 on failure.
 */
static int addressWriteCmd(int fd,
        u_int8_t address,
        enum ltc6804_command_codes_e cmd,
        void (*encode)(u_int8_t *, void *),
        void *data_s) {
    
    // 2 cmd bytes, 2 PEC bytes, 6 reg. group bytes, 2 PEC bytes required in buffer.
    if (encode == NULL) {
        printf("addressWriteCmd() with NULL encode function.\r\n");
        return -1;
    }
    if (data_s == NULL) {
        printf("addressWriteCmd() with NULL data_s parameter.\r\n");
        return -1;
    }
    // assemble TX buffer from array of commands
    if (address > 0x0F) {
        printf("addressWriteCmd() with invalid 6804-2 address: %u\r\n", (unsigned int)address);
        return -1;
    }
    // put cmd and PEC in first 4 bytes of current buffer offset
    ltc6804util_encodeCommand(address, cmd, txB);
    // put data and data PEC in 8 bytes following
    encode(&txB[4], data_s);
    // perform IO
    int stat = spiIo(fd, txB, rxB, 12);
    if (stat < 0) {
        printf("addressWriteCmd(): ioctl() returned %d\r\n", stat);
        return -1;
    }
    return 0;
}

/*! Perform an Address Read Command
 *  For more information see datasheet table 31.
 *  \param fd SPI bus fd.
 *  \param address Array of addresses of device on isoSPI bus (LTC6804-2).
 *  \param cmd Array of commands to send.
 *  \param encode Array of functions to decode received register group structs.
 *  \param data_s Array of struct parameters for decode function.
 *  \param cnt Number of commands to send.
 *  \return returns 0 on success, -1 on failure.
 */
static int addressReadCmd(int fd,
        u_int8_t address,
        enum ltc6804_command_codes_e cmd,
        void (*decode)(void *, u_int8_t *),
        void *data_s) {
    
    // 2 cmd bytes, 2 PEC bytes, 6 reg. group bytes, 2 PEC bytes
    if (decode == NULL) {
        printf("addressReadCmd() with NULL decode function.\r\n");
        return -1;
    }
    if (data_s == NULL) {
        printf("addressReadCmd() with NULL data_s parameter.\r\n");
        return -1;
    }
    // assemble TX buffer
    memset(txB, 0, 12);
    if (address > 0x0F) {
        printf("addressReadCmd() with invalid 6804-2 address: %u\r\n", (unsigned int)address);
        return -1;
    }
    // put cmd and PEC in first 4 bytes of current buffer offset
    ltc6804util_encodeCommand(address, cmd, txB);
    // perform IO
    int stat = spiIo(fd, txB, rxB, 12);
    if (stat < 0) {
        printf("addressWriteCmd(): ioctl() returned %d\r\n", stat);
        return -1;
    }
    // decode data and data PEC in 8 bytes following 4 dummy command bytes.
    decode(data_s, &rxB[4]);
    return 0;
}



/***              Public Functions              ***/

/*! Initialize LTC6804
 *  \param address Address of LTC6804-2 IC.
 *  \param String with name of SPI interface. /dev/spidevx.y for example.
 *  \param SPI_speed SPI clock frequency in Hertz.
 *  \return Function returns 0 on success, -1 on failure.
 */
int ltc6804_init(u_int8_t address, const char *SPI_interface, u_int32_t SPI_speed) {
    
    // only do the below once regardless of '6804 address.
    if (spiFd == 0) {
        // initialize PEC pre-calculated table
        ltc6804util_initPec();
        int stat = spiInit(&spiFd, SPI_interface, SPI_speed);
        if (stat < 0) {
            perror("spiInit");
            return -1;
        }
    }
    dummyIo(spiFd, 1);          // wake SPI interface IC, '6804.
    usleep(500);                // '6804 t_wake = max 300us.
    // perform MUX diagnostic
    ltc6804_startMuxDiag(address);
    usleep(5000);
    // set configuration registers, GPIO pins
    struct ltc6804_cfg_reg_s cfg = {
        .gpio = 0x01,                   // all GPIO pull-downs on except for pin 1
        .refon = 1,                     // leave reference on between conversions
        .swtrd = 0,                     // no effect
        .adcopt = 0,                    // ADC "standard" modes
        .vuv = 1874,                    // VUV = 3.0V
        .vov = 2561,                    // VOV 4.1V
        .dcc = 0,                       // no cell discharge
        .dcto = DCTO_DISABLED_gc,       // disable DCTO
    };
    ltc6804_writeConfig(address, &cfg);
    return 1;
}

/*! Close interface to LTC6804 devices
 */
void ltc6804_close(void) {
    
    spiClose(&spiFd);
}

/*! Read LTC6804 status register group A.
 *  \param address Address of LTC6804-2 IC.
 *  \param g Pointer to register group struct into which data will be copied.
 *  \return Function returns 0 on success, -1 on failure.
 */
int ltc6804_readStatA(u_int8_t address, struct ltc6804_stat_reg_a_s *g) {
    
    // send dummy bit to wakeup '6820 and '6804, 
    dummyIo(spiFd, 1);
    /* Note: We assume here that the operations before next SPI io will
     * take at least 2 * t_ready and therefore we don't explicitly
     * insert a delay here.
     */
    // read register group    
    g->pecOk = PEC_OK;
    int s = addressReadCmd(spiFd, address, CMD_RDSTATA, ltc6804util_decodeStatA, g);
    if (s < 0) {
        perror("addressReadCmd");
        return -1;
    }
    return 0;
}

/*! Read LTC6804 status register group B.
 *  \param address Address of LTC6804-2 IC.
 *  \param g Pointer to register group struct into which data will be copied.
 *  \return Function returns 0 on success, -1 on failure.
 */
int ltc6804_readStatB(u_int8_t address, struct ltc6804_stat_reg_b_s *g) {
    
    // send dummy bit to wakeup '6820 and '6804, 
    dummyIo(spiFd, 1);
    /* Note: We assume here that the operations before next SPI io will
     * take at least 2 * t_ready and therefore we don't explicitly
     * insert a delay here.
     */
    // read register group    
    g->pecOk = PEC_OK;
    int s = addressReadCmd(spiFd, address, CMD_RDSTATB, ltc6804util_decodeStatB, g);
    if (s < 0) {
        perror("addressReadCmd");
        return -1;
    }
    return 0;
}

/*! Read LTC6804 configuration register group.
 *  \param address Address of LTC6804-2 IC.
 *  \param g Pointer to register group struct into which data will be copied.
 *  \return Function returns 0 on success, -1 on failure.
 */
int ltc6804_readConfig(u_int8_t address, struct ltc6804_cfg_reg_s *g) {
    
    // send dummy bit to wakeup '6820 and '6804, 
    dummyIo(spiFd, 1);
    /* Note: We assume here that the operations before next SPI io will
     * take at least 2 * t_ready and therefore we don't explicitly
     * insert a delay here.
     */
    // read register group
    g->pecOk = PEC_OK;
    int s = addressReadCmd(spiFd, address, CMD_RDCFG, ltc6804util_decodeCfg, g);
    if (s < 0) {
        perror("addressReadCmd");
        return -1;
    }
    return 0;
}

/*! Write LTC6804 configuration register group.
 *  \param address Address of LTC6804-2 IC.
 *  \param g Pointer to register group struct into which data will be copied.
 *  \return Function returns 0 on success, -1 on failure.
 */
int ltc6804_writeConfig(u_int8_t address, struct ltc6804_cfg_reg_s *g) {
    
    // send dummy bit to wakeup '6820 and '6804, 
    dummyIo(spiFd, 1);
    /* Note: We assume here that the operations before next SPI io will
     * take at least 2 * t_ready and therefore we don't explicitly
     * insert a delay here.
     */
    // write register group
    g->pecOk = PEC_OK;
    int s = addressWriteCmd(spiFd, address, CMD_WRCFG, ltc6804util_encodeCfg, g);
    if (s < 0) {
        perror("addressReadCmd");
        return -1;
    }
    return 0;
}

/*! Read LTC6804 cell voltage register group.
 *  \param address Address of LTC6804-2 IC.
 *  \param g Pointer to register group struct into which data will be copied.
 *  \param group Cell register group 'A' through 'D'.
 *  \return Function returns 0 on success, -1 on failure.
 */
int ltc6804_readCellVoltageGroup(u_int8_t address, char group, struct ltc6804_cell_voltage_reg_s *g) {
  
    // send dummy bit to wakeup '6820 and '6804, 
    dummyIo(spiFd, 1);
    /* Note: We assume here that the operations before next SPI io will
     * take at least 2 * t_ready and therefore we don't explicitly
     * insert a delay here.
     */
    enum ltc6804_command_codes_e cmd;
    // set command
    switch (group) {
        case 'A':
            cmd = CMD_RDCVA;
            break;
        case 'B':
            cmd = CMD_RDCVB;
            break;
        case 'C':
            cmd = CMD_RDCVC;
            break;
        case 'D':
            cmd = CMD_RDCVD;
            break;
        default:
            printf("ltc6804_readCellVoltageGroup(): cannot read from group %c.\r\n", (int)group);
            return -1;
    }
    // read register group
    g->pecOk = PEC_OK;
    int s = addressReadCmd(spiFd, address, cmd, ltc6804util_decodeCV, g);
    if (s < 0) {
        perror("addressReadCmd");
        return -1;
    }
    return 0;
}

/*! Read all 4 LTC6804 cell voltage register group.
 *  \param address Address of LTC6804-2 IC.
 *  \param c Pointer to cell voltage struct into which data will be copied.
 *  Note: cell 1 voltage is stored in c.cell[0], cell 12 voltage in
 *  c.cell[11].
 *  \param group Cell register group 'A' through 'D'.
 *  \return Function returns 0 on success, -1 on failure.
 */
int ltc6804_readCellVoltageGroups(u_int8_t address, struct ltc6804_cell_voltage_s *c) {
    
    // send dummy bit to wakeup '6820 and '6804, 
    dummyIo(spiFd, 1);
    /* Note: We assume here that the operations before next SPI io will
     * take at least 2 * t_ready and therefore we don't explicitly
     * insert a delay here.
     */
    enum ltc6804_command_codes_e cmd[] = {CMD_RDCVA, CMD_RDCVB, CMD_RDCVC, CMD_RDCVD};
    struct ltc6804_cell_voltage_reg_s g;
    int i;
    int j=0;
    c->pecOk = PEC_OK;
    for (i=0; i<4; i++) {
        g.pecOk = PEC_OK;
        // read register groups
        int s = addressReadCmd(spiFd, address, cmd[i], ltc6804util_decodeCV, &g);
        if (s < 0) {
            perror("addressReadCmd");
            return -1;
        }
        c->cell[j++] = g.offset[0];
        c->cell[j++] = g.offset[1];
        c->cell[j++] = g.offset[2];
        c->pecOk &= g.pecOk;            // if any PEC failed, clear PEC.
    }
    return 0;
}

/*! Read LTC6804 auxiliary register group A.
 *  \param address Address of LTC6804-2 IC.
 *  \param g Pointer to register group struct into which data will be copied.
 *  \return Function returns 0 on success, -1 on failure.
 */
int ltc6804_readAuxGroupA(u_int8_t address, struct ltc6804_aux_reg_a_s *g) {
    
    // send dummy bit to wakeup '6820 and '6804, 
    dummyIo(spiFd, 1);
    /* Note: We assume here that the operations before next SPI io will
     * take at least 2 * t_ready and therefore we don't explicitly
     * insert a delay here.
     */
    // read register group
    g->pecOk = PEC_OK;
    int s = addressReadCmd(spiFd, address, CMD_RDAUXA, ltc6804util_decodeAuxA, g);
    if (s < 0) {
        perror("addressReadCmd");
        return -1;
    }
    return 0;
}

/*! Read LTC6804 auxiliary register group B.
 *  \param address Address of LTC6804-2 IC.
 *  \param g Pointer to register group struct into which data will be copied.
 *  \return Function returns 0 on success, -1 on failure.
 */
int ltc6804_readAuxGroupB(u_int8_t address, struct ltc6804_aux_reg_b_s *g) {
    
    // send dummy bit to wakeup '6820 and '6804, 
    dummyIo(spiFd, 1);
    /* Note: We assume here that the operations before next SPI io will
     * take at least 2 * t_ready and therefore we don't explicitly
     * insert a delay here.
     */
    // read register group
    g->pecOk = PEC_OK;
    int s = addressReadCmd(spiFd, address, CMD_RDAUXB, ltc6804util_decodeAuxB, g);
    if (s < 0) {
        perror("addressReadCmd");
        return -1;
    }
    return 0;
}

/*! Start ADC conversion of cell voltage
 *  \param address Address of LTC6804-2 IC.
 *  \param mode ADC mode (conversion speed)
 *  \param cells Cell mask indicating which cell voltages are converted.
 *  \return Function returns 0 on success, -1 on error.
 */
int ltc6804_startCellVoltageConversion(u_int8_t address,
        enum ltc6804_ADC_modes_e mode,
        enum ltc6804_CH_e cells) {
    
    // send dummy bit to wakeup '6820 and '6804, 
    dummyIo(spiFd, 1);
    /* Note: We assume here that the operations before next SPI io will
     * take at least 2 * t_ready and therefore we don't explicitly
     * insert a delay here.
     */
    int s = sendCmd(spiFd, address, CMD_ADCV | mode | cells);
    if (s < 0) {
        perror("sendCmd");
        return -1;
    }
    return 0;
}

/*! Start ADC conversion of GPIO voltage
 *  \param address Address of LTC6804-2 IC.
 *  \param mode ADC mode (conversion speed)
 *  \param chans Channel mask indicating which GPIO voltages are converted.
 *  \return Function returns 0 on success, -1 on error.
 */
int ltc6804_startGpioVoltageConversion(u_int8_t address,
        enum ltc6804_ADC_modes_e mode,
        enum ltc6804_CHG_e chans) {
    
    // send dummy bit to wakeup '6820 and '6804, 
    dummyIo(spiFd, 1);
    /* Note: We assume here that the operations before next SPI io will
     * take at least 2 * t_ready and therefore we don't explicitly
     * insert a delay here.
     */
    int s = sendCmd(spiFd, address, CMD_ADAX | mode | chans);
    if (s < 0) {
        perror("sendCmd");
        return -1;
    }
    return 0;
}

/*! Start ADC conversion of status group voltages
 *  \param address Address of LTC6804-2 IC.
 *  \param mode ADC mode (conversion speed).
 *  \param chans Channel mask indicating which status group voltages are converted.
 *  \return Function returns 0 on success, -1 on error.
 */
int ltc6804_startStatVoltageConversion(u_int8_t address,
        enum ltc6804_ADC_modes_e mode,
        enum ltc6804_CHST_e chans) {
    
    // send dummy bit to wakeup '6820 and '6804, 
    dummyIo(spiFd, 1);
    /* Note: We assume here that the operations before next SPI io will
     * take at least 2 * t_ready and therefore we don't explicitly
     * insert a delay here.
     */
    int s = sendCmd(spiFd, address, CMD_ADSTAT | mode | chans);
    if (s < 0) {
        perror("sendCmd");
        return -1;
    }
    return 0;
}

/*! Start MUX diagnostic.
 *  \param param address Address of LTC6804-2 IC.
 *  \return Function returns 0 on success, -1 on error.
 */
int ltc6804_startMuxDiag(u_int8_t address) {
    
    // send dummy bit to wakeup '6820 and '6804, 
    dummyIo(spiFd, 1);
    /* Note: We assume here that the operations before next SPI io will
     * take at least 2 * t_ready and therefore we don't explicitly
     * insert a delay here.
     */
    int s = sendCmd(spiFd, address, CMD_DIAGN);
    if (s < 0) {
        perror("sendCmd");
        return -1;
    }
    return 0;
}

/*! Start ADC conversion of cell voltage and GPIO1, GPIO2
 *  \param address Address of LTC6804-2 IC.
 *  \param mode ADC mode (conversion speed).
 *  \return Function returns 0 on success, -1 on error.
 */
int ltc6804_startCellGpioVoltageConversion(u_int8_t address,
        enum ltc6804_ADC_modes_e mode) {
    
    // send dummy bit to wakeup '6820 and '6804, 
    dummyIo(spiFd, 1);
    /* Note: We assume here that the operations before next SPI io will
     * take at least 2 * t_ready and therefore we don't explicitly
     * insert a delay here.
     */
    int s = sendCmd(spiFd, address, CMD_ADCVAX | mode);
    if (s < 0) {
        perror("sendCmd");
        return -1;
    }
    return 0;
}