
/*! \file 
 * File:        beagle_spi.c
 * Author:      martin
 * Created:     August 5, 2013
 *
 * Utility functions to initialize, perform IO, and close the SPI interface to
 * LTC6804 on the Beagle xM using spidev driver.
 *  
 */

#include "beagle_spi.h"
#include <stdio.h>
#include <unistd.h>
//#include <stdlib.h>
#include <stdbool.h>
#include <fcntl.h>
//#include <string.h>

#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <linux/types.h>
#include <linux/spi/spidev.h>



/***                Definitions                 ***/

/***         Private Global Variables           ***/

/***          Public Global Variables           ***/

/***        Private Function Prototypes         ***/

/***             Private Functions              ***/

/***               Public Functions             ***/

/*! Initialize SPI interface.
 *  \param fd Pointer to file descriptor to initialize and use for SPI interface.
 *  \param interface String pointer with spidev SPI device to use for SPI interface.
 *  \param speed SPI interface clock speed in Hz.
 *  \return Function returns 0 on success, -1 on failure.
 */
int spiInit(int *fd, const char *interface, u_int32_t speed) {
    
    // open SPI FD
    *fd = open(interface, O_RDWR);
    if (*fd == 0) {
        perror("open");
        return -1;
    }
    // configure SPI interface
    u_int8_t mode = SPI_MODE_3;
    u_int8_t byteOrder = 0;     // MSB first
    u_int8_t bitsPerWord = 0;   // 8 bits per word
    if ( ioctl(*fd, SPI_IOC_WR_MODE, &mode) < 0 ) {
        perror("SPI_IOC_WR_MODE");
        return -1;
    }
    if ( ioctl(*fd, SPI_IOC_WR_LSB_FIRST, &byteOrder) < 0 ) {
        perror("SPI_IOC_WR_LSB_FIRST");
        return -1;
    }
    if ( ioctl(*fd, SPI_IOC_WR_BITS_PER_WORD, &bitsPerWord) < 0 ) {
        perror("SPI_IOC_WR_BITS_PER_WORD");
        return -1;
    }
    if ( ioctl(*fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0 ) {
        perror("SPI_IOC_WR_MAX_SPEED_HZ");
        return -1;
    }
    return 0;
}

/*! Perform SPI I/O
 *  \param fd Opened SPI File Descriptor.
 *  \param tx Pointer to SPI transmit buffer.
 *  \param rx Pointer to SPI receive buffer.
 *  \param bytes Number of bytes to send/receive over SPI.
 *  \return Function returns ioctl() return code.
 */
int spiIo(int fd, u_int8_t *tx, u_int8_t *rx, unsigned int bytes) {
    
    if (fd == 0) {
        perror("spiIo");
        return -1;
    }
    
    struct spi_ioc_transfer xfer = {
        .tx_buf = (intptr_t)tx,
        .rx_buf = (intptr_t)rx,
        .len = (__u32)bytes,
        .speed_hz = 0,          // default
        .delay_usecs = 10,      // delay 10us between transfers
        .bits_per_word = 0,     // default
        .cs_change = true,      // deselect CS between transfers
    };       // current SPI transfer
    
    int status = ioctl(fd, SPI_IOC_MESSAGE(1), &xfer);
    if (status < 0) {
        perror("ioctl, SPI_IOC_MESSAGE(1)");
    }
    return status;
}

/*! Close SPI interface
 *  \param fd Pointer to SPI file descriptor previously opened.
 */
void spiClose(int *fd) {
    
    if (*fd != 0) {
        close(*fd);
    }
}
