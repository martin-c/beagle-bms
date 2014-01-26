/*! \file 
 * File:        beagle_spi.h
 * Author:      Martin Clemons
 * Created:     August 5, 2013
 */
#pragma once

#include <sys/types.h>



/***                Definitions                 ***/

/***          Public Global Variables           ***/

/***              Public Functions              ***/

int spiInit(int *fd, const char *interface, u_int32_t speed);
int spiIo(int fd, u_int8_t *tx, u_int8_t *rx, unsigned int bytes);
void spiClose(int *fd);

