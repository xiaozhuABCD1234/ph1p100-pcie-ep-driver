/*
 * This file is part of the PCIe driver tools for Linux
 *
 * Copyright (c) 2020-present,  anlogic, Inc.
 * All rights reserved.
 *
 * This source code is licensed under BSD-style license (found in the
 * LICENSE file in the root directory of this source tree)
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <byteswap.h>
#include <string.h>
#include <errno.h>
#include <signal.h>
#include <fcntl.h>
#include <ctype.h>
#include <termios.h>
#include <libgen.h>

#include <sys/types.h>
#include <sys/mman.h>
#include <sys/ioctl.h>

#include <stdio.h>
#include <stdlib.h>

#include "cdev_ctrl.h"

#define FATAL do { fprintf(stderr, "Error at line %d, file %s (%d) [%s]\n", __LINE__, __FILE__, errno, strerror(errno)); exit(1); } while(0)
    


int reg_write(int fd,uint32_t bar_id, uint32_t offset, uint32_t wdata)
{
    struct anlogic_ioc_bar gBar;
    int ret;

    gBar.bar_id = bar_id;
    gBar.bar_type=0;
    gBar.bar_offaddr=offset;
    gBar.wdata=wdata;
    gBar.rdata=0;
    
    ret = ioctl(fd,ANLOGIC_IOCW,&gBar);
    
    return ret;
}

int  reg_read(int fd,uint32_t bar_id,uint32_t offset, uint32_t *rdata)
{
    struct anlogic_ioc_bar gBar;
    int ret;
    gBar.bar_id = bar_id;
    gBar.bar_type=0;
    gBar.bar_offaddr=offset;
    gBar.wdata=0;
    gBar.rdata=0;
    ret = ioctl(fd,ANLOGIC_IOCR,&gBar);
   // printf("read data: 0x%x\n", gBar.rdata);
    
    *rdata = gBar.rdata;
    return ret;
}


char pCtrlDev[]="/dev/ANLOGIC-PCI0_control";

int main(int argc, char **argv)
{
    int fd;
	uint32_t bar_id=0;
	uint32_t offset=0;
    uint32_t wrdata;
	unsigned int reg_value = 0;
	
	uint32_t mode = 0;

    
    bar_id =  strtoul(argv[1], 0, 0);
    
	if (bar_id >= 6) {
		 printf("Invalid bar ID, must be 0..5.\n");
		 return -1;
	}

	/* BAR1/BAR3/BAR5 have no mapping of their own: BAR1 is the I/O BAR and
	 * BAR3/BAR5 are the upper halves of the 64-bit BAR2/BAR4. The driver
	 * leaves xdev->bar[] NULL for them, so refuse them here (audit rule 15). */
	if (bar_id == 1 || bar_id == 3 || bar_id == 5) {
		 printf("BAR%d is not independently mapped (no dedicated resource).\n",
				bar_id);
		 return -1;
	}
	 
	 offset =  strtoul(argv[2], 0, 0);
    
	if (offset % 4 != 0) {
		 printf("Offset must be 4byte aligned.\n");
		 return -1;
	}
    	
	
	if (argc ==4) {
	   wrdata =  strtoul(argv[3], 0, 0);
	   printf("Pci bar%d: address: 0x%x write data: 0x%x.\n",bar_id,offset,wrdata); 
	   mode = 1;
	}
	else if (argc == 3) {
	  mode = 2;
	  printf("Pci bar%d: address: 0x%x read.\n",bar_id,offset); 
	}
	else {
		 printf("Input parameter error.\n");
		 return -1;
	}
	   
    
    if ((fd = open(pCtrlDev, O_RDWR | O_SYNC)) == -1)  //open elbi device
        FATAL;
    printf("character device %s opened.\n", argv[1]);
    fflush(stdout);
	
	
    if (mode == 1) {
		reg_write(fd,bar_id, offset, wrdata);
		printf("Write complete.\n");
	}
	 
	 
   if (mode == 2) {
	   reg_read(fd, bar_id,offset, &reg_value);
	   printf("Read data: 0x%x complete.\n",reg_value);
   }
    
	close(fd);

    return 0;
}
