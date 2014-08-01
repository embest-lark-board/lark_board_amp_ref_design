/*
 *
 * Copyright (C) 2014 Embest Technology Co., Ltd. <http://www.embest-tech.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  - Neither the name of Embest Technology Co., Ltd. nor the
 *    names of its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS", 
 * it is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 */


#ifndef FPGA_REGS_H
#define FPGA_REGS_H

#define FPGA_LWAXI_BASE				(0xff200000)

//interrupt latency test ip
#define FPGA_INT_LATENCY_IP_OFFSET		(0x12800)
#define FPGA_INT_LATENCY_IP_BASE		(FPGA_LWAXI_BASE + FPGA_INT_LATENCY_IP_OFFSET)
#define FPGA_INT_LATENCY_IP_SOFT_RESET	FPGA_INT_LATENCY_IP_BASE
#define FPGA_INT_LATENCY_IP_CYCLE		(FPGA_INT_LATENCY_IP_BASE + (4<<2))
#define FPGA_INT_LATENCY_IP_START		(FPGA_INT_LATENCY_IP_BASE + (8<<2))
#define FPGA_INT_LATENCY_IP_ACK			(FPGA_INT_LATENCY_IP_BASE + (0xc<<2))
#define FPGA_INT_LATENCY_IP_CYCLE_COUNT	(FPGA_INT_LATENCY_IP_BASE + (0x28<<2))
#define FPGA_INT_LATENCY_IP_REQ_COUNT	(FPGA_INT_LATENCY_IP_BASE + (0x18<<2))
#define FPGA_INT_LATENCY_IP_ACK_COUNT	(FPGA_INT_LATENCY_IP_BASE + (0x24<<2))






#endif

