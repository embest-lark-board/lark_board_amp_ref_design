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

.globl lock_code_data_section_to_L2
.globl LockCodeData

.extern LockL2Ways

.equ L2X0_LOCKDOWN_WAY_D_BASE, 	0x900

lock_code_data_section_to_L2:
	PUSH {r0-r11, lr}
	CPSID i                      @Disable IRQ	
	BL		LockL2Ways
	CPSIE       i                @ Enable IRQ
	POP {r0-r11, lr}
	MOV pc,lr

LockCodeData:
	@r0 hold addr to be lock, r1 hold sizes
	@size must be cache size align
	@r2 hold L2Ctrl reg base, r3 offset
	@unlock way1, way3
	@LDR r2, =SOCFPGA_MPUL2_ADDRESS
	MOVW r2, #0xf000
	MOVT r2, #0xfffe
	MOV r3, #L2X0_LOCKDOWN_WAY_D_BASE
	ADD r3, #8
	MOV r4, #0xf5
	STR r4, [r2, r3]
	ADD r3, #4
	STR r4, [r2, r3]
	MOV r3, #0
1:
	LDR r4, [r0,r3]
	ADD r3,r3,#32
	CMP r1,r3
	bne 1b
	@lock way1, way3
	MOV r3, #L2X0_LOCKDOWN_WAY_D_BASE
	ADD r3, #8
	MOV r4, #0x5f
	STR r4, [r2, r3]
	ADD r3, #4
	STR r4, [r2, r3]
	MOV pc, lr