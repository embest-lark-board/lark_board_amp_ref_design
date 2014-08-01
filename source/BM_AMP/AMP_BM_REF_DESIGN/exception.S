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


    .global  CPU_ARM_ExceptUndefInstrHndlr
    .global  CPU_ARM_ExceptSwiHndlr
    .global  CPU_ARM_ExceptPrefetchAbortHndlr
    .global  CPU_ARM_ExceptDataAbortHndlr
    .global  CPU_ARM_ExceptIrqHndlr
    .global  CPU_ARM_ExceptFiqHndlr

    .extern identify_and_clear_source
    .extern C_irq_handler


     .equ    CPU_ARM_CTRL_INT_DIS,   0xC0                            @ Disable both FIQ & IRQ
     .equ    CPU_ARM_CTRL_FIQ_DIS,   0x40                            @ Disable FIQ.
     .equ    CPU_ARM_CTRL_IRQ_DIS,   0x80                            @ Disable IRQ.

     .equ    CPU_ARM_BIT_CNTES_C,    0x80000000                      @ Cycle counter enable set.
     .equ    CPU_ARM_BIT_PMNC_E,     0x00000001                      @ All counter enable bits.
     .equ    CPU_ARM_BIT_PMNC_P,     0x00000002                      @ Event counter reset.
     .equ    CPU_ARM_BIT_PMNC_C,     0x00000004                      @ Cycle counter reset.
     .equ    CPU_ARM_BIT_PMNC_RST,   0x00000006                      @ Reset all counters.

     .equ     OS_CPU_ARM_CONTROL_INT_DIS,        0xC0                    @ Disable both FIQ and IRQ.
    .equ     OS_CPU_ARM_CONTROL_FIQ_DIS,        0x40                    @ Disable FIQ.
    .equ     OS_CPU_ARM_CONTROL_IRQ_DIS,        0x80                    @ Disable IRQ.
    .equ     OS_CPU_ARM_CONTROL_THUMB,          0x20                    @ Set THUMB mode.
    .equ     OS_CPU_ARM_CONTROL_ARM,            0x00                    @ Set ARM mode.

    .equ     OS_CPU_ARM_MODE_MASK,              0x1F
    .equ     OS_CPU_ARM_MODE_USR,               0x10
    .equ     OS_CPU_ARM_MODE_FIQ,               0x11
    .equ     OS_CPU_ARM_MODE_IRQ,               0x12
    .equ     OS_CPU_ARM_MODE_SVC,               0x13
    .equ     OS_CPU_ARM_MODE_ABT,               0x17
    .equ     OS_CPU_ARM_MODE_UND,               0x1B
    .equ     OS_CPU_ARM_MODE_SYS,               0x1F

    .equ     OS_CPU_ARM_EXCEPT_RST,             0x00
    .equ     OS_CPU_ARM_EXCEPT_UND,             0x01
    .equ     OS_CPU_ARM_EXCEPT_SWI,             0x02
    .equ     OS_CPU_ARM_EXCEPT_ABORT_PREFETCH,  0x03
    .equ     OS_CPU_ARM_EXCEPT_ABORT_DATA,      0x04
    .equ     OS_CPU_ARM_EXCEPT_RSVD,            0x05  
    .equ     OS_CPU_ARM_EXCEPT_IRQ,             0x06
    .equ     OS_CPU_ARM_EXCEPT_FIQ,             0x07

    .equ     OS_CPU_ARM_FPEXC_EN,               0x40000000               @VFP enable bit.

    .code 32



CPU_ARM_ExceptUndefInstrHndlr:
    @B ledblink                                                           @ LR offset to return from this exception:  0.
    STMFD   SP!, {R0-R12, LR}                                   @ Push working registers.
    MOV     R2, LR                                              @ Save link register.
    MOV     R0, #OS_CPU_ARM_EXCEPT_UND                          @ Set exception ID to OS_CPU_ARM_EXCEPT_UNDEF_INSTR.
    B            CPU_ARM_ExceptHndlr                         @ Branch to global exception handler.



CPU_ARM_ExceptSwiHndlr:
    @B ledblink                                                            @ LR offset to return from this exception:  0.
    STMFD   SP!, {R0-R12, LR}                                   @ Push working registers.
    MOV     R2, LR                                              @ Save link register.
    MOV     R0, #OS_CPU_ARM_EXCEPT_SWI                          @ Set exception ID to OS_CPU_ARM_EXCEPT_SWI.
    B            CPU_ARM_ExceptHndlr                         @ Branch to global exception handler.



CPU_ARM_ExceptPrefetchAbortHndlr:
    @B ledblink
    SUB     LR, LR, #4                                          @ LR offset to return from this exception: -4.
    STMFD   SP!, {R0-R12, LR}                                   @ Push working registers.
    MOV     R2, LR                                              @ Save link register.
    MOV     R0, #OS_CPU_ARM_EXCEPT_ABORT_PREFETCH               @ Set exception ID to OS_CPU_ARM_EXCEPT_PREFETCH_ABORT.
    B            CPU_ARM_ExceptHndlr                         @ Branch to global exception handler.



CPU_ARM_ExceptDataAbortHndlr:
    @B ledblink
    SUB     LR, LR, #8                                          @ LR offset to return from this exception: -8.
    STMFD   SP!, {R0-R12, LR}                                   @ Push working registers.
    MOV     R2, LR                                              @ Save link register.
    MOV     R0, #OS_CPU_ARM_EXCEPT_ABORT_DATA                   @ Set exception ID to OS_CPU_ARM_EXCEPT_DATA_ABORT.
    B            CPU_ARM_ExceptHndlr                         @ Branch to global exception handler.



CPU_ARM_ExceptFiqHndlr:
    @B ledblink
    SUB     LR, LR, #4                                          @ LR offset to return from this exception: -4.
    STMFD   SP!, {R0-R12, LR}                                   @ Push working registers.
    MOV     R2, LR                                              @ Save link register.
    MOV     R0, #OS_CPU_ARM_EXCEPT_FIQ                          @ Set exception ID to OS_CPU_ARM_EXCEPT_FIQ.
    B            CPU_ARM_ExceptHndlr                         @ Branch to global exception handler.


CPU_ARM_ExceptHndlr:
    MRS     R4, SPSR                                           
    AND     R0, R1, #OS_CPU_ARM_MODE_MASK
    
    LDR     R1, =except_handler                              @ ExceptHandler(ex_mode = R0)
    MOV     LR, PC
    BX      R1
    @@never return here.


@reentrant version irq handler

CPU_ARM_ExceptIrqHndlr:
    SUB     LR, LR, #4                                          @ LR offset to return from this exception: -4.
    SRSDB       sp!,#OS_CPU_ARM_MODE_SVC          @ Save LR_irq and SPSR_irq to SVC mode stack
    CPS #OS_CPU_ARM_MODE_SVC                     @ Switch to SVC mode
    PUSH        {R0-R4,ip}      @ Store other AAPCS registers
    AND         R1, sp, #4
    SUB         sp, sp, R1
    PUSH        {R1, lr}
    BL          identify_and_clear_source       @maybe need mask low priority irq, irq_nr hold in R0
    CPSIE       i                @ Enable IRQ
    BL          C_irq_handler	@we handle irq_isr in SVC mode with interrupt enable
    CPSID i                      @Disable IRQ	@@maybe need unmask low priority irq
    POP         {R1,lr}
    ADD         sp, sp, R1
    POP         {R0-R4,ip}     @ Restore registers
    RFEIA       sp!              @Return using RFE from System mode stack
