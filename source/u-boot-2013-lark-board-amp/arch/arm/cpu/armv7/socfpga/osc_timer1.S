#include <asm-offsets.h>
#include <config.h>
#include <version.h>
#include <asm/system.h>
#include <linux/linkage.h>


#define OSC_TIMER_LOAD				0x0
#define OSC_TIMER_CUR_CNT				0x4
#define OSC_TIEMR_CTL					0x8
#define OSC_TIMER_INT_STATUS	0xc

#define OSC_TIMER_EN					0x1
#define OSC_TIMER_FREE_RUN	(0x0 << 1) 
#define OSC_TIMER_USE_DEF	(0x1 << 1)
#define OSC_TIMER_INT_MASKED	(0x1 << 2)

#define OSC_TIMER_CTL_VAL	(OSC_TIMER_FREE_RUN | OSC_TIMER_INT_MASKED|OSC_TIMER_EN)

#define OSC_TIMER_LOAD_VAL		0xffffffff

.globl osc_timer_reset_release
osc_timer_reset_release:
	ldr r0, = 0xffd05014
	ldr r1, [r0]
	bic r1, #(0x1 << 9)
	str r1, [r0]
	bx lr




/*
 * OSC1 Timer 1 clock is 25MHz -->40ns
 * r0 : timer base
 * r1 : corrupted
 */

.global osc_timer_config
osc_timer_config:
	ldr r0, = CONFIG_OSC1_TIMER1_BASE
	ldr r1, = OSC_TIMER_LOAD_VAL
	add r2, r0, #OSC_TIMER_LOAD
	str r1, [r2]
	
	mov r1, #OSC_TIMER_CTL_VAL
	
	add r2, r0, #OSC_TIEMR_CTL
  	str r1, [r2]
  	bx lr



/* r0 : timer base
 */
.global osc_timer_get
osc_timer_get:
	ldr r0, = CONFIG_OSC1_TIMER1_BASE
	add r1, r0, #OSC_TIMER_CUR_CNT
	ldr r0, [r1]
	bx lr


