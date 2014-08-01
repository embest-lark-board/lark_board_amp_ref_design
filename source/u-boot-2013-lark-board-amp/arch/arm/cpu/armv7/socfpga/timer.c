/*
 *  Copyright (C) 2012 Altera Corporation <www.altera.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <common.h>
#include <asm/io.h>
#include <asm/arch/timer.h>

DECLARE_GLOBAL_DATA_PTR;

static const struct socfpga_timer *timer_base = (void *)CONFIG_SYS_TIMERBASE;

#ifdef CONFIG_SPL_BUILD
extern u32 get_core_id(void);
extern void load_cpu0_global_data_to_cpu1_r8(void);
extern void restore_cpu1_r8(void);
#endif
/*
 * Timer initialization
 */
int timer_init(void)
{
	writel(TIMER_LOAD_VAL, &timer_base->load_val);
	writel(TIMER_LOAD_VAL, &timer_base->curr_val);
	writel(readl(&timer_base->ctrl) | 0x3, &timer_base->ctrl);
	return 0;
}

static u32 read_timer(void)
{
	return readl(&timer_base->curr_val);
}

/*
 * Delay x useconds
 */
void __udelay(unsigned long usec)
{
	unsigned long now, last;
	/*
	 * get the tmo value based on timer clock speed
	 * tmo = delay required / period of timer clock
	 */
	long tmo = usec * CONFIG_TIMER_CLOCK_KHZ / 1000;

	last = read_timer();
	while (tmo > 0) {
		now = read_timer();
		if (last >= now)
			/* normal mode (non roll) */
			tmo -= last - now;
		else
			/* we have overflow of the count down timer */
			tmo -= TIMER_LOAD_VAL - last + now;
		last = now;
	}
}

/*
 * Get the timer value
 */
ulong get_timer(ulong base)
{
	return get_timer_masked() - base;
}

/*
 * Timer : get the time difference
 * Unit of tick is based on the CONFIG_SYS_HZ
 */
 
 /*
***************************************************************************
*                       Embest Tech co., ltd
*                        www.embest-tech.com
***************************************************************************
* this function is called in the qspi driver,
*  cpu1 must restore the global_data_ptr to r8 because of the it uses the gd.
*/	
ulong get_timer_masked(void)
{
	ulong now;
	u32 id;
	unsigned long tbl;
	
#ifdef CONFIG_SPL_BUILD
	id = get_core_id();
	if(id == 1)
	{
		load_cpu0_global_data_to_cpu1_r8();
	}
#endif
	/* current tick value */
	now = read_timer() /
		(CONFIG_TIMER_CLOCK_KHZ * 1000 / CONFIG_SYS_HZ);
	if (gd->lastinc >= now) {
		/* normal mode (non roll) */
		/* move stamp forward with absolute diff ticks */
		gd->tbl += gd->lastinc - now;
	} else {
		/* we have overflow of the count down timer */
		gd->tbl += (TIMER_LOAD_VAL / (CONFIG_TIMER_CLOCK_KHZ * 1000
			/ CONFIG_SYS_HZ)) - gd->lastinc + now;
	}
	gd->lastinc = now;
	tbl = gd->tbl;
	
#ifdef CONFIG_SPL_BUILD
		if(id == 1)
			restore_cpu1_r8();
#endif

	return tbl;
}

/*
 * Reset the timer
 */

/*
***************************************************************************
*                       Embest Tech co., ltd
*                        www.embest-tech.com
***************************************************************************
* this function is called in the qspi driver,
*  cpu1 must restore the global_data_ptr to r8 because of the  it uses the gd.
*/	
void reset_timer(void)
{
	u32 id;

#ifdef CONFIG_SPL_BUILD
	id = get_core_id();
	if(id == 1)
	{
		load_cpu0_global_data_to_cpu1_r8();
	}
#endif

	/* capture current decrementer value time */
	gd->lastinc = read_timer() /
		(CONFIG_TIMER_CLOCK_KHZ * 1000 / CONFIG_SYS_HZ);
	/* start "advancing" time stamp from 0 */
	gd->tbl = 0;
		
#ifdef CONFIG_SPL_BUILD
		if(id == 1)
			restore_cpu1_r8();
#endif
}


/*
 * Get the timer count value
 */
ulong get_timer_count_masked(void)
{
	/* current tick value */
	ulong now = read_timer();
	if (gd->lastinc >= now) {
		/*
		 * normal mode (non roll)
		 * move stamp forward with absolute diff ticks
		 */
		gd->tbl += gd->lastinc - now;
	} else {
		/* we have overflow of the count down timer */
		gd->tbl += TIMER_LOAD_VAL - gd->lastinc + now;
	}
	gd->lastinc = now;
	return gd->tbl;
}

ulong get_timer_count(ulong base)
{
	return get_timer_count_masked() - base;
}

/*
 * Reset the timer
 */
void reset_timer_count(void)
{
	/* capure current decrementer value time */
	gd->lastinc = read_timer();
	/* start "advancing" time stamp from 0 */
	gd->tbl = 0;
}

/*
 * This function is derived from PowerPC code (read timebase as long long).
 * On ARM it just returns the timer value.
 */
unsigned long long get_ticks(void)
{
	return get_timer(0);
}

/*
 * This function is derived from PowerPC code (timebase clock frequency).
 * On ARM it returns the number of timer ticks per second.
 */
ulong get_tbclk(void)
{
	return CONFIG_SYS_HZ;
}
