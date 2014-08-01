#include <types.h>
#include <dma.h>
#include <dma_internal.h>
#include <bmlog.h>

/*
  * Come from the hwlib of the quartus 13.0sp1
 */


#ifndef MIN
#define MIN(a, b) ((a) > (b) ? (b) : (a))
#endif

#define dprintf(...)
//#define dprintf(fmt,arg...) 	bmlog(fmt,##arg)
extern printk_fn printk;

typedef struct ALT_DMA_CHANNEL_INFO_s
{
    u8 flag;
}
ALT_DMA_CHANNEL_INFO_t;

#define ALT_CAST(type, ptr)  ((type) (ptr))

#define alt_write_word(dest, src)       (*ALT_CAST(volatile u32 *, (dest)) = (src))
#define alt_read_word(src)              (*ALT_CAST(volatile u32 *, (src)))

#define alt_clrbits_word(dest, bits)        (alt_write_word(dest, alt_read_word(dest) & ~(bits)))
#define alt_setbits_word(dest, bits)        (alt_write_word(dest, alt_read_word(dest) | (bits)))


#define ALT_DMA_CHANNLE						8

#define ALT_DMA_DSR_OFST 0x0
#define ALT_DMA_DSR_ADDR(base) ALT_CAST(void *, (ALT_CAST(char *, (base)) + ALT_DMA_DSR_OFST))
#define ALT_DMA_DSR_DMASTATUS_SET_MSK 0x0000000f
#define ALT_DMA_DSR_DMASTATUS_GET(value) ((value) & 0x0000000f)

// DMA Program Counter Register
#define ALT_DMA_DPC_OFST 0x4
#define ALT_DMA_DPC_ADDR(base) ALT_CAST(void *, (ALT_CAST(char *, (base)) + ALT_DMA_DPC_OFST))

// Interrupt Enable Register
#define ALT_DMA_INTEN_OFST 0x20
#define ALT_DMA_INTEN_ADDR(base) ALT_CAST(void *, (ALT_CAST(char *, (base)) + ALT_DMA_INTEN_OFST))

// Event-Interrupt Raw Status Register
#define ALT_DMA_INT_EVENT_RIS_OFST 0x24
#define ALT_DMA_INT_EVENT_RIS_ADDR(base) ALT_CAST(void *, (ALT_CAST(char *, (base)) + ALT_DMA_INT_EVENT_RIS_OFST))

// Interrupt Status Register
#define ALT_DMA_INTMIS_OFST 0x28
#define ALT_DMA_INTMIS_ADDR(base) ALT_CAST(void *, (ALT_CAST(char *, (base)) + ALT_DMA_INTMIS_OFST))

// Interrupt Clear Register
#define ALT_DMA_INTCLR_OFST 0x2c
#define ALT_DMA_INTCLR_ADDR(base) ALT_CAST(void *, (ALT_CAST(char *, (base)) + ALT_DMA_INTCLR_OFST))

// Fault Status DMA Manager Register
#define ALT_DMA_FSRD_OFST 0x30
#define ALT_DMA_FSRD_ADDR(base) ALT_CAST(void *, (ALT_CAST(char *, (base)) + ALT_DMA_FSRD_OFST))

// Fault Status DMA Channel Register
#define ALT_DMA_FSRC_OFST 0x34
#define ALT_DMA_FSRC_ADDR(base) ALT_CAST(void *, (ALT_CAST(char *, (base)) + ALT_DMA_FSRC_OFST))

// Fault Type DMA Manager Register
#define ALT_DMA_FTRD_OFST 0x38
#define ALT_DMA_FTRD_ADDR(base) ALT_CAST(void *, (ALT_CAST(char *, (base)) + ALT_DMA_FSRD_OFST))

// Fault Type DMA Channel Registers
#define ALT_DMA_FTRx_OFST(channel) (0x40 + 0x4 * (channel))
#define ALT_DMA_FTRx_ADDR(base, channel) ALT_CAST(void *, (ALT_CAST(char *, (base)) + ALT_DMA_FTRx_OFST(channel)))

// Channel Status Registers
#define ALT_DMA_CSRx_OFST(channel) (0x100 + 0x8 * (channel))
#define ALT_DMA_CSRx_ADDR(base, channel) ALT_CAST(void *, (ALT_CAST(char *, (base)) + ALT_DMA_CSRx_OFST(channel)))
#define ALT_DMA_CSRx_CHANNELSTATUS_SET_MSK 0x0000000f
#define ALT_DMA_CSRx_CHANNELSTATUS_GET(value) ((value) & 0x0000000f)

// Channel Program Counter Registers
#define ALT_DMA_CPCx_OFST(channel) (0x104 + 0x8 * (channel))
#define ALT_DMA_CPCx_ADDR(base, channel) ALT_CAST(void *, (ALT_CAST(char *, (base)) + ALT_DMA_CPCx_OFST(channel)))

// Source Address Registers
#define ALT_DMA_SARx_OFST(channel) (0x400 + 0x20 * (channel))
#define ALT_DMA_SARx_ADDR(base, channel) ALT_CAST(void *, (ALT_CAST(char *, (base)) + ALT_DMA_SARx_OFST(channel)))

// Destination Address Registers
#define ALT_DMA_DARx_OFST(channel) (0x404 + 0x20 * (channel))
#define ALT_DMA_DARx_ADDR(base, channel) ALT_CAST(void *, (ALT_CAST(char *, (base)) + ALT_DMA_DARx_OFST(channel)))

// Channel Control Registers
#define ALT_DMA_CCRx_OFST(channel) (0x408 + 0x20 * (channel))
#define ALT_DMA_CCRx_ADDR(base, channel) ALT_CAST(void *, (ALT_CAST(char *, (base)) + ALT_DMA_CCRx_OFST(channel)))

// Loop Counter 0 Registers
#define ALT_DMA_LC0_x_OFST(channel) (0x40c + 0x20 * (channel))
#define ALT_DMA_LC0_x_ADDR(base, channel) ALT_CAST(void *, (ALT_CAST(char *, (base)) + ALT_DMA_LC0_x_OFST(channel)))

// Loop Counter 1 Registers
#define ALT_DMA_LC1_x_OFST(channel) (0x410 + 0x20 * (channel))
#define ALT_DMA_LC1_x_ADDR(base, channel) ALT_CAST(void *, (ALT_CAST(char *, (base)) + ALT_DMA_LC1_x_OFST(channel)))

// Debug Status Register
#define ALT_DMA_DBGSTATUS_OFST 0xd00
#define ALT_DMA_DBGSTATUS_ADDR(base) ALT_CAST(void *, (ALT_CAST(char *, (base)) + ALT_DMA_DBGSTATUS_OFST))

// Debug Command Register
#define ALT_DMA_DBGCMD_OFST 0xd04
#define ALT_DMA_DBGCMD_ADDR(base) ALT_CAST(void *, (ALT_CAST(char *, (base)) + ALT_DMA_DBGCMD_OFST))

// Debug Instruction-0 Register
#define ALT_DMA_DBGINST0_OFST 0xd08
#define ALT_DMA_DBGINST0_ADDR(base) ALT_CAST(void *, (ALT_CAST(char *, (base)) + ALT_DMA_DBGINST0_OFST))
#define ALT_DMA_DBGINST0_CHANNELNUMBER_SET(value) (((value) & 0x7) << 8)
#define ALT_DMA_DBGINST0_DEBUGTHREAD_SET(value) ((value) & 0x1)
#define ALT_DMA_DBGINST0_DEBUGTHREAD_E_MANAGER 0
#define ALT_DMA_DBGINST0_DEBUGTHREAD_E_CHANNEL 1
#define ALT_DMA_DBGINST0_INSTRUCTIONBYTE0_SET(value) (((value) & 0xff) << 16)
#define ALT_DMA_DBGINST0_INSTRUCTIONBYTE1_SET(value) (((value) & 0xff) << 24)

// Debug Instruction-1 Register
#define ALT_DMA_DBGINST1_OFST 0xd0c
#define ALT_DMA_DBGINST1_ADDR(base) ALT_CAST(void *, (ALT_CAST(char *, (base)) + ALT_DMA_DBGINST1_OFST))

// Configuration Registers 0 - 4
#define ALT_DMA_CR0_OFST 0xe00
#define ALT_DMA_CR1_OFST 0xe04
#define ALT_DMA_CR2_OFST 0xe08
#define ALT_DMA_CR3_OFST 0xe0c
#define ALT_DMA_CR4_OFST 0xe10
#define ALT_DMA_CR0_ADDR(base) ALT_CAST(void *, (ALT_CAST(char *, (base)) + ALT_DMA_CR0_OFST))
#define ALT_DMA_CR1_ADDR(base) ALT_CAST(void *, (ALT_CAST(char *, (base)) + ALT_DMA_CR1_OFST))
#define ALT_DMA_CR2_ADDR(base) ALT_CAST(void *, (ALT_CAST(char *, (base)) + ALT_DMA_CR2_OFST))
#define ALT_DMA_CR3_ADDR(base) ALT_CAST(void *, (ALT_CAST(char *, (base)) + ALT_DMA_CR3_OFST))
#define ALT_DMA_CR4_ADDR(base) ALT_CAST(void *, (ALT_CAST(char *, (base)) + ALT_DMA_CR4_OFST))

// DMA Configuration Register
#define ALT_DMA_CRD_OFST 0xe14
#define ALT_DMA_CRD_ADDR(base) ALT_CAST(void *, (ALT_CAST(char *, (base)) + ALT_DMA_CRD_OFST))

// Watchdog Register
#define ALT_DMA_WD_OFST 0xe80
#define ALT_DMA_WD_ADDR(base) ALT_CAST(void *, (ALT_CAST(char *, (base)) + ALT_DMA_WD_OFST))




/*
 * Component Instance : rstmgr
 * 
 * Instance rstmgr of component ALT_RSTMGR.
 * 
 * 
 */
#define ALT_HPS_ADDR		0
#define ALT_RSTMGR_STAT_OFST        		0x0
#define ALT_RSTMGR_CTL_OFST        		0x4
#define ALT_RSTMGR_COUNTS_OFST		0x8
#define ALT_RSTMGR_MPUMODRST_OFST        0x10
#define ALT_RSTMGR_PERMODRST_OFST        0x14
#define ALT_RSTMGR_PER2MODRST_OFST        0x18
#define ALT_RSTMGR_BRGMODRST_OFST        0x1c
#define ALT_RSTMGR_MISCMODRST_OFST        0x20

/* The address of the ALT_RSTMGR_STAT register for the ALT_RSTMGR instance. */
#define ALT_RSTMGR_STAT_ADDR  ALT_CAST(void *, (ALT_CAST(char *, ALT_RSTMGR_ADDR) + ALT_RSTMGR_STAT_OFST))
/* The address of the ALT_RSTMGR_CTL register for the ALT_RSTMGR instance. */
#define ALT_RSTMGR_CTL_ADDR  ALT_CAST(void *, (ALT_CAST(char *, ALT_RSTMGR_ADDR) + ALT_RSTMGR_CTL_OFST))
/* The address of the ALT_RSTMGR_COUNTS register for the ALT_RSTMGR instance. */
#define ALT_RSTMGR_COUNTS_ADDR  ALT_CAST(void *, (ALT_CAST(char *, ALT_RSTMGR_ADDR) + ALT_RSTMGR_COUNTS_OFST))
/* The address of the ALT_RSTMGR_MPUMODRST register for the ALT_RSTMGR instance. */
#define ALT_RSTMGR_MPUMODRST_ADDR  ALT_CAST(void *, (ALT_CAST(char *, ALT_RSTMGR_ADDR) + ALT_RSTMGR_MPUMODRST_OFST))
/* The address of the ALT_RSTMGR_PERMODRST register for the ALT_RSTMGR instance. */
#define ALT_RSTMGR_PERMODRST_ADDR  ALT_CAST(void *, (ALT_CAST(char *, ALT_RSTMGR_ADDR) + ALT_RSTMGR_PERMODRST_OFST))
/* The address of the ALT_RSTMGR_PER2MODRST register for the ALT_RSTMGR instance. */
#define ALT_RSTMGR_PER2MODRST_ADDR  ALT_CAST(void *, (ALT_CAST(char *, ALT_RSTMGR_ADDR) + ALT_RSTMGR_PER2MODRST_OFST))
/* The address of the ALT_RSTMGR_BRGMODRST register for the ALT_RSTMGR instance. */
#define ALT_RSTMGR_BRGMODRST_ADDR  ALT_CAST(void *, (ALT_CAST(char *, ALT_RSTMGR_ADDR) + ALT_RSTMGR_BRGMODRST_OFST))
/* The address of the ALT_RSTMGR_MISCMODRST register for the ALT_RSTMGR instance. */
#define ALT_RSTMGR_MISCMODRST_ADDR  ALT_CAST(void *, (ALT_CAST(char *, ALT_RSTMGR_ADDR) + ALT_RSTMGR_MISCMODRST_OFST))
/* The base address byte offset for the start of the ALT_RSTMGR component. */
#define ALT_RSTMGR_OFST        0xffd05000
/* The start address of the ALT_RSTMGR component. */
#define ALT_RSTMGR_ADDR        ALT_CAST(void *, (ALT_CAST(char *, ALT_HPS_ADDR) + ALT_RSTMGR_OFST))



/*
 * Field : IRQ Security - irqnonsecure
 * 
 * Specifies the security state of an event-interrupt resource.
 * 
 * If bit index [x] is 0, the DMAC assigns event<x> or irq[x] to the Secure state.
 * 
 * If bit index [x] is 1, the DMAC assigns event<x> or irq[x] to the Non-secure
 * state.
 * 
 * Field Access Macros:
 * 
 */
/* The Least Significant Bit (LSB) position of the ALT_SYSMGR_DMA_CTL_IRQNONSECURE register field. */
#define ALT_SYSMGR_DMA_CTL_IRQNONSECURE_LSB        5
/* The Most Significant Bit (MSB) position of the ALT_SYSMGR_DMA_CTL_IRQNONSECURE register field. */
#define ALT_SYSMGR_DMA_CTL_IRQNONSECURE_MSB        12
/* The width in bits of the ALT_SYSMGR_DMA_CTL_IRQNONSECURE register field. */
#define ALT_SYSMGR_DMA_CTL_IRQNONSECURE_WIDTH      8
/* The mask used to set the ALT_SYSMGR_DMA_CTL_IRQNONSECURE register field value. */
#define ALT_SYSMGR_DMA_CTL_IRQNONSECURE_SET_MSK    0x00001fe0
/* The mask used to clear the ALT_SYSMGR_DMA_CTL_IRQNONSECURE register field value. */
#define ALT_SYSMGR_DMA_CTL_IRQNONSECURE_CLR_MSK    0xffffe01f
/* The reset value of the ALT_SYSMGR_DMA_CTL_IRQNONSECURE register field. */
#define ALT_SYSMGR_DMA_CTL_IRQNONSECURE_RESET      0x0
/* Extracts the ALT_SYSMGR_DMA_CTL_IRQNONSECURE field value from a register. */
#define ALT_SYSMGR_DMA_CTL_IRQNONSECURE_GET(value) (((value) & 0x00001fe0) >> 5)
/* Produces a ALT_SYSMGR_DMA_CTL_IRQNONSECURE register field value suitable for setting the register. */
#define ALT_SYSMGR_DMA_CTL_IRQNONSECURE_SET(value) (((value) << 5) & 0x00001fe0)

#define ALT_SYSMGR_DMA_CTL_CHANSEL_0_E_FPGA 0x0
/*
 * Enumerated value for register field ALT_SYSMGR_DMA_CTL_CHANSEL_0
 * 
 * CAN drives peripheral request interface
 */
#define ALT_SYSMGR_DMA_CTL_CHANSEL_0_E_CAN  0x1

/* The Least Significant Bit (LSB) position of the ALT_SYSMGR_DMA_CTL_CHANSEL_0 register field. */
#define ALT_SYSMGR_DMA_CTL_CHANSEL_0_LSB        0
/* The Most Significant Bit (MSB) position of the ALT_SYSMGR_DMA_CTL_CHANSEL_0 register field. */
#define ALT_SYSMGR_DMA_CTL_CHANSEL_0_MSB        0
/* The width in bits of the ALT_SYSMGR_DMA_CTL_CHANSEL_0 register field. */
#define ALT_SYSMGR_DMA_CTL_CHANSEL_0_WIDTH      1
/* The mask used to set the ALT_SYSMGR_DMA_CTL_CHANSEL_0 register field value. */
#define ALT_SYSMGR_DMA_CTL_CHANSEL_0_SET_MSK    0x00000001
/* The mask used to clear the ALT_SYSMGR_DMA_CTL_CHANSEL_0 register field value. */
#define ALT_SYSMGR_DMA_CTL_CHANSEL_0_CLR_MSK    0xfffffffe
/* The reset value of the ALT_SYSMGR_DMA_CTL_CHANSEL_0 register field. */
#define ALT_SYSMGR_DMA_CTL_CHANSEL_0_RESET      0x0
/* Extracts the ALT_SYSMGR_DMA_CTL_CHANSEL_0 field value from a register. */
#define ALT_SYSMGR_DMA_CTL_CHANSEL_0_GET(value) (((value) & 0x00000001) >> 0)
/* Produces a ALT_SYSMGR_DMA_CTL_CHANSEL_0 register field value suitable for setting the register. */
#define ALT_SYSMGR_DMA_CTL_CHANSEL_0_SET(value) (((value) << 0) & 0x00000001)

/* The Least Significant Bit (LSB) position of the ALT_SYSMGR_DMA_CTL_MGRNONSECURE register field. */
#define ALT_SYSMGR_DMA_CTL_MGRNONSECURE_LSB        4
/* The Most Significant Bit (MSB) position of the ALT_SYSMGR_DMA_CTL_MGRNONSECURE register field. */
#define ALT_SYSMGR_DMA_CTL_MGRNONSECURE_MSB        4
/* The width in bits of the ALT_SYSMGR_DMA_CTL_MGRNONSECURE register field. */
#define ALT_SYSMGR_DMA_CTL_MGRNONSECURE_WIDTH      1
/* The mask used to set the ALT_SYSMGR_DMA_CTL_MGRNONSECURE register field value. */
#define ALT_SYSMGR_DMA_CTL_MGRNONSECURE_SET_MSK    0x00000010
/* The mask used to clear the ALT_SYSMGR_DMA_CTL_MGRNONSECURE register field value. */
#define ALT_SYSMGR_DMA_CTL_MGRNONSECURE_CLR_MSK    0xffffffef
/* The reset value of the ALT_SYSMGR_DMA_CTL_MGRNONSECURE register field. */
#define ALT_SYSMGR_DMA_CTL_MGRNONSECURE_RESET      0x0
/* Extracts the ALT_SYSMGR_DMA_CTL_MGRNONSECURE field value from a register. */
#define ALT_SYSMGR_DMA_CTL_MGRNONSECURE_GET(value) (((value) & 0x00000010) >> 4)
/* Produces a ALT_SYSMGR_DMA_CTL_MGRNONSECURE register field value suitable for setting the register. */
#define ALT_SYSMGR_DMA_CTL_MGRNONSECURE_SET(value) (((value) << 4) & 0x00000010)

/* The base address byte offset for the start of the ALT_SYSMGR component. */
#define ALT_SYSMGR_OFST        0xffd08000
/* The start address of the ALT_SYSMGR component. */
#define ALT_SYSMGR_ADDR        ALT_CAST(void *, (ALT_CAST(char *, ALT_HPS_ADDR) + ALT_SYSMGR_OFST))


#define ALT_SYSMGR_DMA_CTL_OFST        			0x0
#define ALT_SYSMGR_DMA_PERSECURITY_OFST        0x4
/* The address of the ALT_SYSMGR_DMA_CTL register for the ALT_SYSMGR_DMA instance. */
#define ALT_SYSMGR_DMA_CTL_ADDR  ALT_CAST(void *, (ALT_CAST(char *, ALT_SYSMGR_DMA_ADDR) + ALT_SYSMGR_DMA_CTL_OFST))
/* The address of the ALT_SYSMGR_DMA_PERSECURITY register for the ALT_SYSMGR_DMA instance. */
#define ALT_SYSMGR_DMA_PERSECURITY_ADDR  ALT_CAST(void *, (ALT_CAST(char *, ALT_SYSMGR_DMA_ADDR) + ALT_SYSMGR_DMA_PERSECURITY_OFST))
/* The base address byte offset for the start of the ALT_SYSMGR_DMA component. */
#define ALT_SYSMGR_DMA_OFST        0x70
/* The start address of the ALT_SYSMGR_DMA component. */
#define ALT_SYSMGR_DMA_ADDR        ALT_CAST(void *, (ALT_CAST(char *, ALT_SYSMGR_ADDR) + ALT_SYSMGR_DMA_OFST))
/* The lower bound address range of the ALT_SYSMGR_DMA component. */



/* The Least Significant Bit (LSB) position of the ALT_RSTMGR_PERMODRST_DMA register field. */
#define ALT_RSTMGR_PERMODRST_DMA_LSB        28
/* The Most Significant Bit (MSB) position of the ALT_RSTMGR_PERMODRST_DMA register field. */
#define ALT_RSTMGR_PERMODRST_DMA_MSB        28
/* The width in bits of the ALT_RSTMGR_PERMODRST_DMA register field. */
#define ALT_RSTMGR_PERMODRST_DMA_WIDTH      1
/* The mask used to set the ALT_RSTMGR_PERMODRST_DMA register field value. */
#define ALT_RSTMGR_PERMODRST_DMA_SET_MSK    0x10000000
/* The mask used to clear the ALT_RSTMGR_PERMODRST_DMA register field value. */
#define ALT_RSTMGR_PERMODRST_DMA_CLR_MSK    0xefffffff
/* The reset value of the ALT_RSTMGR_PERMODRST_DMA register field. */
#define ALT_RSTMGR_PERMODRST_DMA_RESET      0x1
/* Extracts the ALT_RSTMGR_PERMODRST_DMA field value from a register. */
#define ALT_RSTMGR_PERMODRST_DMA_GET(value) (((value) & 0x10000000) >> 28)
/* Produces a ALT_RSTMGR_PERMODRST_DMA register field value suitable for setting the register. */
#define ALT_RSTMGR_PERMODRST_DMA_SET(value) (((value) << 28) & 0x10000000)


/* The base address byte offset for the start of the ALT_DMASECURE component. */
#define ALT_DMASECURE_OFST        0xffe01000
/* The start address of the ALT_DMASECURE component. */
#define ALT_DMASECURE_ADDR        ALT_CAST(void *, (ALT_CAST(char *, ALT_HPS_ADDR) + ALT_DMASECURE_OFST))

/////

//
// Internal Data structures
//

// This flag marks the channel as being allocated.
#define ALT_DMA_CHANNEL_INFO_FLAG_ALLOCED (1 << 0)

static ALT_DMA_CHANNEL_INFO_t channel_info_array[ALT_DMA_CHANNLE];

s32 alt_dma_init(const ALT_DMA_CFG_t * dma_cfg)
{
	u32 dmapersecurity;
	u32 dmactrl;
	int i;
		
    // Initialize the channel information array
    for (i = 0; i < 8; ++i)
    {
        channel_info_array[i].flag = 0;
    }

    // Update the System Manager DMA configuration items
    
    dmactrl = 0;

    // Handle FPGA / CAN muxing
    for (i = 0; i < 4; ++i)
    {
        // The default is FPGA.
        switch (dma_cfg->periph_mux[i])
        {
        case ALT_DMA_PERIPH_MUX_DEFAULT:
        case ALT_DMA_PERIPH_MUX_FPGA:
            break;
        case ALT_DMA_PERIPH_MUX_CAN:
            dmactrl |= (ALT_SYSMGR_DMA_CTL_CHANSEL_0_SET_MSK << i);
            break;
        default:
            return ALT_E_ERROR;
        }
    }

    // Handle Manager security
    // Default is Secure state.
    switch (dma_cfg->manager_sec)
    {
    case ALT_DMA_SECURITY_DEFAULT:
    case ALT_DMA_SECURITY_SECURE:
        break;
    case ALT_DMA_SECURITY_NONSECURE:
        dmactrl |= ALT_SYSMGR_DMA_CTL_MGRNONSECURE_SET_MSK;
        break;
    default:
        return ALT_E_ERROR;
    }

    // Handle IRQ security
    for (i = 0; i < ALT_SYSMGR_DMA_CTL_IRQNONSECURE_WIDTH; ++i)
    {
        // Default is Secure state.
        switch (dma_cfg->irq_sec[i])
        {
        case ALT_DMA_SECURITY_DEFAULT:
        case ALT_DMA_SECURITY_SECURE:
            break;
        case ALT_DMA_SECURITY_NONSECURE:
            dmactrl |= (1 << (i + ALT_SYSMGR_DMA_CTL_IRQNONSECURE_LSB));
            break;
        default:
            return ALT_E_ERROR;
        }
    }

    alt_write_word(ALT_SYSMGR_DMA_CTL_ADDR, dmactrl);

    // Update the System Manager DMA peripheral security items

    dmapersecurity = 0;

    for (i = 0; i < 32; ++i)
    {
        // Default is Secure state.
        switch (dma_cfg->periph_sec[i])
        {
        case ALT_DMA_SECURITY_DEFAULT:
        case ALT_DMA_SECURITY_SECURE:
            break;
        case ALT_DMA_SECURITY_NONSECURE:
            dmapersecurity |= (1 << i);
            break;
        default:
            return ALT_E_ERROR;
        }
    }

    alt_write_word(ALT_SYSMGR_DMA_PERSECURITY_ADDR, dmapersecurity);

    // Take DMA out of reset.

    alt_clrbits_word(ALT_RSTMGR_PERMODRST_ADDR, ALT_RSTMGR_PERMODRST_DMA_SET_MSK);

    return ALT_E_SUCCESS;
}

s32 alt_dma_uninit(void)
{
	int i;
	
    // DMAKILL all channel and free all allocated channels.
    for (i = 0; i < 8; ++i)
    {
        if (channel_info_array[i].flag & ALT_DMA_CHANNEL_INFO_FLAG_ALLOCED)
        {
            alt_dma_channel_kill((ALT_DMA_CHANNEL_t)i);
            alt_dma_channel_free((ALT_DMA_CHANNEL_t)i);
        }
    }

    // Put DMA into reset.

    alt_setbits_word(ALT_RSTMGR_PERMODRST_ADDR, ALT_RSTMGR_PERMODRST_DMA_SET_MSK);

    return ALT_E_SUCCESS;
}

s32 alt_dma_channel_kill(ALT_DMA_CHANNEL_t channel)
{
	u32 i;
	s32 status;

	// Validate channel
	switch (channel)
	{
		case ALT_DMA_CHANNEL_0:
		case ALT_DMA_CHANNEL_1:
		case ALT_DMA_CHANNEL_2:
		case ALT_DMA_CHANNEL_3:
		case ALT_DMA_CHANNEL_4:
		case ALT_DMA_CHANNEL_5:
		case ALT_DMA_CHANNEL_6:
		case ALT_DMA_CHANNEL_7:
			break;
		default:
			return ALT_E_BAD_ARG;
	}

	// Verify channel is allocated

	if (!(channel_info_array[channel].flag & ALT_DMA_CHANNEL_INFO_FLAG_ALLOCED))
	{
		return ALT_E_ERROR;
	}

	// NOTE: Don't worry about the current channel state. Just issue DMAKILL
	//   instruction. The channel state cannot move from from Stopped back to
	//   Killing.

	// Configure DBGINST0 to execute DMAKILL on the requested channel thread.
	// DMAKILL is short enough not to use DBGINST1 register.

	// For information on APB Interface, see PL330, section 2.5.1.
	// For information on DBGINSTx, see PL330, section 3.3.20 - 3.3.21.
	// For information on DMAKILL, see PL330, section 4.3.6.

	alt_write_word(ALT_DMA_DBGINST0_ADDR(ALT_DMASECURE_ADDR),
	           ALT_DMA_DBGINST0_INSTRUCTIONBYTE0_SET(0x1) |
	           ALT_DMA_DBGINST0_CHANNELNUMBER_SET(channel) |
	           ALT_DMA_DBGINST0_DEBUGTHREAD_SET(ALT_DMA_DBGINST0_DEBUGTHREAD_E_CHANNEL));

	// Execute the instruction held in DBGINST0

	// For information on DBGCMD, see PL330, section 3.3.19.

	alt_write_word(ALT_DMA_DBGCMD_ADDR(ALT_DMASECURE_ADDR), 0);

	// Wait for channel to move to KILLING or STOPPED state. Do not wait for
	// the STOPPED only. If the AXI transaction hangs permanently, it can be
	// waiting indefinately.

	status = ALT_E_SUCCESS;
	ALT_DMA_CHANNEL_STATE_t current;
	i = 20000;

	while (--i)
	{
		status = alt_dma_channel_state_get(channel, &current);
		if (status != ALT_E_SUCCESS)
		{
		    break;
		}
		if (   (current == ALT_DMA_CHANNEL_STATE_KILLING)
		    || (current == ALT_DMA_CHANNEL_STATE_STOPPED))
		{
		    break;
		}
	}

	if (i == 0)
	{
		status = ALT_E_TMO;
	}

	return status;
}


s32 alt_dma_channel_alloc(ALT_DMA_CHANNEL_t channel)
{
    // Validate channel
    switch (channel)
    {
    case ALT_DMA_CHANNEL_0:
    case ALT_DMA_CHANNEL_1:
    case ALT_DMA_CHANNEL_2:
    case ALT_DMA_CHANNEL_3:
    case ALT_DMA_CHANNEL_4:
    case ALT_DMA_CHANNEL_5:
    case ALT_DMA_CHANNEL_6:
    case ALT_DMA_CHANNEL_7:
        break;
    default:
        return ALT_E_BAD_ARG;
    }

    // Verify channel is unallocated

    if (channel_info_array[channel].flag & ALT_DMA_CHANNEL_INFO_FLAG_ALLOCED)
    {
        return ALT_E_ERROR;
    }

    // Mark channel as allocated

    channel_info_array[channel].flag |= ALT_DMA_CHANNEL_INFO_FLAG_ALLOCED;

    return ALT_E_SUCCESS;
}

s32 alt_dma_channel_free(ALT_DMA_CHANNEL_t channel)
{
	ALT_DMA_CHANNEL_STATE_t state;
	s32 status;
	
    // Validate channel
    switch (channel)
    {
    case ALT_DMA_CHANNEL_0:
    case ALT_DMA_CHANNEL_1:
    case ALT_DMA_CHANNEL_2:
    case ALT_DMA_CHANNEL_3:
    case ALT_DMA_CHANNEL_4:
    case ALT_DMA_CHANNEL_5:
    case ALT_DMA_CHANNEL_6:
    case ALT_DMA_CHANNEL_7:
        break;
    default:
        return ALT_E_BAD_ARG;
    }

    // Verify channel is allocated

    if (!(channel_info_array[channel].flag & ALT_DMA_CHANNEL_INFO_FLAG_ALLOCED))
    {
        return ALT_E_ERROR;
    }

    // Verify channel is stopped
    status = alt_dma_channel_state_get(channel, &state);
    if (status != ALT_E_SUCCESS)
    {
        return status;
    }
    if (state != ALT_DMA_CHANNEL_STATE_STOPPED)
    {
        return ALT_E_ERROR;
    }

    // Mark channel as unallocated.

    channel_info_array[channel].flag &= ~ALT_DMA_CHANNEL_INFO_FLAG_ALLOCED;

    return ALT_E_SUCCESS;
}

s32 alt_dma_channel_exec(ALT_DMA_CHANNEL_t channel, ALT_DMA_PROGRAM_t * pgm)
{
	u32 start;
	ALT_DMA_CHANNEL_STATE_t state;
	s32 status;
	
    // Validate channel
    switch (channel)
    {
    case ALT_DMA_CHANNEL_0:
    case ALT_DMA_CHANNEL_1:
    case ALT_DMA_CHANNEL_2:
    case ALT_DMA_CHANNEL_3:
    case ALT_DMA_CHANNEL_4:
    case ALT_DMA_CHANNEL_5:
    case ALT_DMA_CHANNEL_6:
    case ALT_DMA_CHANNEL_7:
        break;
    default:
        return ALT_E_BAD_ARG;
    }

    // Verify channel is allocated

    if (!(channel_info_array[channel].flag & ALT_DMA_CHANNEL_INFO_FLAG_ALLOCED))
    {
        return ALT_E_ERROR;
    }

    // Verify channel is stopped

   status = alt_dma_channel_state_get(channel, &state);
    if (status != ALT_E_SUCCESS)
    {
        return status;
    }
    if (state != ALT_DMA_CHANNEL_STATE_STOPPED)
    {
        return ALT_E_ERROR;
    }

    // Validate the program

    if (alt_dma_program_validate(pgm) != ALT_E_SUCCESS)
    {
        return ALT_E_ERROR;
    }

    //
    // Execute the program
    //

    // Get the start address
   
    //	start = (u32) &pgm->program[pgm->buffer_start];
	start = pgm->program_phy + pgm->buffer_start;

    dprintf("DMA[exec]: pgm->program = %p.\n", pgm->program);
    dprintf("DMA[exec]: start        = %p.\n", (void *)start);

    // Configure DBGINST0 and DBGINST1 to execute DMAGO targetting the requested channel.

    // For information on APB Interface, see PL330, section 2.5.1.
    // For information on DBGINSTx, see PL330, section 3.3.20 - 3.3.21.
    // For information on DMAGO, see PL330, section 4.3.5.

    alt_write_word(ALT_DMA_DBGINST0_ADDR(ALT_DMASECURE_ADDR),
                   ALT_DMA_DBGINST0_INSTRUCTIONBYTE0_SET(0xa0) | 
                   ALT_DMA_DBGINST0_INSTRUCTIONBYTE1_SET(channel));

    alt_write_word(ALT_DMA_DBGINST1_ADDR(ALT_DMASECURE_ADDR), start);

    // Execute the instruction held in DBGINST{0,1}

    // For information on DBGCMD, see PL330, section 3.3.19.
    __asm__ __volatile__("dmb\n");

    alt_write_word(ALT_DMA_DBGCMD_ADDR(ALT_DMASECURE_ADDR), 0);

    return ALT_E_SUCCESS;
}

s32 alt_dma_send_event(ALT_DMA_EVENT_t evt_num)
{
    // Validate evt_num

    switch (evt_num)
    {
    case ALT_DMA_EVENT_0:
    case ALT_DMA_EVENT_1:
    case ALT_DMA_EVENT_2:
    case ALT_DMA_EVENT_3:
    case ALT_DMA_EVENT_4:
    case ALT_DMA_EVENT_5:
    case ALT_DMA_EVENT_6:
    case ALT_DMA_EVENT_7:
    case ALT_DMA_EVENT_ABORT:
        break;
    default:
        return ALT_E_BAD_ARG;
    }

    // Issue the DMASEV on the DMA manager thread.
    // DMASEV is short enough not to use DBGINST1 register.

    // For information on APB Interface, see PL330, section 2.5.1.
    // For information on DBGINSTx, see PL330, section 3.3.20 - 3.3.21.
    // For information on DMASEV, see PL330, section 4.3.15.

    alt_write_word(ALT_DMA_DBGINST0_ADDR(ALT_DMASECURE_ADDR),
                   ALT_DMA_DBGINST0_INSTRUCTIONBYTE0_SET(0x34) | // opcode for DMASEV
                   ALT_DMA_DBGINST0_INSTRUCTIONBYTE1_SET(evt_num << 3) |
                   ALT_DMA_DBGINST0_DEBUGTHREAD_SET(ALT_DMA_DBGINST0_DEBUGTHREAD_E_MANAGER)
        );

    // Execute the instruction held in DBGINST0

    // For information on DBGCMD, see PL330, section 3.3.19.

    alt_write_word(ALT_DMA_DBGCMD_ADDR(ALT_DMASECURE_ADDR), 0);

    return ALT_E_SUCCESS;
}

s32 alt_dma_manager_state_get(ALT_DMA_MANAGER_STATE_t * state)
{
    // For information on DSR, see PL330, section 3.3.1.

    u32 raw_state = alt_read_word(ALT_DMA_DSR_ADDR(ALT_DMASECURE_ADDR));

    *state = (ALT_DMA_MANAGER_STATE_t)ALT_DMA_DSR_DMASTATUS_GET(raw_state);

    return ALT_E_SUCCESS;
}

s32 alt_dma_channel_state_get(ALT_DMA_CHANNEL_t channel, ALT_DMA_CHANNEL_STATE_t * state)
{
    // Validate channel
    switch (channel)
    {
    case ALT_DMA_CHANNEL_0:
    case ALT_DMA_CHANNEL_1:
    case ALT_DMA_CHANNEL_2:
    case ALT_DMA_CHANNEL_3:
    case ALT_DMA_CHANNEL_4:
    case ALT_DMA_CHANNEL_5:
    case ALT_DMA_CHANNEL_6:
    case ALT_DMA_CHANNEL_7:
        break;
    default:
        return ALT_E_BAD_ARG;
    }

    // For information on CSR, see PL330, section 3.3.11.

    u32 raw_state = alt_read_word(ALT_DMA_CSRx_ADDR(ALT_DMASECURE_ADDR, channel));

    *state = (ALT_DMA_CHANNEL_STATE_t)ALT_DMA_CSRx_CHANNELSTATUS_GET(raw_state);

    return ALT_E_SUCCESS;
}

s32 alt_dma_manager_fault_status_get(ALT_DMA_MANAGER_FAULT_t * fault)
{
    // For information on FTRD, see PL330, section 3.3.9.

    *fault = (ALT_DMA_MANAGER_FAULT_t)alt_read_word(ALT_DMA_FTRD_ADDR(ALT_DMASECURE_ADDR));

    return ALT_E_SUCCESS;
}

s32 alt_dma_channel_fault_status_get(ALT_DMA_CHANNEL_t channel,
                                                 ALT_DMA_CHANNEL_FAULT_t * fault)
{
    // Validate channel
    switch (channel)
    {
    case ALT_DMA_CHANNEL_0:
    case ALT_DMA_CHANNEL_1:
    case ALT_DMA_CHANNEL_2:
    case ALT_DMA_CHANNEL_3:
    case ALT_DMA_CHANNEL_4:
    case ALT_DMA_CHANNEL_5:
    case ALT_DMA_CHANNEL_6:
    case ALT_DMA_CHANNEL_7:
        break;
    default:
        return ALT_E_BAD_ARG;
    }

    // For information on FTR, see PL330, section 3.3.10.

    *fault = (ALT_DMA_CHANNEL_FAULT_t)alt_read_word(ALT_DMA_FTRx_ADDR(ALT_DMASECURE_ADDR, channel));

    return ALT_E_SUCCESS;
}

s32 alt_dma_event_int_select(ALT_DMA_EVENT_t evt_num,
                                         ALT_DMA_EVENT_SELECT_t opt)
{
    // Validate evt_num
    switch (evt_num)
    {
    case ALT_DMA_EVENT_0:
    case ALT_DMA_EVENT_1:
    case ALT_DMA_EVENT_2:
    case ALT_DMA_EVENT_3:
    case ALT_DMA_EVENT_4:
    case ALT_DMA_EVENT_5:
    case ALT_DMA_EVENT_6:
    case ALT_DMA_EVENT_7:
    case ALT_DMA_EVENT_ABORT:
        break;
    default:
        return ALT_E_BAD_ARG;
    }

    // For information on INTEN, see PL330, section 3.3.3.

    switch (opt)
    {
    case ALT_DMA_EVENT_SELECT_SEND_EVT:
        alt_clrbits_word(ALT_DMA_INTEN_ADDR(ALT_DMASECURE_ADDR), 1 << evt_num);
        break;
    case ALT_DMA_EVENT_SELECT_SIG_IRQ:
        alt_setbits_word(ALT_DMA_INTEN_ADDR(ALT_DMASECURE_ADDR), 1 << evt_num);
        break;
    default:
        return ALT_E_BAD_ARG;
    }

    return ALT_E_SUCCESS;
}

s32 alt_dma_int_clear(ALT_DMA_EVENT_t irq_num)
{
    // Validate evt_num
    switch (irq_num)
    {
    case ALT_DMA_EVENT_0:
    case ALT_DMA_EVENT_1:
    case ALT_DMA_EVENT_2:
    case ALT_DMA_EVENT_3:
    case ALT_DMA_EVENT_4:
    case ALT_DMA_EVENT_5:
    case ALT_DMA_EVENT_6:
    case ALT_DMA_EVENT_7:
    case ALT_DMA_EVENT_ABORT:
        break;
    default:
        return ALT_E_BAD_ARG;
    }

    // For information on INTCLR, see PL330, section 3.3.6.

    alt_write_word(ALT_DMA_INTCLR_ADDR(ALT_DMASECURE_ADDR), 1 << irq_num);

    return ALT_E_SUCCESS;
}

/*
   If acp is used, the source cache and destination cache must controlled by the dma channel control register.
   the source_cache ARCACHE[3..0] = b0111
   the destination_cache AWCACHE[3..0]=b1011
  */


s32 alt_dma_memory_to_memory(ALT_DMA_CHANNEL_t channel,
                                         ALT_DMA_PROGRAM_t * program,
                                         void * dst,
                                         const void * src,
                                         u32 size,
                                         bool send_evt,
                                         ALT_DMA_EVENT_t evt,
                                         bool use_acp)
{
    s32 status = ALT_E_SUCCESS;
    u32 arcache = ALT_DMA_CCR_OPT_SC_DEFAULT, awcache = ALT_DMA_CCR_OPT_DC_DEFAULT;

    if(use_acp)
    {
       arcache = ALT_DMA_CCR_OPT_SC(ALT_DMA_CCR_OPT_SC_WBRA);
	 awcache = ALT_DMA_CCR_OPT_DC(ALT_DMA_CCR_OPT_DC_WBWA);
    }

    // If the size is zero, and no event is requested, just return success.
    if ((size == 0) && (send_evt == false))
    {
        return status;
    }

    if (status == ALT_E_SUCCESS)
    {
        status = alt_dma_program_init(program);
    }

    if (size != 0)
    {
	u32 sizeleft;
	u32 udst = (u32)dst;
	u32 usrc = (u32)src;

        dprintf("DMA[M->M]: dst  = %p.\n", dst);
        dprintf("DMA[M->M]: src  = %p.\n", src);
        dprintf("DMA[M->M]: size = 0x%x.\n", size);
        
        // Detect if memory regions overshoots the address space.

        if (udst + size - 1 < udst)
        {
            return ALT_E_BAD_ARG;
        }
        if (usrc + size - 1 < usrc)
        {
            return ALT_E_BAD_ARG;
        }

        // Detect if memory regions overlaps.

        if (udst > usrc)
        {
            if (usrc + size - 1 > udst)
            {
                return ALT_E_BAD_ARG;
            }
        }
        else
        {
            if (udst + size - 1 > usrc)
            {
                return ALT_E_BAD_ARG;
            }
        }

        if (status == ALT_E_SUCCESS)
        {
            status = alt_dma_program_DMAMOV(program, ALT_DMA_PROGRAM_REG_SAR, usrc);
        }
        if (status == ALT_E_SUCCESS)
        {
            status = alt_dma_program_DMAMOV(program, ALT_DMA_PROGRAM_REG_DAR, udst);
        }

        sizeleft = size;

        //
        // The algorithm uses the strategy described in PL330 B.3.1.
        // It is extended for 2-byte and 1-byte unaligned cases.
        //

        // First see how many byte(s) we need to transfer to get src to be 8 byte aligned
        if (usrc & 0x7)
        {
            u32 aligncount = MIN(8 - (usrc & 0x7), sizeleft);
            sizeleft -= aligncount;

            dprintf("DMA[M->M]: Total pre-alignment 1-byte burst size tranfer(s): %d.\n", (unsigned)aligncount);

            // Program in the following parameters:
            //  - SS8 (Source      burst size of 1-byte)
            //  - DS8 (Destination burst size of 1-byte)
            //  - SBx (Source      burst length of [aligncount] transfers)
            //  - DBx (Destination burst length of [aligncount] transfers)
            //  - All other options default.

            if (status == ALT_E_SUCCESS)
            {
                status = alt_dma_program_DMAMOV(program, ALT_DMA_PROGRAM_REG_CCR,
                                                (   ((aligncount - 1) << 4) // SB
                                                  | ALT_DMA_CCR_OPT_SS8
                                                  | ALT_DMA_CCR_OPT_SA_DEFAULT
                                                  | ALT_DMA_CCR_OPT_SP_DEFAULT
                                                  | arcache
                                                  | ((aligncount - 1) << 18) // DB
                                                  | ALT_DMA_CCR_OPT_DS8
                                                  | ALT_DMA_CCR_OPT_DA_DEFAULT
                                                  | ALT_DMA_CCR_OPT_DP_DEFAULT
                                                  | awcache
                                                  | ALT_DMA_CCR_OPT_ES_DEFAULT
                                                )
                    );
            }
            if (status == ALT_E_SUCCESS)
            {
                status = alt_dma_program_DMALD(program, ALT_DMA_PROGRAM_INST_MOD_NONE);/* load data into MFIFO from src address */
            }
            if (status == ALT_E_SUCCESS)
            {
                status = alt_dma_program_DMAST(program, ALT_DMA_PROGRAM_INST_MOD_NONE);/* store MFIFO's data into dst address */
            }
        }

        // This is the number of 8-byte bursts
        u32 burstcount = sizeleft >> 3;

        bool correction = (burstcount != 0);

        // Update the size left to transfer
        sizeleft &= 0x7;

        dprintf("DMA[M->M]: Total Main 8-byte burst size transfer(s): %u.\n", (unsigned)burstcount);
        dprintf("DMA[M->M]: Total Main 1-byte burst size transfer(s): %u.\n", (unsigned)sizeleft);

        // Determine how many 16 length bursts can be done

        if (burstcount >> 4)
        {
            u32 length16burstcount = burstcount >> 4;
            burstcount &= 0xf;

            dprintf("DMA[M->M]:   Number of 16 burst length 8-byte transfer(s): %u.\n", (unsigned)length16burstcount);
            dprintf("DMA[M->M]:   Number of remaining 8-byte transfer(s):       %u.\n", (unsigned)burstcount);

            // Program in the following parameters:
            //  - SS64 (Source      burst size of 8-byte)
            //  - DS64 (Destination burst size of 8-byte)
            //  - SB16 (Source      burst length of 16 transfers)
            //  - DB16 (Destination burst length of 16 transfers)
            //  - All other options default.

            if (status == ALT_E_SUCCESS)
            {
                status = alt_dma_program_DMAMOV(program, ALT_DMA_PROGRAM_REG_CCR,
                                                (   ALT_DMA_CCR_OPT_SB16
                                                  | ALT_DMA_CCR_OPT_SS64
                                                  | ALT_DMA_CCR_OPT_SA_DEFAULT
                                                  | ALT_DMA_CCR_OPT_SP_DEFAULT
                                                  | arcache
                                                  | ALT_DMA_CCR_OPT_DB16
                                                  | ALT_DMA_CCR_OPT_DS64
                                                  | ALT_DMA_CCR_OPT_DA_DEFAULT
                                                  | ALT_DMA_CCR_OPT_DP_DEFAULT
                                                  | awcache
                                                  | ALT_DMA_CCR_OPT_ES_DEFAULT
                                                )
                    );
            }

            while (length16burstcount > 0)
            {
                if (status != ALT_E_SUCCESS)
                {
                    break;
                }

                u32 loopcount = MIN(length16burstcount, 256);
                length16burstcount -= loopcount;

                dprintf("DMA[M->M]:   Looping %ux 16 burst length 8-byte transfer(s).\n", (unsigned)loopcount);

                if ((status == ALT_E_SUCCESS) && (loopcount > 1))
                {
                    status = alt_dma_program_DMALP(program, loopcount);
                }
                if (status == ALT_E_SUCCESS)
                {
                    status = alt_dma_program_DMALD(program, ALT_DMA_PROGRAM_INST_MOD_NONE);
                }
                if (status == ALT_E_SUCCESS)
                {
                    status = alt_dma_program_DMAST(program, ALT_DMA_PROGRAM_INST_MOD_NONE);
                }
                if ((status == ALT_E_SUCCESS) && (loopcount > 1))
                {
                    status = alt_dma_program_DMALPEND(program, ALT_DMA_PROGRAM_INST_MOD_NONE);
                }
            }
        }

        // At this point, we should have [burstcount] 8-byte transfer(s)
        // remaining. [burstcount] should be less than 16.

        // Do one more burst with a SB / DB of length [burstcount].

        if (burstcount)
        {
            // Program in the following parameters:
            //  - SS64 (Source      burst size of 8-byte)
            //  - DS64 (Destination burst size of 8-byte)
            //  - SBx  (Source      burst length of [burstlength] transfers)
            //  - DBx  (Destination burst length of [burstlength] transfers)
            //  - All other options default.

            if (status == ALT_E_SUCCESS)
            {
                status = alt_dma_program_DMAMOV(program, ALT_DMA_PROGRAM_REG_CCR,
                                                (   ((burstcount - 1) << 4) // SB
                                                  | ALT_DMA_CCR_OPT_SS64
                                                  | ALT_DMA_CCR_OPT_SA_DEFAULT
                                                  | ALT_DMA_CCR_OPT_SP_DEFAULT
                                                  | arcache
                                                  | ((burstcount - 1) << 18) // DB
                                                  | ALT_DMA_CCR_OPT_DS64
                                                  | ALT_DMA_CCR_OPT_DA_DEFAULT
                                                  | ALT_DMA_CCR_OPT_DP_DEFAULT
                                                  | awcache
                                                  | ALT_DMA_CCR_OPT_ES_DEFAULT
                                                )
                    );
            }
            if (status == ALT_E_SUCCESS)
            {
                status = alt_dma_program_DMALD(program, ALT_DMA_PROGRAM_INST_MOD_NONE);
            }
            if (status == ALT_E_SUCCESS)
            {
                status = alt_dma_program_DMAST(program, ALT_DMA_PROGRAM_INST_MOD_NONE);
            }
        }

        // This is where the last DMAMOV CCR and DMAST is done if an
        // alignment correction required.

        if (   (correction == true)
            && ((usrc & 0x7) != (udst & 0x7)) // If src and dst are mod-8 congruent, no correction is needed.
           )
        {
            if (status == ALT_E_SUCCESS)
            {
                // Determine what type of correction.

                // Set the source parameters to match that of the destination
                // parameters. This way the SAR is increment in the same fashion as
                // DAR. This will allow the non 8-byte transfers to copy correctly.

                u32 ccr;

                if ((usrc & 0x3) == (udst & 0x3))
                {
                    dprintf("DMA[M->M]: Single correction 4-byte burst size tranfer.\n");

                    // Program in the following parameters:
                    //  - SS32 (Source      burst size of 4-byte)
                    //  - DS32 (Destination burst size of 4-byte)
                    //  - SB1  (Source      burst length of 1 transfer)
                    //  - DB1  (Destination burst length of 1 transfer)
                    //  - All other options default.

                    ccr = (   ALT_DMA_CCR_OPT_SB1
                            | ALT_DMA_CCR_OPT_SS32
                            | ALT_DMA_CCR_OPT_SA_DEFAULT
                            | ALT_DMA_CCR_OPT_SP_DEFAULT
                            | arcache
                            | ALT_DMA_CCR_OPT_DB1
                            | ALT_DMA_CCR_OPT_DS32
                            | ALT_DMA_CCR_OPT_DA_DEFAULT
                            | ALT_DMA_CCR_OPT_DP_DEFAULT
                            | awcache
                            | ALT_DMA_CCR_OPT_ES_DEFAULT
                          );
                }
                else if ((usrc & 0x1) == (udst & 0x1))
                {
                    dprintf("DMA[M->M]: Single correction 2-byte burst size tranfer.\n");

                    // Program in the following parameters:
                    //  - SS16 (Source      burst size of 2-byte)
                    //  - DS16 (Destination burst size of 2-byte)
                    //  - SB1  (Source      burst length of 1 transfer)
                    //  - DB1  (Destination burst length of 1 transfer)
                    //  - All other options default.

                    ccr = (   ALT_DMA_CCR_OPT_SB1
                            | ALT_DMA_CCR_OPT_SS16
                            | ALT_DMA_CCR_OPT_SA_DEFAULT
                            | ALT_DMA_CCR_OPT_SP_DEFAULT
                            | arcache
                            | ALT_DMA_CCR_OPT_DB1
                            | ALT_DMA_CCR_OPT_DS16
                            | ALT_DMA_CCR_OPT_DA_DEFAULT
                            | ALT_DMA_CCR_OPT_DP_DEFAULT
                            | awcache
                            | ALT_DMA_CCR_OPT_ES_DEFAULT
                          );
                }
                else
                {
                    dprintf("DMA[M->M]: Single correction 1-byte burst size tranfer.\n");

                    // Program in the following parameters:
                    //  - SS8 (Source      burst size of 1-byte)
                    //  - DS8 (Destination burst size of 1-byte)
                    //  - SB1 (Source      burst length of 1 transfer)
                    //  - DB1 (Destination burst length of 1 transfer)
                    //  - All other options default.

                    ccr = (   ALT_DMA_CCR_OPT_SB1
                            | ALT_DMA_CCR_OPT_SS8
                            | ALT_DMA_CCR_OPT_SA_DEFAULT
                            | ALT_DMA_CCR_OPT_SP_DEFAULT
                            | arcache
                            | ALT_DMA_CCR_OPT_DB1
                            | ALT_DMA_CCR_OPT_DS8
                            | ALT_DMA_CCR_OPT_DA_DEFAULT
                            | ALT_DMA_CCR_OPT_DP_DEFAULT
                            | awcache
                            | ALT_DMA_CCR_OPT_ES_DEFAULT
                          );
                }

                status = alt_dma_program_DMAMOV(program, ALT_DMA_PROGRAM_REG_CCR,
                                                ccr);
            }
            if (status == ALT_E_SUCCESS)
            {
                status = alt_dma_program_DMAST(program, ALT_DMA_PROGRAM_INST_MOD_NONE);
            }
        }

        // At this point, there should be 0 - 7 1-byte transfers remaining.

        if (sizeleft)
        {
            dprintf("DMA[M->M]: Total post 1-byte burst size tranfer(s): %d.\n", (unsigned)sizeleft);

            // Program in the following parameters:
            //  - SS8 (Source      burst size of 1-byte)
            //  - DS8 (Destination burst size of 1-byte)
            //  - SBx (Source      burst length of [sizeleft] transfers)
            //  - DBx (Destination burst length of [sizeleft] transfers)
            //  - All other options default.

            if (status == ALT_E_SUCCESS)
            {
                status = alt_dma_program_DMAMOV(program, ALT_DMA_PROGRAM_REG_CCR,
                                                (   ((sizeleft - 1) << 4) // SB
                                                  | ALT_DMA_CCR_OPT_SS8
                                                  | ALT_DMA_CCR_OPT_SA_DEFAULT
                                                  | ALT_DMA_CCR_OPT_SP_DEFAULT
                                                  | arcache
                                                  | ((sizeleft - 1) << 18) // DB
                                                  | ALT_DMA_CCR_OPT_DS8
                                                  | ALT_DMA_CCR_OPT_DA_DEFAULT
                                                  | ALT_DMA_CCR_OPT_DP_DEFAULT
                                                  | awcache
                                                  | ALT_DMA_CCR_OPT_ES_DEFAULT
                                                )
                    );
            }
            if (status == ALT_E_SUCCESS)
            {
                status = alt_dma_program_DMALD(program, ALT_DMA_PROGRAM_INST_MOD_NONE);
            }
            if (status == ALT_E_SUCCESS)
            {
                status = alt_dma_program_DMAST(program, ALT_DMA_PROGRAM_INST_MOD_NONE);
            }
        }
    } // if (size != 0)

    // Send event if requested.
    if (send_evt)
    {
    	  if (status == ALT_E_SUCCESS)
        {
            dprintf("DMA[M->M]: Wating AXI write data end ...\n");
            status = alt_dma_program_DMAWMB(program);
        }
        if (status == ALT_E_SUCCESS)
        {
            dprintf("DMA[M->M]: Adding event ...\n");
            status = alt_dma_program_DMASEV(program, evt);
        }
    }

    // Now that everything is done, end the program.
    if (status == ALT_E_SUCCESS)
    {
        status = alt_dma_program_DMAEND(program);
    }

    // If there was a problem assembling the program, clean up the buffer and exit.
    if (status != ALT_E_SUCCESS)
    {
        // Do not report the status for the clear operation. A failure should be
        // reported regardless of if the clear is successful.
        alt_dma_program_clear(program);
        return status;
    }

    // Execute the program on the given channel.
    return alt_dma_channel_exec(channel, program);
}


static s32 alt_dma_PERIPH_FPGA_0_to_memory_single(ALT_DMA_PROGRAM_t * program,
                                                      ALT_DMA_PERIPH_t periph,
                                                      u32 size)
{
    s32 status = ALT_E_SUCCESS;

    // Program in the following parameters:
    //  - SS8 (Source      burst size of 1-byte)
    //  - DS8 (Destination burst size of 1-byte)
    //  - SB1 (Source      burst length of 1 transfer)
    //  - DB1 (Destination burst length of 1 transfer)
    //  - SAF (Source      address fixed)
    //  - All other options default.

    if (status == ALT_E_SUCCESS)
    {
        status = alt_dma_program_DMAMOV(program, ALT_DMA_PROGRAM_REG_CCR,
                                        (   ALT_DMA_CCR_OPT_SB1
                                          | ALT_DMA_CCR_OPT_SS8
                                          | ALT_DMA_CCR_OPT_SAF
                                          | ALT_DMA_CCR_OPT_SP_DEFAULT
                                          | ALT_DMA_CCR_OPT_SC_DEFAULT
                                          | ALT_DMA_CCR_OPT_DB1
                                          | ALT_DMA_CCR_OPT_DS8
                                          | ALT_DMA_CCR_OPT_DA_DEFAULT
                                          | ALT_DMA_CCR_OPT_DP_DEFAULT
                                          | ALT_DMA_CCR_OPT_DC_DEFAULT
                                          | ALT_DMA_CCR_OPT_ES_DEFAULT
                                        )
            );
    }

    u32 sizeleft = size;

    while (sizeleft > 0)
    {
        if (status != ALT_E_SUCCESS)
        {
            break;
        }

        u32 loopcount = MIN(sizeleft, 256);
        sizeleft -= loopcount;

        dprintf("DMA[P->M][S]: Creating loop for %u transfer(s).\n", (unsigned)loopcount);
		bmlog("DMA[P->M][S]: Creating loop for %u transfer(s).\n", (unsigned)loopcount);

        if ((status == ALT_E_SUCCESS) && (loopcount > 1))
        {
            status = alt_dma_program_DMALP(program, loopcount);
        }
        if (status == ALT_E_SUCCESS)
        {
            status = alt_dma_program_DMAFLUSHP(program, periph);
        }
        if (status == ALT_E_SUCCESS)
        {
            status = alt_dma_program_DMAWFP(program, periph, ALT_DMA_PROGRAM_INST_MOD_SINGLE);
        }
        if (status == ALT_E_SUCCESS)
        {
            status = alt_dma_program_DMALD(program, ALT_DMA_PROGRAM_INST_MOD_SINGLE);
        }
        if (status == ALT_E_SUCCESS)
        {
            status = alt_dma_program_DMAST(program, ALT_DMA_PROGRAM_INST_MOD_SINGLE);
        }
        if ((status == ALT_E_SUCCESS) && (loopcount > 1))
        {
            status = alt_dma_program_DMALPEND(program, ALT_DMA_PROGRAM_INST_MOD_SINGLE);
        }
    }

    return status;
}                                              


static s32 alt_dma_PERIPH_FPGA_0_to_memory(ALT_DMA_PROGRAM_t * program,
                                               ALT_DMA_PERIPH_t periph,
                                               ALT_PERIPH_FPGA_HANDLE_t * handle,
                                               void * dst,
                                               u32 size)
{
    s32 status = ALT_E_SUCCESS;

    if (status == ALT_E_SUCCESS)
    {
        status = alt_dma_program_DMAMOV(program, ALT_DMA_PROGRAM_REG_DAR, (u32)dst);
    }
    if (status == ALT_E_SUCCESS)
    {
        status = alt_dma_program_DMAMOV(program, ALT_DMA_PROGRAM_REG_SAR, (u32)(handle->location));
		//status = alt_dma_program_DMAMOV(program, ALT_DMA_PROGRAM_REG_SAR, (u32)0xff709000);
    }

    // Determine if FIFOs are enabled from the FCR cache
#if 0
    if (ALT_UART_FCR_FIFOE_GET(handle->fcr) != 0)
    {
        dprintf("DMA[P->M][16550]: FIFOs enabled.\n");

        //
        // FIFOs are enabled.
        //

        u32 rx_size;
        u32 burst_size;
        ALT_16550_FIFO_TRIGGER_RX_t trig_rx;

        // Get the RX FIFO Size
        // Use the register interface to avoid coupling the 16550 and DMA.
        rx_size = ALT_UART_CPR_FIFO_MOD_GET(alt_read_word(ALT_UART_CPR_ADDR(handle->location))) << 4;

        // Get the RX FIFO Trigger Level from the FCR cache
        trig_rx = (ALT_16550_FIFO_TRIGGER_RX_t)ALT_UART_FCR_RT_GET(handle->fcr);

        switch (trig_rx)
        {
        case ALT_16550_FIFO_TRIGGER_RX_ANY:
            burst_size = 1;
            break;
        case ALT_16550_FIFO_TRIGGER_RX_QUARTER_FULL:
            burst_size = rx_size >> 2; // divide by 4
            break;
        case ALT_16550_FIFO_TRIGGER_RX_HALF_FULL:
            burst_size = rx_size >> 1; // divide by 2
            break;
        case ALT_16550_FIFO_TRIGGER_RX_ALMOST_FULL:
            burst_size = rx_size - 2;
            break;
        default:
            // This case should never happen.
            return ALT_E_ERROR;
        }

        if (burst_size < 16)
        {
            // There's no point bursting 1 byte at a time per notify, so just do single transfers.
            if (status == ALT_E_SUCCESS)
            {
                status = alt_dma_16550_to_memory_single(program,
                                                        periph,
                                                        size);
            }
        }
        else
        {
            u32 sizeleft = size;

            // Now trim the burst size to a multiple of 16.
            // This will optimize the bursting in the fewest possible commands.
            dprintf("DMA[P->M][16550]: Untrimmed burst size = %u.\n", (unsigned)burst_size);
            burst_size &= ~0xf;
            dprintf("DMA[P->M][16550]: Trimmed burst size   = %u.\n", (unsigned)burst_size);

            // Determine how many burst transfers can be done
            u32 burst_count = 0;

            burst_count = sizeleft / burst_size;
            sizeleft -= burst_count * burst_size;

            if (burst_count == 0)
            {
                // Do the transfer.
                if (status == ALT_E_SUCCESS)
                {
                    status = alt_dma_16550_to_memory_single(program,
                                                            periph,
                                                            sizeleft);
                }
            }
            else
            {
                // Do the burst transfers
                if (status == ALT_E_SUCCESS)
                {
                    status = alt_dma_16550_to_memory_burst(program,
                                                           periph,
                                                           burst_size,
                                                           burst_count);
                }

                // Program the DMA engine to transfer the non-burstable items in single transfers.
                if (status == ALT_E_SUCCESS)
                {
                    status = alt_dma_16550_to_memory_single(program,
                                                            periph,
                                                            sizeleft);
                }

            } // if (burst_count == 0)
        }
    }
    else
#endif
    {
        dprintf("DMA[P->M][fpga2arm]: FIFOs disabled.\n");
		bmlog("DMA[P->M][fpga2arm]: FIFOs disabled.\n");

        //
        // FIFOs are disabled.
        //

        status = alt_dma_PERIPH_FPGA_0_to_memory_single(program,
                                                periph,
                                                size);
    }

    return status;
}



s32 alt_dma_periph_to_memory(ALT_DMA_CHANNEL_t channel,
                                         ALT_DMA_PROGRAM_t * program,
                                         void * dst,
                                         ALT_DMA_PERIPH_t srcp,
                                         u32 size,
                                         void * periph_info,
                                         bool send_evt,
                                         ALT_DMA_EVENT_t evt)
{
    s32 status = ALT_E_SUCCESS;

    if ((size == 0) && (send_evt == false))
    {
        return ALT_E_SUCCESS;
    }

    if (status == ALT_E_SUCCESS)
    {
        dprintf("DMA[P->M]: Init Program.\n");
		bmlog("DMA[P->M]: Init Program.\n");
        status = alt_dma_program_init(program);
    }

    if ((status == ALT_E_SUCCESS) && (size != 0))
    {
        switch (srcp)
        {
        case ALT_DMA_PERIPH_UART0_RX:
        case ALT_DMA_PERIPH_UART1_RX:
            break;
        case ALT_DMA_PERIPH_FPGA_0:
			status = alt_dma_PERIPH_FPGA_0_to_memory(program, srcp,
                                             (ALT_PERIPH_FPGA_HANDLE_t *)periph_info, dst, size);
			break;
        case ALT_DMA_PERIPH_FPGA_1:
        case ALT_DMA_PERIPH_FPGA_2:
        case ALT_DMA_PERIPH_FPGA_3:
        case ALT_DMA_PERIPH_FPGA_4:
        case ALT_DMA_PERIPH_FPGA_5:
        case ALT_DMA_PERIPH_FPGA_6:
        case ALT_DMA_PERIPH_FPGA_7:
        default:
            status = ALT_E_BAD_ARG;
            break;
        }
    }

    // Send event if requested.
    if (send_evt)
    {
        if (status == ALT_E_SUCCESS)
        {
            dprintf("DMA[P->M]: Adding event.\n");
			bmlog("DMA[P->M]: Adding event.\n");
            status = alt_dma_program_DMASEV(program, evt);
        }
    }


    // Now that everything is done, end the program.
    if (status == ALT_E_SUCCESS)
    {
        status = alt_dma_program_DMAEND(program);
    }

    // If there was a problem assembling the program, clean up the buffer and exit.
    if (status != ALT_E_SUCCESS)
    {
        // Do not report the status for the clear operation. A failure should be
        // reported regardless of if the clear is successful.
        alt_dma_program_clear(program);
        return status;
    }

    // Execute the program on the given channel.
	bmlog("fpga2arm alt_dma_channel_exec\n");
    return alt_dma_channel_exec(channel, program);
}
























