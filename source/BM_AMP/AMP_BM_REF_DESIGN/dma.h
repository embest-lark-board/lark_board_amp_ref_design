#ifndef __DMA_H
#define __DMA_H

/*****************************************************************************************************************/
#define ALT_DMA_PROGRAM_CACHE_LINE_SIZE     (32)
#define ALT_DMA_PROGRAM_CACHE_LINE_COUNT    (16)

#ifndef ALT_DMA_PROGRAM_PROVISION_BUFFER_SIZE
#define ALT_DMA_PROGRAM_PROVISION_BUFFER_SIZE   (ALT_DMA_PROGRAM_CACHE_LINE_SIZE * ALT_DMA_PROGRAM_CACHE_LINE_COUNT)
#endif


/*!
 * This type enumerates the DMA security state options available.
 */
typedef enum ALT_DMA_SECURITY_e
{
    ALT_DMA_SECURITY_DEFAULT   = 0, /*!< Use the default security value (e.g. reset default) */
    ALT_DMA_SECURITY_SECURE    = 1, /*!< Secure */
    ALT_DMA_SECURITY_NONSECURE = 2  /*!< Non-secure */
}
ALT_DMA_SECURITY_t;

/*!
 * This type definition enumerates the SoC system peripherals implementing the
 * required request interface that enables direct DMA transfers to/from the
 * device.
 *
 * FPGA soft IP interface to the DMA are required to comply with the Synopsys
 * protocol.
 *
 * Request interface numbers 4 through 7 are multiplexed between the CAN
 * controllers and soft logic implemented in the FPGA fabric. The selection
 * between the CAN controller and FPGA interfaces is determined at DMA
 * initialization.
 */
typedef enum ALT_DMA_PERIPH_e
{
    ALT_DMA_PERIPH_FPGA_0             = 0,  /*!< FPGA soft IP interface 0 */
    ALT_DMA_PERIPH_FPGA_1             = 1,  /*!< FPGA soft IP interface 1 */
    ALT_DMA_PERIPH_FPGA_2             = 2,  /*!< FPGA soft IP interface 2 */
    ALT_DMA_PERIPH_FPGA_3             = 3,  /*!< FPGA soft IP interface 3 */

    ALT_DMA_PERIPH_FPGA_4_OR_CAN0_IF1 = 4,  /*!< Selectively MUXed FPGA 4 or CAN 0 interface 1 */
    ALT_DMA_PERIPH_FPGA_5_OR_CAN0_IF2 = 5,  /*!< Selectively MUXed FPGA 5 or CAN 0 interface 2 */
    ALT_DMA_PERIPH_FPGA_6_OR_CAN1_IF1 = 6,  /*!< Selectively MUXed FPGA 6 or CAN 1 interface 1 */
    ALT_DMA_PERIPH_FPGA_7_OR_CAN1_IF2 = 7,  /*!< Selectively MUXed FPGA 7 or CAN 1 interface 2 */

    ALT_DMA_PERIPH_FPGA_4             = 4,  /*!< Alias for ALT_DMA_PERIPH_FPGA_4_OR_CAN0_IF1 */
    ALT_DMA_PERIPH_FPGA_5             = 5,  /*!< Alias for ALT_DMA_PERIPH_FPGA_5_OR_CAN0_IF2 */
    ALT_DMA_PERIPH_FPGA_6             = 6,  /*!< Alias for ALT_DMA_PERIPH_FPGA_6_OR_CAN1_IF1 */
    ALT_DMA_PERIPH_FPGA_7             = 7,  /*!< Alias for ALT_DMA_PERIPH_FPGA_7_OR_CAN1_IF2 */

    ALT_DMA_PERIPH_CAN0_IF1           = 4,  /*!< Alias for ALT_DMA_PERIPH_FPGA_4_OR_CAN0_IF1 */
    ALT_DMA_PERIPH_CAN0_IF2           = 5,  /*!< Alias for ALT_DMA_PERIPH_FPGA_5_OR_CAN0_IF2 */
    ALT_DMA_PERIPH_CAN1_IF1           = 6,  /*!< Alias for ALT_DMA_PERIPH_FPGA_6_OR_CAN1_IF1 */
    ALT_DMA_PERIPH_CAN1_IF2           = 7,  /*!< Alias for ALT_DMA_PERIPH_FPGA_7_OR_CAN1_IF2 */

    ALT_DMA_PERIPH_I2C0_TX            = 8,  /*!< I<sup>2</sup>C 0 TX */
    ALT_DMA_PERIPH_I2C0_RX            = 9,  /*!< I<sup>2</sup>C 0 RX */
    ALT_DMA_PERIPH_I2C1_TX            = 10, /*!< I<sup>2</sup>C 1 TX */
    ALT_DMA_PERIPH_I2C1_RX            = 11, /*!< I<sup>2</sup>C 1 RX */
    ALT_DMA_PERIPH_I2C2_TX            = 12, /*!< I<sup>2</sup>C 2 TX */
    ALT_DMA_PERIPH_I2C2_RX            = 13, /*!< I<sup>2</sup>C 2 RX */
    ALT_DMA_PERIPH_I2C3_TX            = 14, /*!< I<sup>2</sup>C 3 TX */
    ALT_DMA_PERIPH_I2C3_RX            = 15, /*!< I<sup>2</sup>C 3 RX */
    ALT_DMA_PERIPH_SPI0_MASTER_TX     = 16, /*!< SPI 0 Master TX */
    ALT_DMA_PERIPH_SPI0_MASTER_RX     = 17, /*!< SPI 0 Master RX */
    ALT_DMA_PERIPH_SPI0_SLAVE_TX      = 18, /*!< SPI 0 Slave TX */
    ALT_DMA_PERIPH_SPI0_SLAVE_RX      = 19, /*!< SPI 0 Slave RX */
    ALT_DMA_PERIPH_SPI1_MASTER_TX     = 20, /*!< SPI 1 Master TX */
    ALT_DMA_PERIPH_SPI1_MASTER_RX     = 21, /*!< SPI 1 Master RX */
    ALT_DMA_PERIPH_SPI1_SLAVE_TX      = 22, /*!< SPI 1 Slave TX */
    ALT_DMA_PERIPH_SPI1_SLAVE_RX      = 23, /*!< SPI 1 Slave RX */
    ALT_DMA_PERIPH_QSPI_FLASH_TX      = 24, /*!< QSPI Flash TX */
    ALT_DMA_PERIPH_QSPI_FLASH_RX      = 25, /*!< QSPI Flash RX */
    ALT_DMA_PERIPH_STM                = 26, /*!< System Trace Macrocell */
    ALT_DMA_PERIPH_RESERVED           = 27, /*!< Reserved */
    ALT_DMA_PERIPH_UART0_TX           = 28, /*!< UART 0 TX */
    ALT_DMA_PERIPH_UART0_RX           = 29, /*!< UART 0 RX */
    ALT_DMA_PERIPH_UART1_TX           = 30, /*!< UART 1 TX */
    ALT_DMA_PERIPH_UART1_RX           = 31  /*!< UART 1 RX */
}
ALT_DMA_PERIPH_t;

/*!
 * This type definition enumerates the DMA event-interrupt resources.
 */
typedef enum ALT_DMA_EVENT_e
{
    ALT_DMA_EVENT_0     = 0, /*!< DMA Event 0 */
    ALT_DMA_EVENT_1     = 1, /*!< DMA Event 1 */
    ALT_DMA_EVENT_2     = 2, /*!< DMA Event 2 */
    ALT_DMA_EVENT_3     = 3, /*!< DMA Event 3 */
    ALT_DMA_EVENT_4     = 4, /*!< DMA Event 4 */
    ALT_DMA_EVENT_5     = 5, /*!< DMA Event 5 */
    ALT_DMA_EVENT_6     = 6, /*!< DMA Event 6 */
    ALT_DMA_EVENT_7     = 7, /*!< DMA Event 7 */
    ALT_DMA_EVENT_ABORT = 8  /*!< DMA Abort Event */
}
ALT_DMA_EVENT_t;


/*****************************************************************************************************************************/
typedef enum ALT_DMA_MANAGER_STATE_e
{
    ALT_DMA_MANAGER_STATE_STOPPED     = 0, /*!< Stopped */
    ALT_DMA_MANAGER_STATE_EXECUTING   = 1, /*!< Executing */
    ALT_DMA_MANAGER_STATE_CACHE_MISS  = 2, /*!< Cache Miss */
    ALT_DMA_MANAGER_STATE_UPDATING_PC = 3, /*!< Updating PC */
    ALT_DMA_MANAGER_STATE_WFE         = 4, /*!< Waiting for Event */
    ALT_DMA_MANAGER_STATE_FAULTING    = 15 /*!< Faulting */
}
ALT_DMA_MANAGER_STATE_t;

/*!
 * This type definition enumerates the operational states that a DMA channel
 * may have.
 */
typedef enum ALT_DMA_CHANNEL_STATE_e
{
    ALT_DMA_CHANNEL_STATE_STOPPED             = 0,  /*!< Stopped */
    ALT_DMA_CHANNEL_STATE_EXECUTING           = 1,  /*!< Executing */
    ALT_DMA_CHANNEL_STATE_CACHE_MISS          = 2,  /*!< Cache Miss */
    ALT_DMA_CHANNEL_STATE_UPDATING_PC         = 3,  /*!< Updating PC */
    ALT_DMA_CHANNEL_STATE_WFE                 = 4,  /*!< Waiting for Event */
    ALT_DMA_CHANNEL_STATE_AT_BARRIER          = 5,  /*!< At Barrier */
    ALT_DMA_CHANNEL_STATE_WFP                 = 7,  /*!< Waiting for Peripheral */
    ALT_DMA_CHANNEL_STATE_KILLING             = 8,  /*!< Killing */
    ALT_DMA_CHANNEL_STATE_COMPLETING          = 9,  /*!< Completing */
    ALT_DMA_CHANNEL_STATE_FAULTING_COMPLETING = 14, /*!< Faulting Completing */
    ALT_DMA_CHANNEL_STATE_FAULTING            = 15  /*!< Faulting */
}
ALT_DMA_CHANNEL_STATE_t;

/*!
 * This type definition enumerates the possible fault status that the DMA
 * manager can have as a register mask.
 */
typedef enum ALT_DMA_MANAGER_FAULT_e
{
    /*!
     * The DMA manager abort occured because of an instruction issued through
     * the debug interface.
     */
    ALT_DMA_MANAGER_FAULT_DBG_INSTR       = (s32)(1UL << 30),

    /*!
     * The DMA manager instruction fetch AXI bus response was not OKAY.
     */
    ALT_DMA_MANAGER_FAULT_INSTR_FETCH_ERR = (s32)(1UL << 16),

    /*!
     * The DMA manager attempted to execute DMAWFE or DMASEV with
     * inappropriate security permissions.
     */
    ALT_DMA_MANAGER_FAULT_MGR_EVNT_ERR    = (s32)(1UL <<  5),

    /*!
     * The DMA manager attempted to execute DMAGO with inappropriate security
     * permissions.
     */
    ALT_DMA_MANAGER_FAULT_DMAGO_ERR       = (s32)(1UL <<  4),

    /*!
     * The DMA manager attempted to execute an instruction operand that was
     * not valid for the DMA configuration.
     */
    ALT_DMA_MANAGER_FAULT_OPERAND_INVALID = (s32)(1UL <<  1),

    /*!
     * The DMA manager attempted to execute an undefined instruction.
     */
    ALT_DMA_MANAGER_FAULT_UNDEF_INSTR     = (s32)(1UL <<  0)
}
ALT_DMA_MANAGER_FAULT_t;

/*!
 * This type definition enumerates the possible fault status that a channel
 * may have as a register mask.
 */
typedef enum ALT_DMA_CHANNEL_FAULT_e
{
    /*!
     * The DMA channel has locked up due to resource starvation.
     */
    ALT_DMA_CHANNEL_FAULT_LOCKUP_ERR          = (s32)(1UL << 31),

    /*!
     * The DMA channel abort occured because of an instruction issued through
     * the debug interface.
     */
    ALT_DMA_CHANNEL_FAULT_DBG_INSTR           = (s32)(1UL << 30),

    /*!
     * The DMA channel data read AXI bus reponse was not OKAY.
     */
    ALT_DMA_CHANNEL_FAULT_DATA_READ_ERR       = (s32)(1UL << 18),

    /*!
     * The DMA channel data write AXI bus response was not OKAY.
     */
    ALT_DMA_CHANNEL_FAULT_DATA_WRITE_ERR      = (s32)(1UL << 17),

    /*!
     * The DMA channel instruction fetch AXI bus response was not OKAY.
     */
    ALT_DMA_CHANNEL_FAULT_INSTR_FETCH_ERR     = (s32)(1UL << 16),

    /*!
     * The DMA channel MFIFO did not have the data for the DMAST instruction.
     */
    ALT_DMA_CHANNEL_FAULT_ST_DATA_UNAVAILABLE = (s32)(1UL << 13),

    /*!
     * The DMA channel MFIFO is too small to hold the DMALD instruction data,
     * or too small to servic the DMAST instruction request.
     */
    ALT_DMA_CHANNEL_FAULT_MFIFO_ERR           = (s32)(1UL << 12),

    /*!
     * The DMA channel in non-secure state attempted to perform a secure read
     * or write.
     */
    ALT_DMA_CHANNEL_FAULT_CH_RDWR_ERR         = (s32)(1UL <<  7),

    /*!
     * The DMA channel in non-secure state attempted to execute the DMAWFP,
     * DMALDP, DMASTP, or DMAFLUSHP instruction involving a secure peripheral.
     */
    ALT_DMA_CHANNEL_FAULT_CH_PERIPH_ERR       = (s32)(1UL <<  6),

    /*!
     * The DMA channel in non-secure state attempted to execute the DMAWFE or
     * DMASEV instruction for a secure event or secure interrupt (if
     * applicable).
     */
    ALT_DMA_CHANNEL_FAULT_CH_EVNT_ERR         = (s32)(1UL <<  5),

    /*!
     * The DMA channel attempted to execute an instruction operand that was
     * not valid for the DMA configuration.
     */
    ALT_DMA_CHANNEL_FAULT_OPERAND_INVALID     = (s32)(1UL <<  1),

    /*!
     * The DMA channel attempted to execute an undefined instruction.
     */
    ALT_DMA_CHANNEL_FAULT_UNDEF_INSTR         = (s32)(1UL <<  0)
}
ALT_DMA_CHANNEL_FAULT_t;

/*!
 * This type definition enumerates the possible DMA event-interrupt behavior
 * option selections when a DMASEV instruction is executed.
 */
typedef enum ALT_DMA_EVENT_SELECT_e
{
    /*!
     * If the DMA controller executes DMASEV for the event-interrupt resource
     * then the DMA sends the event to all of the channel threads.
     */
    ALT_DMA_EVENT_SELECT_SEND_EVT,

    /*!
     * If the DMA controller executes DMASEV for the event-interrupt resource
     * then the DMA sets the \b irq[N] HIGH.
     */
    ALT_DMA_EVENT_SELECT_SIG_IRQ
}
ALT_DMA_EVENT_SELECT_t;

/*!
 * This type enumerates the DMA peripheral interface MUX selection options
 * available.
 */
typedef enum ALT_DMA_PERIPH_MUX_e
{
    /*! 
     * Accept the reset default MUX selection
     */ 
    ALT_DMA_PERIPH_MUX_DEFAULT = 0,

    /*!
     * Select FPGA as the peripheral interface
     */
    ALT_DMA_PERIPH_MUX_FPGA    = 1,

    /*!
     * Select CAN as the peripheral interface
     */
    ALT_DMA_PERIPH_MUX_CAN     = 2
}
ALT_DMA_PERIPH_MUX_t;


/*!
 * This type defines the structure used to specify the configuration of the
 * security states and peripheral interface MUX selections for the DMA
 * controller.
 */
typedef struct ALT_DMA_CFG_s
{
    /*!
     * DMA Manager security state configuration.
     */
    ALT_DMA_SECURITY_t manager_sec;

    /*!
     * DMA interrupt output security state configurations. Security state
     * configurations are 0-based index-aligned with the enumeration values
     * ALT_DMA_EVENT_0 through ALT_DMA_EVENT_7 of the ALT_DMA_EVENT_t type.
     */
    ALT_DMA_SECURITY_t irq_sec[8];

    /*!
     * Peripheral request interface security state configurations. Security
     * state configurations are 0-based index-aligned with the enumeration
     * values of the ALT_DMA_PERIPH_t type.
     */
    ALT_DMA_SECURITY_t periph_sec[32];

    /*!
     * DMA Peripheral Register Interface MUX Selections. MUX selections are
     * 0-based index-aligned with the enumeration values
     * ALT_DMA_PERIPH_FPGA_4_OR_CAN0_IF1 through
     * ALT_DMA_PERIPH_FPGA_7_OR_CAN1_IF2 of the ALT_DMA_PERIPH_t type.
     */
    ALT_DMA_PERIPH_MUX_t periph_mux[4];
}
ALT_DMA_CFG_t;


/*!
 * \addtogroup ALT_DMA_COMMON DMA Controller Common API Definitions
 *
 * This module contains the common definitions for the DMA controller related
 * APIs.
 *
 * @{
 */

/*!
 * This type definition enumerates the DMA controller channel threads.
 */
typedef enum ALT_DMA_CHANNEL_e
{
    ALT_DMA_CHANNEL_0 = 0, /*!< DMA Channel Thread 0 */
    ALT_DMA_CHANNEL_1 = 1, /*!< DMA Channel Thread 1 */
    ALT_DMA_CHANNEL_2 = 2, /*!< DMA Channel Thread 2 */
    ALT_DMA_CHANNEL_3 = 3, /*!< DMA Channel Thread 3 */
    ALT_DMA_CHANNEL_4 = 4, /*!< DMA Channel Thread 4 */
    ALT_DMA_CHANNEL_5 = 5, /*!< DMA Channel Thread 5 */
    ALT_DMA_CHANNEL_6 = 6, /*!< DMA Channel Thread 6 */
    ALT_DMA_CHANNEL_7 = 7  /*!< DMA Channel Thread 7 */
}
ALT_DMA_CHANNEL_t;

typedef struct ALT_DMA_PROGRAM_s
{
    u32 flag;

    u16 buffer_start;
    u16 code_size;

    u16 loop0;
    u16 loop1;

    u16 sar;
    u16 dar;

    /*
     * Add a little extra space so that regardless of where this structure
     * sits in memory, a suitable start address can be aligned to the cache
     * line stride while providing the requested buffer space.
     */
    u8 program[ALT_DMA_PROGRAM_PROVISION_BUFFER_SIZE +
                    ALT_DMA_PROGRAM_CACHE_LINE_SIZE];
    u32 program_phy;
}
ALT_DMA_PROGRAM_t;

/*! The operation was successful. */
#define ALT_E_SUCCESS               	(0)
#define ALT_E_ERROR				(-1)
#define ALT_E_ARG_RANGE		(-8)
#define ALT_E_BAD_ARG			(-9)
#define ALT_E_BAD_OPERATION     (-10)
#define ALT_E_TMO				(-12)
#define ALT_E_BUF_OVF               	(-20)


s32 alt_dma_init(const ALT_DMA_CFG_t * dma_cfg);
s32 alt_dma_uninit(void);
s32 alt_dma_channel_kill(ALT_DMA_CHANNEL_t channel);
s32 alt_dma_channel_alloc(ALT_DMA_CHANNEL_t channel);
s32 alt_dma_channel_free(ALT_DMA_CHANNEL_t channel);
s32 alt_dma_channel_state_get(ALT_DMA_CHANNEL_t channel,
                                          ALT_DMA_CHANNEL_STATE_t * state);
s32 alt_dma_channel_exec(ALT_DMA_CHANNEL_t channel,
                                     ALT_DMA_PROGRAM_t * pgm);
s32 alt_dma_memory_to_memory(ALT_DMA_CHANNEL_t channel,
                                         ALT_DMA_PROGRAM_t * program,
                                         void * dest,
                                         const void * src,
                                         u32 size,
                                         bool send_evt,
                                         ALT_DMA_EVENT_t evt,
                                         bool use_acp);

s32 alt_dma_int_clear(ALT_DMA_EVENT_t irq_num);
s32 alt_dma_event_int_select(ALT_DMA_EVENT_t evt_num,
                                         ALT_DMA_EVENT_SELECT_t opt);
s32 alt_dma_periph_to_memory(ALT_DMA_CHANNEL_t channel,
                                         ALT_DMA_PROGRAM_t * program,
                                         void * dst,
                                         ALT_DMA_PERIPH_t srcp,
                                         u32 size,
                                         void * periph_info,
                                         bool send_evt,
                                         ALT_DMA_EVENT_t evt);
                                     
#endif