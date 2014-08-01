#ifndef __DMA_INTERNEL_H
#define __DMA_INTERNEL_H

#include <types.h>
#include <dma.h>

typedef enum ALT_DMA_PROGRAM_REG_e
{
    /*! Source Address Register */
    ALT_DMA_PROGRAM_REG_SAR = 0x0,

    /*! Destination Address Register */
    ALT_DMA_PROGRAM_REG_DAR = 0x2,

    /*! Channel Control Register */
    ALT_DMA_PROGRAM_REG_CCR = 0x1
}
ALT_DMA_PROGRAM_REG_t;


typedef enum ALT_DMA_PROGRAM_INST_MOD_e
{
    /*!
     * This DMA instruction modifier specifies that no special modifier is
     * added to the instruction.
     */
    ALT_DMA_PROGRAM_INST_MOD_NONE,

    /*!
     * Depending on the DMA microcode instruction modified, this modifier
     * specifies <b>S</b> case for a <b>[S|B]</b> or a <b>\<single></b> for a
     * <b>\<single|burst|periph></b>.
     */
    ALT_DMA_PROGRAM_INST_MOD_SINGLE,

    /*!
     * Depending on the DMA microcode instruction modified, this modifier
     * specifies <b>B</b> case for a <b>[S|B]</b> or a <b>\<burst></b> for a
     * <b>\<single|burst|periph></b>.
     */
    ALT_DMA_PROGRAM_INST_MOD_BURST,

    /*!
     * This DMA instruction modifier specifies a <b>\<periph></b> for a
     * <b>\<single|burst|periph></b>.
     */
    ALT_DMA_PROGRAM_INST_MOD_PERIPH
}
ALT_DMA_PROGRAM_INST_MOD_t;

/*!
 * This function initializes a system memory buffer for use as a DMA microcode
 * program buffer. This should be the first API call made on the program
 * buffer type.
 *
 * \param       pgm
 *              A pointer to a DMA program buffer structure.
 *
 * \retval      ALT_E_SUCCESS   The operation was successful.
 * \retval      ALT_E_ERROR     Details about error status code
 */
s32 alt_dma_program_init(ALT_DMA_PROGRAM_t * pgm);

/*!
 * This function verifies that the DMA microcode program buffer is no longer
 * in use and performs any needed uninitialization steps.
 *
 * \param       pgm
 *              A pointer to a DMA program buffer structure.
 *
 * \retval      ALT_E_SUCCESS   The operation was successful.
 * \retval      ALT_E_ERROR     Details about error status code
 */
s32 alt_dma_program_uninit(ALT_DMA_PROGRAM_t * pgm);

/*!
 * This function clears the existing DMA microcode program in the given
 * program buffer.
 *
 * \param       pgm
 *              A pointer to a DMA program buffer structure.
 *
 * \retval      ALT_E_SUCCESS   The operation was successful.
 * \retval      ALT_E_ERROR     Details about error status code.
 */
s32 alt_dma_program_clear(ALT_DMA_PROGRAM_t * pgm);

/*!
 * This function validate that the given DMA microcode program buffer contains
 * a well formed program.
 *
 * \param       pgm
 *              A pointer to a DMA program buffer structure.
 *
 * \retval      ALT_E_SUCCESS   The given program is well formed.
 * \retval      ALT_E_ERROR     The given program is not well formed.
 */
s32 alt_dma_program_validate(const ALT_DMA_PROGRAM_t * pgm);

/*!
 * This function reports the number bytes incremented for the register
 * specified. The purpose is to determine the progress of an ongoing DMA
 * transfer.
 *
 * It is implemented by calculating the difference of the programmed SAR or DAR
 * with the current channel SAR or DAR register value.
 *
 * \param       pgm
 *              A pointer to a DMA program buffer structure.
 *
 * \param       channel
 *              The channel that the program is running on.
 *
 * \param       reg
 *              Register to change the value for. Valid for only
 *              ALT_DMA_PROGRAM_REG_SAR and ALT_DMA_PROGRAM_REG_DAR.
 *
 * \param       current
 *              The current snapshot value of the register read from the DMA
 *              channel.
 *
 * \param       progress
 *              [out] A pointer to a memory location that will be used to store
 *              the number of bytes transfered.
 *
 * \retval      ALT_E_SUCCESS   The operation was successful.
 * \retval      ALT_E_ERROR     Details about error status code.
 * \retval      ALT_E_BAD_ARG   The specified channel is invalid, the specified
 *                              register is invalid, or the DMAMOV for the
 *                              specified register has not yet been assembled
 *                              in the current program buffer.
 */
s32 alt_dma_program_progress_reg(ALT_DMA_PROGRAM_t * pgm,
                                             ALT_DMA_PROGRAM_REG_t reg,
                                             u32 current, u32 * progress);

/*!
 * This function updates a pre-existing DMAMOV value affecting the SAR or DAR
 * registers. This allows for pre-assembled programs that can be used on
 * different source and destination addresses.
 *
 * \param       pgm
 *              A pointer to a DMA program buffer structure.
 *
 * \param       reg
 *              Register to change the value for. Valid for only
 *              ALT_DMA_PROGRAM_REG_SAR and ALT_DMA_PROGRAM_REG_DAR.
 *
 * \param       val
 *              The value to update to.
 *
 * \retval      ALT_E_SUCCESS   The operation was successful.
 * \retval      ALT_E_ERROR     Details about error status code.
 * \retval      ALT_E_BAD_ARG   The specified register is invalid or the DMAMOV
 *                              for the specified register has not yet been
 *                              assembled in the current program buffer.
 */
s32 alt_dma_program_update_reg(ALT_DMA_PROGRAM_t * pgm,
                                           ALT_DMA_PROGRAM_REG_t reg, u32 val);

/*!
 */

/*!
 * Assembles a DMAADDH (Add Halfword) instruction into the microcode program
 * buffer. This instruction uses 3 bytes of buffer space.
 *
 * \param       pgm
 *              The DMA program buffer to contain the assembled instruction.
 *
 * \param       addr_reg
 *              The channel address register (ALT_DMA_PROGRAM_REG_DAR or
 *              ALT_DMA_PROGRAM_REG_SAR) to add the value to.
 *
 * \param       val
 *              The 16-bit unsigned value to add to the channel address
 *              register.
 *
 * \retval      ALT_E_SUCCESS       Successful instruction assembly status.
 * \retval      ALT_E_DMA_BUF_OVF   DMA program buffer overflow.
 * \retval      ALT_E_BAD_ARG       Invalid channel register specified.
 */
// Assembler Syntax: DMAADDH <address_register>, <16-bit immediate>
s32 alt_dma_program_DMAADDH(ALT_DMA_PROGRAM_t * pgm,
                                        ALT_DMA_PROGRAM_REG_t addr_reg, u16 val);

/*!
 * Assembles a DMAADNH (Add Negative Halfword) instruction into the microcode
 * program buffer. This instruction uses 3 bytes of buffer space.
 *
 * \param       pgm
 *              The DMA programm buffer to contain the assembled instruction.
 *
 * \param       addr_reg
 *              The channel address register (ALT_DMA_PROGRAM_REG_DAR or
 *              ALT_DMA_PROGRAM_REG_SAR) to add the value to.
 *
 * \param       val
 *              The 16-bit unsigned value to add to the channel address
 *              register.
 *
 * \retval      ALT_E_SUCCESS       Successful instruction assembly status.
 * \retval      ALT_E_DMA_BUF_OVF   DMA program buffer overflow.
 * \retval      ALT_E_BAD_ARG       Invalid channel register specified.
 */
// Assembler Syntax: DMAADNH <address_register>, <16-bit immediate>
s32 alt_dma_program_DMAADNH(ALT_DMA_PROGRAM_t * pgm,
                                        ALT_DMA_PROGRAM_REG_t addr_reg, u16 val);

/*!
 * Assembles a DMAEND (End) instruction into the microcode program buffer.
 * This instruction uses 1 byte of buffer space.
 *
 * \param       pgm
 *              The DMA programm buffer to contain the assembled instruction.
 *
 * \retval      ALT_E_SUCCESS       Successful instruction assembly status.
 * \retval      ALT_E_DMA_BUF_OVF   DMA program buffer overflow.
 */
// Assembler Syntax: DMAEND
s32 alt_dma_program_DMAEND(ALT_DMA_PROGRAM_t * pgm);

/*!
 * Assembles a DMAFLUSHP (Flush Peripheral) instruction into the microcode
 * program buffer. This instruction uses 2 bytes of buffer space.
 *
 * \param       pgm
 *              The DMA programm buffer to contain the assembled instruction.
 *
 * \param       periph
 *              The peripheral to flush.
 *
 * \retval      ALT_E_SUCCESS       Successful instruction assembly status.
 * \retval      ALT_E_DMA_BUF_OVF   DMA program buffer overflow.
 * \retval      ALT_E_BAD_ARG       Invalid peripheral specified.
 */
// Assembler Syntax: DMAFLUSHP <peripheral>
s32 alt_dma_program_DMAFLUSHP(ALT_DMA_PROGRAM_t * pgm,
                                          ALT_DMA_PERIPH_t periph);

/*!
 * Assembles a DMAGO (Go) instruction into the microcode program buffer. This
 * instruction uses 6 bytes of buffer space.
 *
 * \param       pgm
 *              The DMA programm buffer to contain the assembled instruction.
 *
 * \param       channel
 *              The stopped channel to act upon.
 *
 * \param       val
 *              The value to write to the channel program counter register.
 *
 * \param       sec
 *              The security state for the operation.
 *
 * \retval      ALT_E_SUCCESS       Successful instruction assembly status.
 * \retval      ALT_E_DMA_BUF_OVF   DMA program buffer overflow.
 * \retval      ALT_E_BAD_ARG       Invalid channel or security specified.
 */
// Assembler Syntax: DMAGO <channel_number>, <32-bit_immediate> [, ns]
s32 alt_dma_program_DMAGO(ALT_DMA_PROGRAM_t * pgm,
                                      ALT_DMA_CHANNEL_t channel, u32 val,
                                      ALT_DMA_SECURITY_t sec);

/*!
 * Assembles a DMAKILL (Kill) instruction into the microcode program buffer.
 * This instruction uses 1 byte of buffer space.
 *
 * \param       pgm
 *              The DMA programm buffer to contain the assembled instruction.
 *
 * \retval      ALT_E_SUCCESS       Successful instruction assembly status.
 * \retval      ALT_E_DMA_BUF_OVF   DMA program buffer overflow.
 */
// Assembler Syntax: DMAKILL
s32 alt_dma_program_DMAKILL(ALT_DMA_PROGRAM_t * pgm);

/*!
 * Assembles a DMALD (Load) instruction into the microcode program buffer.
 * This instruction uses 1 byte of buffer space.
 *
 * \param       pgm
 *              The DMA programm buffer to contain the assembled instruction.
 *
 * \param       mod
 *              The program instruction modifier for the type of transfer.
 *              Only ALT_DMA_PROGRAM_INST_MOD_SINGLE and 
 *              ALT_DMA_PROGRAM_INST_MOD_BURST are valid options.
 *
 * \retval      ALT_E_SUCCESS       Successful instruction assembly status.
 * \retval      ALT_E_DMA_BUF_OVF   DMA program buffer overflow.
 * \retval      ALT_E_BAD_ARG       Invalid instruction modifier specified.
 */
// Assembler Syntax: DMALD[S|B]
s32 alt_dma_program_DMALD(ALT_DMA_PROGRAM_t * pgm,
                                      ALT_DMA_PROGRAM_INST_MOD_t mod);

/*!
 * Assembles a DMALDP (Load and notify Peripheral) instruction into the
 * microcode program buffer. This instruction uses 2 bytes of buffer space.
 *
 * \param       pgm
 *              The DMA programm buffer to contain the assembled instruction.
 *
 * \param       mod
 *              The program instruction modifier for the type of transfer.
 *              Only ALT_DMA_PROGRAM_INST_MOD_SINGLE and 
 *              ALT_DMA_PROGRAM_INST_MOD_BURST are valid options.
 *
 * \param       periph
 *              The peripheral to notify.
 *
 * \retval      ALT_E_SUCCESS       Successful instruction assembly status.
 * \retval      ALT_E_DMA_BUF_OVF   DMA program buffer overflow.
 * \retval      ALT_E_BAD_ARG       Invalid instruction modifier or peripheral
 *                                  specified.
 */
// Assembler Syntax: DMALDP<S|B> <peripheral>
s32 alt_dma_program_DMALDP(ALT_DMA_PROGRAM_t * pgm,
                                       ALT_DMA_PROGRAM_INST_MOD_t mod, ALT_DMA_PERIPH_t periph);

/*!
 * Assembles a DMALP (Loop) instruction into the microcode program buffer.
 * This instruction uses 2 bytes of buffer space.
 *
 * \param       pgm
 *              The DMA programm buffer to contain the assembled instruction.
 *
 * \param       iterations
 *              The number of iterations to run for. Valid values are 1 - 256.
 *
 * \retval      ALT_E_SUCCESS       Successful instruction assembly status.
 * \retval      ALT_E_DMA_BUF_OVF   DMA program buffer overflow.
 * \retval      ALT_E_BAD_ARG       Invalid iterations specified.
 * \retval      ALT_E_BAD_OPERATION All loop registers are in use.
 */
// Assembler Syntax: DMALP [<LC0>|<LC1>] <loop_iterations>
s32 alt_dma_program_DMALP(ALT_DMA_PROGRAM_t * pgm,
                                      u32 iterations);

/*!
 * Assembles a DMALPEND (Loop End) instruction into the microcode program
 * buffer. This instruction uses 2 bytes of buffer space.
 *
 * \param       pgm
 *              The DMA programm buffer to contain the assembled instruction.
 *
 * \param       mod
 *              The program instruction modifier for the loop terminator. Only
 *              ALT_DMA_PROGRAM_INST_MOD_NONE, ALT_DMA_PROGRAM_INST_MOD_SINGLE
 *              and ALT_DMA_PROGRAM_INST_MOD_BURST are valid options.
 *
 * \retval      ALT_E_SUCCESS       Successful instruction assembly status.
 * \retval      ALT_E_DMA_BUF_OVF   DMA program buffer overflow.
 * \retval      ALT_E_BAD_ARG       Invalid instruction modifier specified.
 * \retval      ALT_E_ARG_RANGE     Loop size is too large to be supported.
 * \retval      ALT_E_BAD_OPERATION A valid DMALP or DMALPFE was not added to
 *                                  the program buffer before adding this
 *                                  DMALPEND instruction.
 */
// Assembler Syntax: DMALPEND[S|B]
s32 alt_dma_program_DMALPEND(ALT_DMA_PROGRAM_t * pgm,
                                         ALT_DMA_PROGRAM_INST_MOD_t mod);

/*!
 * Assembles a DMALPFE (Loop Forever) instruction into the microcode program
 * buffer. No instruction is added to the buffer but a previous DMALPEND to
 * create an infinite loop.
 *
 * \param       pgm
 *              The DMA programm buffer to contain the assembled instruction.
 *
 * \retval      ALT_E_SUCCESS       Successful instruction assembly status.
 * \retval      ALT_E_DMA_BUF_OVF   DMA program buffer overflow.
 */
// Assembler Syntax: DMALPFE
s32 alt_dma_program_DMALPFE(ALT_DMA_PROGRAM_t * pgm);

/*!
 * Assembles a DMAMOV (Move) instruction into the microcode program buffer.
 * This instruction uses 6 bytes of buffer space.
 *
 * \param       pgm
 *              The DMA programm buffer to contain the assembled instruction.
 *
 * \param       chan_reg
 *              The channel non-looping register (ALT_DMA_PROGRAM_REG_SAR,
 *              ALT_DMA_PROGRAM_REG_DAR or ALT_DMA_PROGRAM_REG_CCR) to copy
 *              the value to.
 *
 * \param       val
 *              The value to write to the specified register.
 *
 * \retval      ALT_E_SUCCESS       Successful instruction assembly status.
 * \retval      ALT_E_DMA_BUF_OVF   DMA program buffer overflow.
 * \retval      ALT_E_BAD_ARG       Invalid channel register specified.
 */
// Assembler Syntax: DMAMOV <destination_register>, <32-bit_immediate>
s32 alt_dma_program_DMAMOV(ALT_DMA_PROGRAM_t * pgm,
                                       ALT_DMA_PROGRAM_REG_t chan_reg, u32 val);

/*!
 * Assembles a DMANOP (No Operation) instruction into the microcode program
 * buffer. This instruction uses 1 byte of buffer space.
 *
 * \param       pgm
 *              The DMA programm buffer to contain the assembled instruction.
 *
 * \retval      ALT_E_SUCCESS       Successful instruction assembly status.
 * \retval      ALT_E_DMA_BUF_OVF   DMA program buffer overflow.
 */
// Assembler Syntax: DMANOP
s32 alt_dma_program_DMANOP(ALT_DMA_PROGRAM_t * pgm);

/*!
 * Assembles a DMARMB (Read Memory Barrier) instruction into the microcode
 * program buffer. This instruction uses 1 byte of buffer space.
 *
 * \param       pgm
 *              The DMA programm buffer to contain the assembled instruction.
 *
 * \retval      ALT_E_SUCCESS       Successful instruction assembly status.
 * \retval      ALT_E_DMA_BUF_OVF   DMA program buffer overflow.
 */
// Assembler Syntax: DMARMB
s32 alt_dma_program_DMARMB(ALT_DMA_PROGRAM_t * pgm);

/*!
 * Assembles a DMASEV (Send Event) instruction into the microcode program
 * buffer. This instruction uses 2 byte of buffer space.
 *
 * \param       pgm
 *              The DMA programm buffer to contain the assembled instruction.
 *
 * \param       evt
 *              The event to send.
 *
 * \retval      ALT_E_SUCCESS       Successful instruction assembly status.
 * \retval      ALT_E_DMA_BUF_OVF   DMA program buffer overflow.
 * \retval      ALT_E_BAD_ARG       Invalid event specified.
 */
// Assembler Syntax: DMASEV <event_num>
s32 alt_dma_program_DMASEV(ALT_DMA_PROGRAM_t * pgm,
                                       ALT_DMA_EVENT_t evt);

/*!
 * Assembles a DMAST (Store) instruction into the microcode program buffer.
 * This instruction uses 1 byte of buffer space.
 *
 * \param       pgm
 *              The DMA programm buffer to contain the assembled instruction.
 *
 * \param       mod
 *              The program instruction modifier for the type of transfer.
 *              Only ALT_DMA_PROGRAM_INST_MOD_SINGLE and 
 *              ALT_DMA_PROGRAM_INST_MOD_BURST are valid options.
 *
 * \retval      ALT_E_SUCCESS       Successful instruction assembly status.
 * \retval      ALT_E_DMA_BUF_OVF   DMA program buffer overflow.
 */
// Assembler Syntax: DMAST[S|B]
s32 alt_dma_program_DMAST(ALT_DMA_PROGRAM_t * pgm,
                                      ALT_DMA_PROGRAM_INST_MOD_t mod);

/*!
 * Assembles a DMASTP (Store and notify Peripheral) instruction into the
 * microcode program buffer. This instruction uses 2 bytes of buffer space.
 *
 * \param       pgm
 *              The DMA programm buffer to contain the assembled instruction.
 *
 * \param       mod
 *              The program instruction modifier for the type of transfer.
 *              Only ALT_DMA_PROGRAM_INST_MOD_SINGLE and 
 *              ALT_DMA_PROGRAM_INST_MOD_BURST are valid options.
 *
 * \param       periph
 *              The peripheral to notify.
 *
 * \retval      ALT_E_SUCCESS       Successful instruction assembly status.
 * \retval      ALT_E_DMA_BUF_OVF   DMA program buffer overflow.
 * \retval      ALT_E_BAD_ARG       Invalid instruction modifier or peripheral
 *                                  specified.
 */
// Assembler Syntax: DMASTP<S|B> <peripheral>
s32 alt_dma_program_DMASTP(ALT_DMA_PROGRAM_t * pgm,
                                       ALT_DMA_PROGRAM_INST_MOD_t mod, ALT_DMA_PERIPH_t periph);

/*!
 * Assembles a DMASTZ (Store Zero) instruction into the microcode program
 * buffer. This instruction uses 1 byte of buffer space.
 *
 * \param       pgm
 *              The DMA programm buffer to contain the assembled instruction.
 *
 * \retval      ALT_E_SUCCESS       Successful instruction assembly status.
 * \retval      ALT_E_DMA_BUF_OVF   DMA program buffer overflow.
 */
// Assembler Syntax: DMASTZ
s32 alt_dma_program_DMASTZ(ALT_DMA_PROGRAM_t * pgm);

/*!
 * Assembles a DMAWFE (Wait For Event) instruction into the microcode program
 * buffer. This instruction uses 2 byte of buffer space.
 *
 * \param       pgm
 *              The DMA programm buffer to contain the assembled instruction.
 *
 * \param       evt
 *              The event to wait for.
 *
 * \param       invalid
 *              If invalid is set to true, the instruction will be configured
 *              to invalidate the instruction cache for the current DMA
 *              thread.
 *
 * \retval      ALT_E_SUCCESS       Successful instruction assembly status.
 * \retval      ALT_E_DMA_BUF_OVF   DMA program buffer overflow.
 * \retval      ALT_E_BAD_ARG       Invalid event specified.
 */
// Assembler Syntax: DMAWFE <event_num>[, invalid]
s32 alt_dma_program_DMAWFE(ALT_DMA_PROGRAM_t * pgm,
                                       ALT_DMA_EVENT_t evt, bool invalid);

/*!
 * Assembles a DMAWFP (Wait for Peripheral) instruction into the microcode
 * program buffer. This instruction uses 2 bytes of buffer space.
 *
 * \param       pgm
 *              The DMA programm buffer to contain the assembled instruction.
 *
 * \param       periph
 *              The peripheral to wait on.
 *
 * \param       mod
 *              The program instruction modifier for the type of transfer.
 *              Only ALT_DMA_PROGRAM_INST_MOD_SINGLE,
 *              ALT_DMA_PROGRAM_INST_MOD_BURST, or
 *              ALT_DMA_PROGRAM_INST_MOD_PERIPH are valid options.
 *
 * \retval      ALT_E_SUCCESS       Successful instruction assembly status.
 * \retval      ALT_E_DMA_BUF_OVF   DMA program buffer overflow.
 * \retval      ALT_E_BAD_ARG       Invalid peripheral or instruction modifier
 *                                  specified.
 */
// Assembler Syntax: DMAWFP <peripheral>, <single|burst|periph>
s32 alt_dma_program_DMAWFP(ALT_DMA_PROGRAM_t * pgm,
                                       ALT_DMA_PERIPH_t periph, ALT_DMA_PROGRAM_INST_MOD_t mod);

/*!
 * Assembles a DMAWMB (Write Memory Barrier) instruction into the microcode
 * program buffer. This instruction uses 1 byte of buffer space.
 *
 * \param       pgm
 *              The DMA programm buffer to contain the assembled instruction.
 *
 * \retval      ALT_E_SUCCESS       Successful instruction assembly status.
 * \retval      ALT_E_DMA_BUF_OVF   DMA program buffer overflow.
 */
// Assembler Syntax: DMAWMB
s32 alt_dma_program_DMAWMB(ALT_DMA_PROGRAM_t * pgm);

/*!
 * \addtogroup DMA_CCR Support for DMAMOV CCR
 *
 * The ALT_DMA_CCR_OPT_* macro definitions are defined here to facilitate the
 * dynamic microcode programming of the assembler directive:
\verbatim

DMAMOV CCR, [SB<1-16>] [SS<8|16|32|64|128>] [SA<I|F>]
            [SP<imm3>] [SC<imm4>]
            [DB<1-16>] [DS<8|16|32|64|128>] [DA<I|F>]
            [DP<imm3>] [DC<imm4>]
            [ES<8|16|32|64|128>]

\endverbatim
* with a DMAMOV instruction (see: alt_dma_program_DMAMOV()).
*
* For example the assembler directive:
\verbatim
DMAMOV CCR SB1 SS32 DB1 DS32
\endverbatim
* would be dynamically programmed with the following API call:
\verbatim
alt_dma_program_DMAMOV( pgm,
                        ALT_DMA_PROGRAM_REG_CCR,
                        (   ALT_DMA_CCR_OPT_SB1
                          | ALT_DMA_CCR_OPT_SS32
                          | ALT_DMA_CCR_OPT_SA_DEFAULT
                          | ALT_DMA_CCR_OPT_SP_DEFAULT
                          | ALT_DMA_CCR_OPT_SC_DEFAULT
                          | ALT_DMA_CCR_OPT_DB1
                          | ALT_DMA_CCR_OPT_DS32
                          | ALT_DMA_CCR_OPT_DA_DEFAULT
                          | ALT_DMA_CCR_OPT_DP_DEFAULT
                          | ALT_DMA_CCR_OPT_DC_DEFAULT
                          | ALT_DMA_CCR_OPT_ES8
                        )
                      );
\endverbatim
*
* Each CCR option category should be specified regardless of whether it
* specifies a custom value or the normal default value (i.e. an
* ALT_DMA_CCR_OPT_*_DEFAULT.
*
* @{
*/

/*
 * Source Address {Fixed,Incrementing}
 */
/*! Source Address Fixed address burst. */
#define ALT_DMA_CCR_OPT_SAF         (0 << 0)
/*! Source Address Incrementing address burst. */
#define ALT_DMA_CCR_OPT_SAI         (1 << 0)
/*! Source Address Default value. */
#define ALT_DMA_CCR_OPT_SA_DEFAULT  ALT_DMA_CCR_OPT_SAI

/*
 * Source burst Size (in bits)
 */
/*! Source burst Size of 8 bits. */
#define ALT_DMA_CCR_OPT_SS8         (0 << 1)
/*! Source burst Size of 16 bits. */
#define ALT_DMA_CCR_OPT_SS16        (1 << 1)
/*! Source burst Size of 32 bits. */
#define ALT_DMA_CCR_OPT_SS32        (2 << 1)
/*! Source burst Size of 64 bits. */
#define ALT_DMA_CCR_OPT_SS64        (3 << 1)
/*! Source burst Size of 128 bits. */
#define ALT_DMA_CCR_OPT_SS128       (4 << 1)
/*! Source burst Size default bits. */
#define ALT_DMA_CCR_OPT_SS_DEFAULT  ALT_DMA_CCR_OPT_SS8

/*
 * Source burst Length (in transfer(s))
 */
/*! Source Burst length of 1 transfer. */
#define ALT_DMA_CCR_OPT_SB1         (0x0 << 4)
/*! Source Burst length of 2 transfers. */
#define ALT_DMA_CCR_OPT_SB2         (0x1 << 4)
/*! Source Burst length of 3 transfers. */
#define ALT_DMA_CCR_OPT_SB3         (0x2 << 4)
/*! Source Burst length of 4 transfers. */
#define ALT_DMA_CCR_OPT_SB4         (0x3 << 4)
/*! Source Burst length of 5 transfers. */
#define ALT_DMA_CCR_OPT_SB5         (0x4 << 4)
/*! Source Burst length of 6 transfers. */
#define ALT_DMA_CCR_OPT_SB6         (0x5 << 4)
/*! Source Burst length of 7 transfers. */
#define ALT_DMA_CCR_OPT_SB7         (0x6 << 4)
/*! Source Burst length of 8 transfers. */
#define ALT_DMA_CCR_OPT_SB8         (0x7 << 4)
/*! Source Burst length of 9 transfers. */
#define ALT_DMA_CCR_OPT_SB9         (0x8 << 4)
/*! Source Burst length of 10 transfers. */
#define ALT_DMA_CCR_OPT_SB10        (0x9 << 4)
/*! Source Burst length of 11 transfers. */
#define ALT_DMA_CCR_OPT_SB11        (0xa << 4)
/*! Source Burst length of 12 transfers. */
#define ALT_DMA_CCR_OPT_SB12        (0xb << 4)
/*! Source Burst length of 13 transfers. */
#define ALT_DMA_CCR_OPT_SB13        (0xc << 4)
/*! Source Burst length of 14 transfers. */
#define ALT_DMA_CCR_OPT_SB14        (0xd << 4)
/*! Source Burst length of 15 transfers. */
#define ALT_DMA_CCR_OPT_SB15        (0xe << 4)
/*! Source Burst length of 16 transfers. */
#define ALT_DMA_CCR_OPT_SB16        (0xf << 4)
/*! Source Burst length default transfers. */
#define ALT_DMA_CCR_OPT_SB_DEFAULT  ALT_DMA_CCR_OPT_SB1

/*
 * Source Protection
 */
/*! Source Protection bits for AXI bus ARPROT[2:0]. */
#define ALT_DMA_CCR_OPT_SP(imm3)    ((imm3) << 8)
/*! Source Protection bits default value. */
#define ALT_DMA_CCR_OPT_SP_DEFAULT  ALT_DMA_CCR_OPT_SP(0)

/*
 * Source cache
 */
/*! Source Cache bits for AXI bus ARCACHE[2:0]. */
#define ALT_DMA_CCR_OPT_SC(imm4)    ((imm4) << 11)
/*! Source Cache bits default value. */
#define ALT_DMA_CCR_OPT_SC_DEFAULT  ALT_DMA_CCR_OPT_SC(0)
/*! Source Cache is write-back, read allocate, ARCACHE[3] is fixed low
      and here ARCACHE[2..0]= b111
  */
#define ALT_DMA_CCR_OPT_SC_WBRA	(0b111)

/*
 * Destination Address {Fixed,Incrementing}
 */
/*! Destination Address Fixed address burst. */
#define ALT_DMA_CCR_OPT_DAF         (0 << 14)
/*! Destination Address Incrementing address burst. */
#define ALT_DMA_CCR_OPT_DAI         (1 << 14)
/*! Destination Address Default value. */
#define ALT_DMA_CCR_OPT_DA_DEFAULT  ALT_DMA_CCR_OPT_DAI

/*
 * Destination burst Size (in bits)
 */
/*! Destination burst Size of 8 bits. */
#define ALT_DMA_CCR_OPT_DS8         (0 << 15)
/*! Destination burst Size of 16 bits. */
#define ALT_DMA_CCR_OPT_DS16        (1 << 15)
/*! Destination burst Size of 32 bits. */
#define ALT_DMA_CCR_OPT_DS32        (2 << 15)
/*! Destination burst Size of 64 bits. */
#define ALT_DMA_CCR_OPT_DS64        (3 << 15)
/*! Destination burst Size of 128 bits. */
#define ALT_DMA_CCR_OPT_DS128       (4 << 15)
/*! Destination burst Size default bits. */
#define ALT_DMA_CCR_OPT_DS_DEFAULT  ALT_DMA_CCR_OPT_DS8

/*
 * Destination Burst length (in transfer(s))
 */
/*! Destination Burst length of 1 transfer. */
#define ALT_DMA_CCR_OPT_DB1         (0x0 << 18)
/*! Destination Burst length of 2 transfers. */
#define ALT_DMA_CCR_OPT_DB2         (0x1 << 18)
/*! Destination Burst length of 3 transfers. */
#define ALT_DMA_CCR_OPT_DB3         (0x2 << 18)
/*! Destination Burst length of 4 transfers. */
#define ALT_DMA_CCR_OPT_DB4         (0x3 << 18)
/*! Destination Burst length of 5 transfers. */
#define ALT_DMA_CCR_OPT_DB5         (0x4 << 18)
/*! Destination Burst length of 6 transfers. */
#define ALT_DMA_CCR_OPT_DB6         (0x5 << 18)
/*! Destination Burst length of 7 transfers. */
#define ALT_DMA_CCR_OPT_DB7         (0x6 << 18)
/*! Destination Burst length of 8 transfers. */
#define ALT_DMA_CCR_OPT_DB8         (0x7 << 18)
/*! Destination Burst length of 9 transfers. */
#define ALT_DMA_CCR_OPT_DB9         (0x8 << 18)
/*! Destination Burst length of 10 transfers. */
#define ALT_DMA_CCR_OPT_DB10        (0x9 << 18)
/*! Destination Burst length of 11 transfers. */
#define ALT_DMA_CCR_OPT_DB11        (0xa << 18)
/*! Destination Burst length of 12 transfers. */
#define ALT_DMA_CCR_OPT_DB12        (0xb << 18)
/*! Destination Burst length of 13 transfers. */
#define ALT_DMA_CCR_OPT_DB13        (0xc << 18)
/*! Destination Burst length of 14 transfers. */
#define ALT_DMA_CCR_OPT_DB14        (0xd << 18)
/*! Destination Burst length of 15 transfers. */
#define ALT_DMA_CCR_OPT_DB15        (0xe << 18)
/*! Destination Burst length of 16 transfers. */
#define ALT_DMA_CCR_OPT_DB16        (0xf << 18)
/*! Destination Burst length default transfers. */
#define ALT_DMA_CCR_OPT_DB_DEFAULT  ALT_DMA_CCR_OPT_DB1

/*
 * Destination Protection
 */
/*! Destination Protection bits for AXI bus AWPROT[2:0]. */
#define ALT_DMA_CCR_OPT_DP(imm3)    ((imm3) << 22)
/*! Destination Protection bits default value. */
#define ALT_DMA_CCR_OPT_DP_DEFAULT  ALT_DMA_CCR_OPT_DP(0)

/*
 * Destination Cache
 */
/*! Destination Cache bits for AXI bus AWCACHE[3,1:0]. */
#define ALT_DMA_CCR_OPT_DC(imm4)    ((imm4) << 25)
/*! Destination Cache bits default value. */
#define ALT_DMA_CCR_OPT_DC_DEFAULT  ALT_DMA_CCR_OPT_DC(0)
/*! Destination Cache is write-back, read allocate, AWCACHE[2] is fixed low
      and here is AWCACHE[3,1,0] = b111
 */
#define ALT_DMA_CCR_OPT_DC_WBWA		(0b111)

/*
 * Endian Swap size (in bits)
 */
/*! Endian Swap: No swap, 8-bit data. */
#define ALT_DMA_CCR_OPT_ES8         (0 << 28)
/*! Endian Swap: Swap bytes within 16-bit data. */
#define ALT_DMA_CCR_OPT_ES16        (1 << 28)
/*! Endian Swap: Swap bytes within 32-bit data. */
#define ALT_DMA_CCR_OPT_ES32        (2 << 28)
/*! Endian Swap: Swap bytes within 64-bit data. */
#define ALT_DMA_CCR_OPT_ES64        (3 << 28)
/*! Endian Swap: Swap bytes within 128-bit data. */
#define ALT_DMA_CCR_OPT_ES128       (4 << 28)
/*! Endian Swap: Default byte swap. */
#define ALT_DMA_CCR_OPT_ES_DEFAULT  ALT_DMA_CCR_OPT_ES8

/*! Default CCR register options for a DMAMOV CCR assembler directive. */
#define ALT_DMA_CCR_OPT_DEFAULT \
    (ALT_DMA_CCR_OPT_SB1 | ALT_DMA_CCR_OPT_SS8 | ALT_DMA_CCR_OPT_SAI | \
     ALT_DMA_CCR_OPT_SP(0) | ALT_DMA_CCR_OPT_SC(0) | \
     ALT_DMA_CCR_OPT_DB1 | ALT_DMA_CCR_OPT_DS8 | ALT_DMA_CCR_OPT_DAI | \
     ALT_DMA_CCR_OPT_DP(0) | ALT_DMA_CCR_OPT_DC(0) | \
     ALT_DMA_CCR_OPT_ES8)





















#endif