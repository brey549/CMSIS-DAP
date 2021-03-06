/*
 * Copyright (c) 2014, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * THIS SOFTWARE IS PROVIDED BY FREESCALE "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL FREESCALE BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 */
/*
 * WARNING! DO NOT EDIT THIS FILE DIRECTLY!
 *
 * This file was generated automatically and any changes may be lost.
 */
#ifndef __HW_FB_REGISTERS_H__
#define __HW_FB_REGISTERS_H__

#include "regs.h"

/*
 * MK70F12 FB
 *
 * FlexBus external bus interface
 *
 * Registers defined in this header file:
 * - HW_FB_CSARn - Chip select address register
 * - HW_FB_CSMRn - Chip select mask register
 * - HW_FB_CSCRn - Chip select control register
 * - HW_FB_CSPMCR - Chip select port multiplexing control register
 *
 * - hw_fb_t - Struct containing all module registers.
 */

//! @name Module base addresses
//@{
#ifndef REGS_FB_BASE
#define HW_FB_INSTANCE_COUNT (1U) //!< Number of instances of the FB module.
#define REGS_FB_BASE (0x4000C000U) //!< Base address for FB.
#endif
//@}

//-------------------------------------------------------------------------------------------
// HW_FB_CSARn - Chip select address register
//-------------------------------------------------------------------------------------------

#ifndef __LANGUAGE_ASM__
/*!
 * @brief HW_FB_CSARn - Chip select address register (RW)
 *
 * Reset value: 0x00000000U
 *
 * The CSARn registers specify the chip-select base addresses. Because the
 * FlexBus module is one of the slaves connected to the crossbar switch, it is only
 * accessible within a certain memory range. Refer to the device memory map for the
 * applicable FlexBus "expansion" address range for which the chip-selects can
 * be active. Set the CSARn registers appropriately.
 */
typedef union _hw_fb_csarn
{
    uint32_t U;
    struct _hw_fb_csarn_bitfields
    {
        uint32_t RESERVED0 : 16;       //!< [15:0]
        uint32_t BA : 16;              //!< [31:16] Base address
    } B;
} hw_fb_csarn_t;
#endif

/*!
 * @name Constants and macros for entire FB_CSARn register
 */
//@{
#define HW_FB_CSARn_COUNT (6U)

#define HW_FB_CSARn_ADDR(n)      (REGS_FB_BASE + 0x0U + (0xCU * n))

#ifndef __LANGUAGE_ASM__
#define HW_FB_CSARn(n)           (*(__IO hw_fb_csarn_t *) HW_FB_CSARn_ADDR(n))
#define HW_FB_CSARn_RD(n)        (HW_FB_CSARn(n).U)
#define HW_FB_CSARn_WR(n, v)     (HW_FB_CSARn(n).U = (v))
#define HW_FB_CSARn_SET(n, v)    (HW_FB_CSARn_WR(n, HW_FB_CSARn_RD(n) |  (v)))
#define HW_FB_CSARn_CLR(n, v)    (HW_FB_CSARn_WR(n, HW_FB_CSARn_RD(n) & ~(v)))
#define HW_FB_CSARn_TOG(n, v)    (HW_FB_CSARn_WR(n, HW_FB_CSARn_RD(n) ^  (v)))
#endif
//@}

/*
 * Constants & macros for individual FB_CSARn bitfields
 */

/*!
 * @name Register FB_CSARn, field BA[31:16] (RW)
 */
//@{
#define BP_FB_CSARn_BA       (16U)         //!< Bit position for FB_CSARn_BA.
#define BM_FB_CSARn_BA       (0xFFFF0000U) //!< Bit mask for FB_CSARn_BA.
#define BS_FB_CSARn_BA       (16U)         //!< Bit field size in bits for FB_CSARn_BA.

#ifndef __LANGUAGE_ASM__
//! @brief Read current value of the FB_CSARn_BA field.
#define BR_FB_CSARn_BA(n)    (HW_FB_CSARn(n).B.BA)
#endif

//! @brief Format value for bitfield FB_CSARn_BA.
#define BF_FB_CSARn_BA(v)    (__REG_VALUE_TYPE((__REG_VALUE_TYPE((v), uint32_t) << BP_FB_CSARn_BA), uint32_t) & BM_FB_CSARn_BA)

#ifndef __LANGUAGE_ASM__
//! @brief Set the BA field to a new value.
#define BW_FB_CSARn_BA(n, v) (HW_FB_CSARn_WR(n, (HW_FB_CSARn_RD(n) & ~BM_FB_CSARn_BA) | BF_FB_CSARn_BA(v)))
#endif
//@}
//-------------------------------------------------------------------------------------------
// HW_FB_CSMRn - Chip select mask register
//-------------------------------------------------------------------------------------------

#ifndef __LANGUAGE_ASM__
/*!
 * @brief HW_FB_CSMRn - Chip select mask register (RW)
 *
 * Reset value: 0x00000000U
 *
 * CSMRn registers specify the address mask and allowable access types for the
 * respective chip-selects.
 */
typedef union _hw_fb_csmrn
{
    uint32_t U;
    struct _hw_fb_csmrn_bitfields
    {
        uint32_t V : 1;                //!< [0] Valid
        uint32_t RESERVED0 : 7;        //!< [7:1]
        uint32_t WP : 1;               //!< [8] Write protect
        uint32_t RESERVED1 : 7;        //!< [15:9]
        uint32_t BAM : 16;             //!< [31:16] Base address mask
    } B;
} hw_fb_csmrn_t;
#endif

/*!
 * @name Constants and macros for entire FB_CSMRn register
 */
//@{
#define HW_FB_CSMRn_COUNT (6U)

#define HW_FB_CSMRn_ADDR(n)      (REGS_FB_BASE + 0x4U + (0xCU * n))

#ifndef __LANGUAGE_ASM__
#define HW_FB_CSMRn(n)           (*(__IO hw_fb_csmrn_t *) HW_FB_CSMRn_ADDR(n))
#define HW_FB_CSMRn_RD(n)        (HW_FB_CSMRn(n).U)
#define HW_FB_CSMRn_WR(n, v)     (HW_FB_CSMRn(n).U = (v))
#define HW_FB_CSMRn_SET(n, v)    (HW_FB_CSMRn_WR(n, HW_FB_CSMRn_RD(n) |  (v)))
#define HW_FB_CSMRn_CLR(n, v)    (HW_FB_CSMRn_WR(n, HW_FB_CSMRn_RD(n) & ~(v)))
#define HW_FB_CSMRn_TOG(n, v)    (HW_FB_CSMRn_WR(n, HW_FB_CSMRn_RD(n) ^  (v)))
#endif
//@}

/*
 * Constants & macros for individual FB_CSMRn bitfields
 */

/*!
 * @name Register FB_CSMRn, field V[0] (RW)
 *
 * Values:
 * - 0 - Chip select invalid
 * - 1 - Chip select valid
 */
//@{
#define BP_FB_CSMRn_V        (0U)          //!< Bit position for FB_CSMRn_V.
#define BM_FB_CSMRn_V        (0x00000001U) //!< Bit mask for FB_CSMRn_V.
#define BS_FB_CSMRn_V        (1U)          //!< Bit field size in bits for FB_CSMRn_V.

#ifndef __LANGUAGE_ASM__
//! @brief Read current value of the FB_CSMRn_V field.
#define BR_FB_CSMRn_V(n)     (BITBAND_ACCESS32(HW_FB_CSMRn_ADDR(n), BP_FB_CSMRn_V))
#endif

//! @brief Format value for bitfield FB_CSMRn_V.
#define BF_FB_CSMRn_V(v)     (__REG_VALUE_TYPE((__REG_VALUE_TYPE((v), uint32_t) << BP_FB_CSMRn_V), uint32_t) & BM_FB_CSMRn_V)

#ifndef __LANGUAGE_ASM__
//! @brief Set the V field to a new value.
#define BW_FB_CSMRn_V(n, v)  (BITBAND_ACCESS32(HW_FB_CSMRn_ADDR(n), BP_FB_CSMRn_V) = (v))
#endif
//@}

/*!
 * @name Register FB_CSMRn, field WP[8] (RW)
 *
 * Values:
 * - 0 - Read and write accesses are allowed
 * - 1 - Only read accesses are allowed
 */
//@{
#define BP_FB_CSMRn_WP       (8U)          //!< Bit position for FB_CSMRn_WP.
#define BM_FB_CSMRn_WP       (0x00000100U) //!< Bit mask for FB_CSMRn_WP.
#define BS_FB_CSMRn_WP       (1U)          //!< Bit field size in bits for FB_CSMRn_WP.

#ifndef __LANGUAGE_ASM__
//! @brief Read current value of the FB_CSMRn_WP field.
#define BR_FB_CSMRn_WP(n)    (BITBAND_ACCESS32(HW_FB_CSMRn_ADDR(n), BP_FB_CSMRn_WP))
#endif

//! @brief Format value for bitfield FB_CSMRn_WP.
#define BF_FB_CSMRn_WP(v)    (__REG_VALUE_TYPE((__REG_VALUE_TYPE((v), uint32_t) << BP_FB_CSMRn_WP), uint32_t) & BM_FB_CSMRn_WP)

#ifndef __LANGUAGE_ASM__
//! @brief Set the WP field to a new value.
#define BW_FB_CSMRn_WP(n, v) (BITBAND_ACCESS32(HW_FB_CSMRn_ADDR(n), BP_FB_CSMRn_WP) = (v))
#endif
//@}

/*!
 * @name Register FB_CSMRn, field BAM[31:16] (RW)
 *
 * Values:
 * - 0 - Corresponding address bit is used in chip-select decode
 * - 1 - Corresponding address bit is a don't care in chip-select decode.
 */
//@{
#define BP_FB_CSMRn_BAM      (16U)         //!< Bit position for FB_CSMRn_BAM.
#define BM_FB_CSMRn_BAM      (0xFFFF0000U) //!< Bit mask for FB_CSMRn_BAM.
#define BS_FB_CSMRn_BAM      (16U)         //!< Bit field size in bits for FB_CSMRn_BAM.

#ifndef __LANGUAGE_ASM__
//! @brief Read current value of the FB_CSMRn_BAM field.
#define BR_FB_CSMRn_BAM(n)   (HW_FB_CSMRn(n).B.BAM)
#endif

//! @brief Format value for bitfield FB_CSMRn_BAM.
#define BF_FB_CSMRn_BAM(v)   (__REG_VALUE_TYPE((__REG_VALUE_TYPE((v), uint32_t) << BP_FB_CSMRn_BAM), uint32_t) & BM_FB_CSMRn_BAM)

#ifndef __LANGUAGE_ASM__
//! @brief Set the BAM field to a new value.
#define BW_FB_CSMRn_BAM(n, v) (HW_FB_CSMRn_WR(n, (HW_FB_CSMRn_RD(n) & ~BM_FB_CSMRn_BAM) | BF_FB_CSMRn_BAM(v)))
#endif
//@}
//-------------------------------------------------------------------------------------------
// HW_FB_CSCRn - Chip select control register
//-------------------------------------------------------------------------------------------

#ifndef __LANGUAGE_ASM__
/*!
 * @brief HW_FB_CSCRn - Chip select control register (RW)
 *
 * Reset value: 0x003FFC00U
 *
 * Each CSCRn controls the auto-acknowledge, address setup and hold times, port
 * size, burst capability, and number of wait states. To support the global
 * chip-select (FB_CS0) the CSCR0 reset values differ from the other CSCRs. The reset
 * value of CSCR0 is as follows: Bits 31-24 are 0 Bit 23-3 are device-dependent
 * Bits 3-0 are 0 See the Chip Configuration details for your particular device
 * for information on the exact CSCR0 reset value.
 */
typedef union _hw_fb_cscrn
{
    uint32_t U;
    struct _hw_fb_cscrn_bitfields
    {
        uint32_t RESERVED0 : 3;        //!< [2:0]
        uint32_t BSTW : 1;             //!< [3] Burst-write enable
        uint32_t BSTR : 1;             //!< [4] Burst-read enable
        uint32_t BEM : 1;              //!< [5] Byte-enable mode
        uint32_t PS : 2;               //!< [7:6] Port size
        uint32_t AA : 1;               //!< [8] Auto-acknowledge enable
        uint32_t BLS : 1;              //!< [9] Byte-lane shift
        uint32_t WS : 6;               //!< [15:10] Wait states
        uint32_t WRAH : 2;             //!< [17:16] Write address hold or deselect
        uint32_t RDAH : 2;             //!< [19:18] Read address hold or deselect
        uint32_t ASET : 2;             //!< [21:20] Address setup
        uint32_t EXTS : 1;             //!< [22]
        uint32_t SWSEN : 1;            //!< [23] Secondary wait state enable
        uint32_t RESERVED1 : 2;        //!< [25:24]
        uint32_t SWS : 6;              //!< [31:26] Secondary wait states
    } B;
} hw_fb_cscrn_t;
#endif

/*!
 * @name Constants and macros for entire FB_CSCRn register
 */
//@{
#define HW_FB_CSCRn_COUNT (6U)

#define HW_FB_CSCRn_ADDR(n)      (REGS_FB_BASE + 0x8U + (0xCU * n))

#ifndef __LANGUAGE_ASM__
#define HW_FB_CSCRn(n)           (*(__IO hw_fb_cscrn_t *) HW_FB_CSCRn_ADDR(n))
#define HW_FB_CSCRn_RD(n)        (HW_FB_CSCRn(n).U)
#define HW_FB_CSCRn_WR(n, v)     (HW_FB_CSCRn(n).U = (v))
#define HW_FB_CSCRn_SET(n, v)    (HW_FB_CSCRn_WR(n, HW_FB_CSCRn_RD(n) |  (v)))
#define HW_FB_CSCRn_CLR(n, v)    (HW_FB_CSCRn_WR(n, HW_FB_CSCRn_RD(n) & ~(v)))
#define HW_FB_CSCRn_TOG(n, v)    (HW_FB_CSCRn_WR(n, HW_FB_CSCRn_RD(n) ^  (v)))
#endif
//@}

/*
 * Constants & macros for individual FB_CSCRn bitfields
 */

/*!
 * @name Register FB_CSCRn, field BSTW[3] (RW)
 *
 * Values:
 * - 0 - Break data larger than the specified port size into individual,
 *     port-sized, non-burst writes. For example, a longword write to an 8-bit port
 *     takes four byte writes.
 * - 1 - Enables burst write of data larger than the specified port size,
 *     including longword writes to 8 and 16-bit ports, word writes to 8-bit ports, and
 *     line writes to 8-, 16-, and 32-bit ports.
 */
//@{
#define BP_FB_CSCRn_BSTW     (3U)          //!< Bit position for FB_CSCRn_BSTW.
#define BM_FB_CSCRn_BSTW     (0x00000008U) //!< Bit mask for FB_CSCRn_BSTW.
#define BS_FB_CSCRn_BSTW     (1U)          //!< Bit field size in bits for FB_CSCRn_BSTW.

#ifndef __LANGUAGE_ASM__
//! @brief Read current value of the FB_CSCRn_BSTW field.
#define BR_FB_CSCRn_BSTW(n)  (BITBAND_ACCESS32(HW_FB_CSCRn_ADDR(n), BP_FB_CSCRn_BSTW))
#endif

//! @brief Format value for bitfield FB_CSCRn_BSTW.
#define BF_FB_CSCRn_BSTW(v)  (__REG_VALUE_TYPE((__REG_VALUE_TYPE((v), uint32_t) << BP_FB_CSCRn_BSTW), uint32_t) & BM_FB_CSCRn_BSTW)

#ifndef __LANGUAGE_ASM__
//! @brief Set the BSTW field to a new value.
#define BW_FB_CSCRn_BSTW(n, v) (BITBAND_ACCESS32(HW_FB_CSCRn_ADDR(n), BP_FB_CSCRn_BSTW) = (v))
#endif
//@}

/*!
 * @name Register FB_CSCRn, field BSTR[4] (RW)
 *
 * Values:
 * - 0 - Data exceeding the specified port size is broken into individual,
 *     port-sized, non-burst reads. For example, a longword read from an 8-bit port is
 *     broken into four 8-bit reads.
 * - 1 - Enables data burst reads larger than the specified port size, including
 *     longword reads from 8- and 16-bit ports, word reads from 8-bit ports, and
 *     line reads from 8, 16-, and 32-bit ports.
 */
//@{
#define BP_FB_CSCRn_BSTR     (4U)          //!< Bit position for FB_CSCRn_BSTR.
#define BM_FB_CSCRn_BSTR     (0x00000010U) //!< Bit mask for FB_CSCRn_BSTR.
#define BS_FB_CSCRn_BSTR     (1U)          //!< Bit field size in bits for FB_CSCRn_BSTR.

#ifndef __LANGUAGE_ASM__
//! @brief Read current value of the FB_CSCRn_BSTR field.
#define BR_FB_CSCRn_BSTR(n)  (BITBAND_ACCESS32(HW_FB_CSCRn_ADDR(n), BP_FB_CSCRn_BSTR))
#endif

//! @brief Format value for bitfield FB_CSCRn_BSTR.
#define BF_FB_CSCRn_BSTR(v)  (__REG_VALUE_TYPE((__REG_VALUE_TYPE((v), uint32_t) << BP_FB_CSCRn_BSTR), uint32_t) & BM_FB_CSCRn_BSTR)

#ifndef __LANGUAGE_ASM__
//! @brief Set the BSTR field to a new value.
#define BW_FB_CSCRn_BSTR(n, v) (BITBAND_ACCESS32(HW_FB_CSCRn_ADDR(n), BP_FB_CSCRn_BSTR) = (v))
#endif
//@}

/*!
 * @name Register FB_CSCRn, field BEM[5] (RW)
 *
 * Values:
 * - 0 - The FB_BE n signals are not asserted for reads. The FB_BE n signals are
 *     asserted for data write only.
 * - 1 - The FB_BE n signals are asserted for read and write accesses
 */
//@{
#define BP_FB_CSCRn_BEM      (5U)          //!< Bit position for FB_CSCRn_BEM.
#define BM_FB_CSCRn_BEM      (0x00000020U) //!< Bit mask for FB_CSCRn_BEM.
#define BS_FB_CSCRn_BEM      (1U)          //!< Bit field size in bits for FB_CSCRn_BEM.

#ifndef __LANGUAGE_ASM__
//! @brief Read current value of the FB_CSCRn_BEM field.
#define BR_FB_CSCRn_BEM(n)   (BITBAND_ACCESS32(HW_FB_CSCRn_ADDR(n), BP_FB_CSCRn_BEM))
#endif

//! @brief Format value for bitfield FB_CSCRn_BEM.
#define BF_FB_CSCRn_BEM(v)   (__REG_VALUE_TYPE((__REG_VALUE_TYPE((v), uint32_t) << BP_FB_CSCRn_BEM), uint32_t) & BM_FB_CSCRn_BEM)

#ifndef __LANGUAGE_ASM__
//! @brief Set the BEM field to a new value.
#define BW_FB_CSCRn_BEM(n, v) (BITBAND_ACCESS32(HW_FB_CSCRn_ADDR(n), BP_FB_CSCRn_BEM) = (v))
#endif
//@}

/*!
 * @name Register FB_CSCRn, field PS[7:6] (RW)
 *
 * Values:
 * - 00 - 32-bit port size. Valid data sampled and driven on FB_D[31:0]
 * - 01 - 8-bit port size. Valid data sampled and driven on FB_D[31:24] if BLS =
 *     0 or FB_D[7:0] if BLS = 1
 * - 10 - 16-bit port size. Valid data sampled and driven on FB_D[31:16] if BLS
 *     = 0 or FB_D[15:0] if BLS = 1
 * - 11 - 16-bit port size. Valid data sampled and driven on FB_D[31:16] if BLS
 *     = 0 or FB_D[15:0] if BLS = 1
 */
//@{
#define BP_FB_CSCRn_PS       (6U)          //!< Bit position for FB_CSCRn_PS.
#define BM_FB_CSCRn_PS       (0x000000C0U) //!< Bit mask for FB_CSCRn_PS.
#define BS_FB_CSCRn_PS       (2U)          //!< Bit field size in bits for FB_CSCRn_PS.

#ifndef __LANGUAGE_ASM__
//! @brief Read current value of the FB_CSCRn_PS field.
#define BR_FB_CSCRn_PS(n)    (HW_FB_CSCRn(n).B.PS)
#endif

//! @brief Format value for bitfield FB_CSCRn_PS.
#define BF_FB_CSCRn_PS(v)    (__REG_VALUE_TYPE((__REG_VALUE_TYPE((v), uint32_t) << BP_FB_CSCRn_PS), uint32_t) & BM_FB_CSCRn_PS)

#ifndef __LANGUAGE_ASM__
//! @brief Set the PS field to a new value.
#define BW_FB_CSCRn_PS(n, v) (HW_FB_CSCRn_WR(n, (HW_FB_CSCRn_RD(n) & ~BM_FB_CSCRn_PS) | BF_FB_CSCRn_PS(v)))
#endif
//@}

/*!
 * @name Register FB_CSCRn, field AA[8] (RW)
 *
 * Values:
 * - 0 - No internal FB_TA is asserted. Cycle is terminated externally
 * - 1 - Internal transfer acknowledge is asserted as specified by WS
 */
//@{
#define BP_FB_CSCRn_AA       (8U)          //!< Bit position for FB_CSCRn_AA.
#define BM_FB_CSCRn_AA       (0x00000100U) //!< Bit mask for FB_CSCRn_AA.
#define BS_FB_CSCRn_AA       (1U)          //!< Bit field size in bits for FB_CSCRn_AA.

#ifndef __LANGUAGE_ASM__
//! @brief Read current value of the FB_CSCRn_AA field.
#define BR_FB_CSCRn_AA(n)    (BITBAND_ACCESS32(HW_FB_CSCRn_ADDR(n), BP_FB_CSCRn_AA))
#endif

//! @brief Format value for bitfield FB_CSCRn_AA.
#define BF_FB_CSCRn_AA(v)    (__REG_VALUE_TYPE((__REG_VALUE_TYPE((v), uint32_t) << BP_FB_CSCRn_AA), uint32_t) & BM_FB_CSCRn_AA)

#ifndef __LANGUAGE_ASM__
//! @brief Set the AA field to a new value.
#define BW_FB_CSCRn_AA(n, v) (BITBAND_ACCESS32(HW_FB_CSCRn_ADDR(n), BP_FB_CSCRn_AA) = (v))
#endif
//@}

/*!
 * @name Register FB_CSCRn, field BLS[9] (RW)
 *
 * Values:
 * - 0 - Not shifted. Data is left-justfied on FB_AD.
 * - 1 - Shifted. Data is right justified on FB_AD.
 */
//@{
#define BP_FB_CSCRn_BLS      (9U)          //!< Bit position for FB_CSCRn_BLS.
#define BM_FB_CSCRn_BLS      (0x00000200U) //!< Bit mask for FB_CSCRn_BLS.
#define BS_FB_CSCRn_BLS      (1U)          //!< Bit field size in bits for FB_CSCRn_BLS.

#ifndef __LANGUAGE_ASM__
//! @brief Read current value of the FB_CSCRn_BLS field.
#define BR_FB_CSCRn_BLS(n)   (BITBAND_ACCESS32(HW_FB_CSCRn_ADDR(n), BP_FB_CSCRn_BLS))
#endif

//! @brief Format value for bitfield FB_CSCRn_BLS.
#define BF_FB_CSCRn_BLS(v)   (__REG_VALUE_TYPE((__REG_VALUE_TYPE((v), uint32_t) << BP_FB_CSCRn_BLS), uint32_t) & BM_FB_CSCRn_BLS)

#ifndef __LANGUAGE_ASM__
//! @brief Set the BLS field to a new value.
#define BW_FB_CSCRn_BLS(n, v) (BITBAND_ACCESS32(HW_FB_CSCRn_ADDR(n), BP_FB_CSCRn_BLS) = (v))
#endif
//@}

/*!
 * @name Register FB_CSCRn, field WS[15:10] (RW)
 */
//@{
#define BP_FB_CSCRn_WS       (10U)         //!< Bit position for FB_CSCRn_WS.
#define BM_FB_CSCRn_WS       (0x0000FC00U) //!< Bit mask for FB_CSCRn_WS.
#define BS_FB_CSCRn_WS       (6U)          //!< Bit field size in bits for FB_CSCRn_WS.

#ifndef __LANGUAGE_ASM__
//! @brief Read current value of the FB_CSCRn_WS field.
#define BR_FB_CSCRn_WS(n)    (HW_FB_CSCRn(n).B.WS)
#endif

//! @brief Format value for bitfield FB_CSCRn_WS.
#define BF_FB_CSCRn_WS(v)    (__REG_VALUE_TYPE((__REG_VALUE_TYPE((v), uint32_t) << BP_FB_CSCRn_WS), uint32_t) & BM_FB_CSCRn_WS)

#ifndef __LANGUAGE_ASM__
//! @brief Set the WS field to a new value.
#define BW_FB_CSCRn_WS(n, v) (HW_FB_CSCRn_WR(n, (HW_FB_CSCRn_RD(n) & ~BM_FB_CSCRn_WS) | BF_FB_CSCRn_WS(v)))
#endif
//@}

/*!
 * @name Register FB_CSCRn, field WRAH[17:16] (RW)
 *
 * Values:
 * - 00 - Hold address and attributes one cycle after FB_CSn negates on writes.
 *     (Default FB_CSn)
 * - 01 - Hold address and attributes two cycles after FB_CSn negates on writes.
 * - 10 - Hold address and attributes three cycles after FB_CSn negates on
 *     writes.
 * - 11 - Hold address and attributes four cycles after FB_CSn negates on
 *     writes. (Default FB_CS0)
 */
//@{
#define BP_FB_CSCRn_WRAH     (16U)         //!< Bit position for FB_CSCRn_WRAH.
#define BM_FB_CSCRn_WRAH     (0x00030000U) //!< Bit mask for FB_CSCRn_WRAH.
#define BS_FB_CSCRn_WRAH     (2U)          //!< Bit field size in bits for FB_CSCRn_WRAH.

#ifndef __LANGUAGE_ASM__
//! @brief Read current value of the FB_CSCRn_WRAH field.
#define BR_FB_CSCRn_WRAH(n)  (HW_FB_CSCRn(n).B.WRAH)
#endif

//! @brief Format value for bitfield FB_CSCRn_WRAH.
#define BF_FB_CSCRn_WRAH(v)  (__REG_VALUE_TYPE((__REG_VALUE_TYPE((v), uint32_t) << BP_FB_CSCRn_WRAH), uint32_t) & BM_FB_CSCRn_WRAH)

#ifndef __LANGUAGE_ASM__
//! @brief Set the WRAH field to a new value.
#define BW_FB_CSCRn_WRAH(n, v) (HW_FB_CSCRn_WR(n, (HW_FB_CSCRn_RD(n) & ~BM_FB_CSCRn_WRAH) | BF_FB_CSCRn_WRAH(v)))
#endif
//@}

/*!
 * @name Register FB_CSCRn, field RDAH[19:18] (RW)
 *
 * Values:
 * - 00 - If AA is cleared, 1 cycle. If AA is set, 0 cycles.
 * - 01 - If AA is cleared, 2 cycles. If AA is set, 1 cycle.
 * - 10 - If AA is cleared, 3 cycles. If AA is set, 2 cycles.
 * - 11 - If AA is cleared, 4 cycles. If AA is set, 3 cycles.
 */
//@{
#define BP_FB_CSCRn_RDAH     (18U)         //!< Bit position for FB_CSCRn_RDAH.
#define BM_FB_CSCRn_RDAH     (0x000C0000U) //!< Bit mask for FB_CSCRn_RDAH.
#define BS_FB_CSCRn_RDAH     (2U)          //!< Bit field size in bits for FB_CSCRn_RDAH.

#ifndef __LANGUAGE_ASM__
//! @brief Read current value of the FB_CSCRn_RDAH field.
#define BR_FB_CSCRn_RDAH(n)  (HW_FB_CSCRn(n).B.RDAH)
#endif

//! @brief Format value for bitfield FB_CSCRn_RDAH.
#define BF_FB_CSCRn_RDAH(v)  (__REG_VALUE_TYPE((__REG_VALUE_TYPE((v), uint32_t) << BP_FB_CSCRn_RDAH), uint32_t) & BM_FB_CSCRn_RDAH)

#ifndef __LANGUAGE_ASM__
//! @brief Set the RDAH field to a new value.
#define BW_FB_CSCRn_RDAH(n, v) (HW_FB_CSCRn_WR(n, (HW_FB_CSCRn_RD(n) & ~BM_FB_CSCRn_RDAH) | BF_FB_CSCRn_RDAH(v)))
#endif
//@}

/*!
 * @name Register FB_CSCRn, field ASET[21:20] (RW)
 *
 * Values:
 * - 00 - Assert FB_CSn on first rising clock edge after address is asserted.
 *     (Default FB_CSn)
 * - 01 - Assert FB_CSn on second rising clock edge after address is asserted.
 * - 10 - Assert FB_CSn on third rising clock edge after address is asserted.
 * - 11 - Assert FB_CSn on fourth rising clock edge after address is asserted.
 *     (Default FB_CS0)
 */
//@{
#define BP_FB_CSCRn_ASET     (20U)         //!< Bit position for FB_CSCRn_ASET.
#define BM_FB_CSCRn_ASET     (0x00300000U) //!< Bit mask for FB_CSCRn_ASET.
#define BS_FB_CSCRn_ASET     (2U)          //!< Bit field size in bits for FB_CSCRn_ASET.

#ifndef __LANGUAGE_ASM__
//! @brief Read current value of the FB_CSCRn_ASET field.
#define BR_FB_CSCRn_ASET(n)  (HW_FB_CSCRn(n).B.ASET)
#endif

//! @brief Format value for bitfield FB_CSCRn_ASET.
#define BF_FB_CSCRn_ASET(v)  (__REG_VALUE_TYPE((__REG_VALUE_TYPE((v), uint32_t) << BP_FB_CSCRn_ASET), uint32_t) & BM_FB_CSCRn_ASET)

#ifndef __LANGUAGE_ASM__
//! @brief Set the ASET field to a new value.
#define BW_FB_CSCRn_ASET(n, v) (HW_FB_CSCRn_WR(n, (HW_FB_CSCRn_RD(n) & ~BM_FB_CSCRn_ASET) | BF_FB_CSCRn_ASET(v)))
#endif
//@}

/*!
 * @name Register FB_CSCRn, field EXTS[22] (RW)
 *
 * Values:
 * - 0 - FB_TS /FB_ALE asserts for one bus clock cycle
 * - 1 - FB_TS /FB_ALE remains asserted until the first positive clock edge
 *     after FB_CSn asserts
 */
//@{
#define BP_FB_CSCRn_EXTS     (22U)         //!< Bit position for FB_CSCRn_EXTS.
#define BM_FB_CSCRn_EXTS     (0x00400000U) //!< Bit mask for FB_CSCRn_EXTS.
#define BS_FB_CSCRn_EXTS     (1U)          //!< Bit field size in bits for FB_CSCRn_EXTS.

#ifndef __LANGUAGE_ASM__
//! @brief Read current value of the FB_CSCRn_EXTS field.
#define BR_FB_CSCRn_EXTS(n)  (BITBAND_ACCESS32(HW_FB_CSCRn_ADDR(n), BP_FB_CSCRn_EXTS))
#endif

//! @brief Format value for bitfield FB_CSCRn_EXTS.
#define BF_FB_CSCRn_EXTS(v)  (__REG_VALUE_TYPE((__REG_VALUE_TYPE((v), uint32_t) << BP_FB_CSCRn_EXTS), uint32_t) & BM_FB_CSCRn_EXTS)

#ifndef __LANGUAGE_ASM__
//! @brief Set the EXTS field to a new value.
#define BW_FB_CSCRn_EXTS(n, v) (BITBAND_ACCESS32(HW_FB_CSCRn_ADDR(n), BP_FB_CSCRn_EXTS) = (v))
#endif
//@}

/*!
 * @name Register FB_CSCRn, field SWSEN[23] (RW)
 *
 * Values:
 * - 0 - The WS value inserts wait states before an internal transfer
 *     acknowledge is generated for all transfers
 * - 1 - The SWS value inserts wait states before an internal transfer
 *     acknowledge is generated for burst transfer secondary terminations
 */
//@{
#define BP_FB_CSCRn_SWSEN    (23U)         //!< Bit position for FB_CSCRn_SWSEN.
#define BM_FB_CSCRn_SWSEN    (0x00800000U) //!< Bit mask for FB_CSCRn_SWSEN.
#define BS_FB_CSCRn_SWSEN    (1U)          //!< Bit field size in bits for FB_CSCRn_SWSEN.

#ifndef __LANGUAGE_ASM__
//! @brief Read current value of the FB_CSCRn_SWSEN field.
#define BR_FB_CSCRn_SWSEN(n) (BITBAND_ACCESS32(HW_FB_CSCRn_ADDR(n), BP_FB_CSCRn_SWSEN))
#endif

//! @brief Format value for bitfield FB_CSCRn_SWSEN.
#define BF_FB_CSCRn_SWSEN(v) (__REG_VALUE_TYPE((__REG_VALUE_TYPE((v), uint32_t) << BP_FB_CSCRn_SWSEN), uint32_t) & BM_FB_CSCRn_SWSEN)

#ifndef __LANGUAGE_ASM__
//! @brief Set the SWSEN field to a new value.
#define BW_FB_CSCRn_SWSEN(n, v) (BITBAND_ACCESS32(HW_FB_CSCRn_ADDR(n), BP_FB_CSCRn_SWSEN) = (v))
#endif
//@}

/*!
 * @name Register FB_CSCRn, field SWS[31:26] (RW)
 */
//@{
#define BP_FB_CSCRn_SWS      (26U)         //!< Bit position for FB_CSCRn_SWS.
#define BM_FB_CSCRn_SWS      (0xFC000000U) //!< Bit mask for FB_CSCRn_SWS.
#define BS_FB_CSCRn_SWS      (6U)          //!< Bit field size in bits for FB_CSCRn_SWS.

#ifndef __LANGUAGE_ASM__
//! @brief Read current value of the FB_CSCRn_SWS field.
#define BR_FB_CSCRn_SWS(n)   (HW_FB_CSCRn(n).B.SWS)
#endif

//! @brief Format value for bitfield FB_CSCRn_SWS.
#define BF_FB_CSCRn_SWS(v)   (__REG_VALUE_TYPE((__REG_VALUE_TYPE((v), uint32_t) << BP_FB_CSCRn_SWS), uint32_t) & BM_FB_CSCRn_SWS)

#ifndef __LANGUAGE_ASM__
//! @brief Set the SWS field to a new value.
#define BW_FB_CSCRn_SWS(n, v) (HW_FB_CSCRn_WR(n, (HW_FB_CSCRn_RD(n) & ~BM_FB_CSCRn_SWS) | BF_FB_CSCRn_SWS(v)))
#endif
//@}

//-------------------------------------------------------------------------------------------
// HW_FB_CSPMCR - Chip select port multiplexing control register
//-------------------------------------------------------------------------------------------

#ifndef __LANGUAGE_ASM__
/*!
 * @brief HW_FB_CSPMCR - Chip select port multiplexing control register (RW)
 *
 * Reset value: 0x00000000U
 *
 * The CSPMCR register controls the multiplexing of the FlexBus signals. A bus
 * error occurs when: writing a reserved value, writing to a reserved bit location
 * in this register, or not accessing this register as 32-bit.
 */
typedef union _hw_fb_cspmcr
{
    uint32_t U;
    struct _hw_fb_cspmcr_bitfields
    {
        uint32_t RESERVED0 : 12;       //!< [11:0]
        uint32_t GROUP5 : 4;           //!< [15:12] FlexBus signal group 5 multiplex
                                       //! control
        uint32_t GROUP4 : 4;           //!< [19:16] FlexBus signal group 4 multiplex
                                       //! control
        uint32_t GROUP3 : 4;           //!< [23:20] FlexBus signal group 3 multiplex
                                       //! control
        uint32_t GROUP2 : 4;           //!< [27:24] FlexBus signal group 2 multiplex
                                       //! control
        uint32_t GROUP1 : 4;           //!< [31:28] FlexBus signal group 1 multiplex
                                       //! control
    } B;
} hw_fb_cspmcr_t;
#endif

/*!
 * @name Constants and macros for entire FB_CSPMCR register
 */
//@{
#define HW_FB_CSPMCR_ADDR        (REGS_FB_BASE + 0x60U)

#ifndef __LANGUAGE_ASM__
#define HW_FB_CSPMCR             (*(__IO hw_fb_cspmcr_t *) HW_FB_CSPMCR_ADDR)
#define HW_FB_CSPMCR_RD()        (HW_FB_CSPMCR.U)
#define HW_FB_CSPMCR_WR(v)       (HW_FB_CSPMCR.U = (v))
#define HW_FB_CSPMCR_SET(v)      (HW_FB_CSPMCR_WR(HW_FB_CSPMCR_RD() |  (v)))
#define HW_FB_CSPMCR_CLR(v)      (HW_FB_CSPMCR_WR(HW_FB_CSPMCR_RD() & ~(v)))
#define HW_FB_CSPMCR_TOG(v)      (HW_FB_CSPMCR_WR(HW_FB_CSPMCR_RD() ^  (v)))
#endif
//@}

/*
 * Constants & macros for individual FB_CSPMCR bitfields
 */

/*!
 * @name Register FB_CSPMCR, field GROUP5[15:12] (RW)
 *
 * Controls the multiplexing of the FB_TA, FB_CS3, and FB_BE_7_0 signals. When
 * GROUP5 is not 0000, you must set the CSCRn[AA] bit. Else, the bus hangs during
 * a transfer.
 *
 * Values:
 * - 0000 - FB_TA
 * - 0001 - FB_CS3. You must also set CSCRn[AA].
 * - 0010 - FB_BE_7_0. You must also set CSCRn[AA].
 */
//@{
#define BP_FB_CSPMCR_GROUP5  (12U)         //!< Bit position for FB_CSPMCR_GROUP5.
#define BM_FB_CSPMCR_GROUP5  (0x0000F000U) //!< Bit mask for FB_CSPMCR_GROUP5.
#define BS_FB_CSPMCR_GROUP5  (4U)          //!< Bit field size in bits for FB_CSPMCR_GROUP5.

#ifndef __LANGUAGE_ASM__
//! @brief Read current value of the FB_CSPMCR_GROUP5 field.
#define BR_FB_CSPMCR_GROUP5  (HW_FB_CSPMCR.B.GROUP5)
#endif

//! @brief Format value for bitfield FB_CSPMCR_GROUP5.
#define BF_FB_CSPMCR_GROUP5(v) (__REG_VALUE_TYPE((__REG_VALUE_TYPE((v), uint32_t) << BP_FB_CSPMCR_GROUP5), uint32_t) & BM_FB_CSPMCR_GROUP5)

#ifndef __LANGUAGE_ASM__
//! @brief Set the GROUP5 field to a new value.
#define BW_FB_CSPMCR_GROUP5(v) (HW_FB_CSPMCR_WR((HW_FB_CSPMCR_RD() & ~BM_FB_CSPMCR_GROUP5) | BF_FB_CSPMCR_GROUP5(v)))
#endif
//@}

/*!
 * @name Register FB_CSPMCR, field GROUP4[19:16] (RW)
 *
 * Controls the multiplexing of the FB_TBST, FB_CS2, and FB_BE_15_8 signals.
 *
 * Values:
 * - 0000 - FB_TBST
 * - 0001 - FB_CS2
 * - 0010 - FB_BE_15_8
 */
//@{
#define BP_FB_CSPMCR_GROUP4  (16U)         //!< Bit position for FB_CSPMCR_GROUP4.
#define BM_FB_CSPMCR_GROUP4  (0x000F0000U) //!< Bit mask for FB_CSPMCR_GROUP4.
#define BS_FB_CSPMCR_GROUP4  (4U)          //!< Bit field size in bits for FB_CSPMCR_GROUP4.

#ifndef __LANGUAGE_ASM__
//! @brief Read current value of the FB_CSPMCR_GROUP4 field.
#define BR_FB_CSPMCR_GROUP4  (HW_FB_CSPMCR.B.GROUP4)
#endif

//! @brief Format value for bitfield FB_CSPMCR_GROUP4.
#define BF_FB_CSPMCR_GROUP4(v) (__REG_VALUE_TYPE((__REG_VALUE_TYPE((v), uint32_t) << BP_FB_CSPMCR_GROUP4), uint32_t) & BM_FB_CSPMCR_GROUP4)

#ifndef __LANGUAGE_ASM__
//! @brief Set the GROUP4 field to a new value.
#define BW_FB_CSPMCR_GROUP4(v) (HW_FB_CSPMCR_WR((HW_FB_CSPMCR_RD() & ~BM_FB_CSPMCR_GROUP4) | BF_FB_CSPMCR_GROUP4(v)))
#endif
//@}

/*!
 * @name Register FB_CSPMCR, field GROUP3[23:20] (RW)
 *
 * Controls the multiplexing of the FB_CS5, FB_TSIZ1, and FB_BE_23_16 signals.
 *
 * Values:
 * - 0000 - FB_CS5
 * - 0001 - FB_TSIZ1
 * - 0010 - FB_BE_23_16
 */
//@{
#define BP_FB_CSPMCR_GROUP3  (20U)         //!< Bit position for FB_CSPMCR_GROUP3.
#define BM_FB_CSPMCR_GROUP3  (0x00F00000U) //!< Bit mask for FB_CSPMCR_GROUP3.
#define BS_FB_CSPMCR_GROUP3  (4U)          //!< Bit field size in bits for FB_CSPMCR_GROUP3.

#ifndef __LANGUAGE_ASM__
//! @brief Read current value of the FB_CSPMCR_GROUP3 field.
#define BR_FB_CSPMCR_GROUP3  (HW_FB_CSPMCR.B.GROUP3)
#endif

//! @brief Format value for bitfield FB_CSPMCR_GROUP3.
#define BF_FB_CSPMCR_GROUP3(v) (__REG_VALUE_TYPE((__REG_VALUE_TYPE((v), uint32_t) << BP_FB_CSPMCR_GROUP3), uint32_t) & BM_FB_CSPMCR_GROUP3)

#ifndef __LANGUAGE_ASM__
//! @brief Set the GROUP3 field to a new value.
#define BW_FB_CSPMCR_GROUP3(v) (HW_FB_CSPMCR_WR((HW_FB_CSPMCR_RD() & ~BM_FB_CSPMCR_GROUP3) | BF_FB_CSPMCR_GROUP3(v)))
#endif
//@}

/*!
 * @name Register FB_CSPMCR, field GROUP2[27:24] (RW)
 *
 * Controls the multiplexing of the FB_CS4, FB_TSIZ0, and FB_BE_31_24 signals.
 *
 * Values:
 * - 0000 - FB_CS4
 * - 0001 - FB_TSIZ0
 * - 0010 - FB_BE_31_24
 */
//@{
#define BP_FB_CSPMCR_GROUP2  (24U)         //!< Bit position for FB_CSPMCR_GROUP2.
#define BM_FB_CSPMCR_GROUP2  (0x0F000000U) //!< Bit mask for FB_CSPMCR_GROUP2.
#define BS_FB_CSPMCR_GROUP2  (4U)          //!< Bit field size in bits for FB_CSPMCR_GROUP2.

#ifndef __LANGUAGE_ASM__
//! @brief Read current value of the FB_CSPMCR_GROUP2 field.
#define BR_FB_CSPMCR_GROUP2  (HW_FB_CSPMCR.B.GROUP2)
#endif

//! @brief Format value for bitfield FB_CSPMCR_GROUP2.
#define BF_FB_CSPMCR_GROUP2(v) (__REG_VALUE_TYPE((__REG_VALUE_TYPE((v), uint32_t) << BP_FB_CSPMCR_GROUP2), uint32_t) & BM_FB_CSPMCR_GROUP2)

#ifndef __LANGUAGE_ASM__
//! @brief Set the GROUP2 field to a new value.
#define BW_FB_CSPMCR_GROUP2(v) (HW_FB_CSPMCR_WR((HW_FB_CSPMCR_RD() & ~BM_FB_CSPMCR_GROUP2) | BF_FB_CSPMCR_GROUP2(v)))
#endif
//@}

/*!
 * @name Register FB_CSPMCR, field GROUP1[31:28] (RW)
 *
 * Controls the multiplexing of the FB_ALE, FB_CS1, and FB_TS signals.
 *
 * Values:
 * - 0000 - FB_ALE
 * - 0001 - FB_CS1
 * - 0010 - FB_TS
 */
//@{
#define BP_FB_CSPMCR_GROUP1  (28U)         //!< Bit position for FB_CSPMCR_GROUP1.
#define BM_FB_CSPMCR_GROUP1  (0xF0000000U) //!< Bit mask for FB_CSPMCR_GROUP1.
#define BS_FB_CSPMCR_GROUP1  (4U)          //!< Bit field size in bits for FB_CSPMCR_GROUP1.

#ifndef __LANGUAGE_ASM__
//! @brief Read current value of the FB_CSPMCR_GROUP1 field.
#define BR_FB_CSPMCR_GROUP1  (HW_FB_CSPMCR.B.GROUP1)
#endif

//! @brief Format value for bitfield FB_CSPMCR_GROUP1.
#define BF_FB_CSPMCR_GROUP1(v) (__REG_VALUE_TYPE((__REG_VALUE_TYPE((v), uint32_t) << BP_FB_CSPMCR_GROUP1), uint32_t) & BM_FB_CSPMCR_GROUP1)

#ifndef __LANGUAGE_ASM__
//! @brief Set the GROUP1 field to a new value.
#define BW_FB_CSPMCR_GROUP1(v) (HW_FB_CSPMCR_WR((HW_FB_CSPMCR_RD() & ~BM_FB_CSPMCR_GROUP1) | BF_FB_CSPMCR_GROUP1(v)))
#endif
//@}

//-------------------------------------------------------------------------------------------
// hw_fb_t - module struct
//-------------------------------------------------------------------------------------------
/*!
 * @brief All FB module registers.
 */
#ifndef __LANGUAGE_ASM__
#pragma pack(1)
typedef struct _hw_fb
{
    struct {
        __IO hw_fb_csarn_t CSARn;          //!< [0x0] Chip select address register
        __IO hw_fb_csmrn_t CSMRn;          //!< [0x4] Chip select mask register
        __IO hw_fb_cscrn_t CSCRn;          //!< [0x8] Chip select control register
    } CS[6];
    uint8_t _reserved0[24];
    __IO hw_fb_cspmcr_t CSPMCR;            //!< [0x60] Chip select port multiplexing control register
} hw_fb_t;
#pragma pack()

//! @brief Macro to access all FB registers.
//! @return Reference (not a pointer) to the registers struct. To get a pointer to the struct,
//!     use the '&' operator, like <code>&HW_FB</code>.
#define HW_FB          (*(hw_fb_t *) REGS_FB_BASE)
#endif

#endif // __HW_FB_REGISTERS_H__
// v22/130726/0.9
// EOF
