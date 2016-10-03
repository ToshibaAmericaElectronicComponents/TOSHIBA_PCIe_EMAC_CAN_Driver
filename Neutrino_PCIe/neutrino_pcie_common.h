/* ============================================================================
 * COPYRIGHT © 2015
 *
 * Toshiba America Electronic Components
 *
 * PROJECT:   NEUTRINO
 *
 * Permission is hereby granted,
 * free of charge, to any person obtaining a copy of this software annotated
 * with this license and the Software, to deal in the Software without
 * restriction, including without limitation the rights to use, copy, modify,
 * merge, publish, distribute, sublicense, and/or sell copies of the Software,
 * and to permit persons to whom the Software is furnished to do so, subject
 * to the following conditions:
 *
 *
 * EXAMPLE PROGRAMS ARE PROVIDED AS-IS WITH NO WARRANTY OF ANY KIND, 
 * EITHER EXPRESS OR IMPLIED.
 *
 * TOSHIBA ASSUMES NO LIABILITY FOR CUSTOMERS' PRODUCT DESIGN OR APPLICATIONS.
 * 
 * THIS SOFTWARE IS PROVIDED AS-IS AND HAS NOT BEEN FULLY TESTED.  IT IS
 * INTENDED FOR REFERENCE USE ONLY.
 * 
 * TOSHIBA DISCLAIMS ALL EXPRESS AND IMPLIED WARRANTIES AND ALL LIABILITY OR
 * ANY DAMAGES ASSOCIATED WITH YOUR USE OF THIS SOFTWARE.
 *
 * THIS SOFTWARE IS BEING DISTRIBUTED BY TOSHIBA SOLELY ON AN "AS IS" BASIS
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE HEREBY DISCLAIMED. IN NO EVENT SHALL TOSHIBA BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 *
 * ========================================================================= */

/*! History:   
 *  01-August-2016	: Initial 
 */

/*!@file: neutrino_pcie_common.c
 * @brief: Driver functions.
 */

#ifndef __Neutrino_PCIe__COMMON_H__
#define __Neutrino_PCIe__COMMON_H__


/* NOTE: Uncomment below line for function trace log messages in KERNEL LOG */
#define YDEBUG

#define Y_TRUE 														1
#define Y_FALSE 													0
#define Y_SUCCESS 												0
#define Y_FAILURE 												1
#define Y_INV_WR 													1
#define Y_INV_RD 													2
#define Y_INV_ARG 												3
#define Y_MAX_THRD_XEEDED 								4


/* C data types typedefs */
typedef unsigned short									 	BOOL;
typedef char 															CHAR;
typedef char 														 *CHARP;
typedef int 															INT;
typedef int 														 *INTP;
typedef long 															LONG;
typedef long 														 *LONGP;
typedef short  														SHORT;
typedef short 													 *SHORTP;
typedef unsigned 													UINT;
typedef unsigned 												 *UINTP;
typedef unsigned char 										UCHAR;
typedef unsigned char 										*UCHARP;
typedef unsigned long 										ULONG;
typedef unsigned long 									 *ULONGP;
typedef unsigned short									 	USHORT;
typedef unsigned short									 *USHORTP;
typedef void 															VOID;
typedef void 														 *VOIDP;


/* For debug prints*/
#ifdef RELEASE_PACKAGE
#undef NTN_DEBUG_L1
#undef NTN_DEBUG_L2
#endif

#define NMSGPR_INFO(x...)  								printk(KERN_INFO x)
#define NMSGPR_ALERT(x...) 								printk(KERN_ALERT x)
#define NMSGPR_ERR(x...)   								printk(KERN_ERR x)

#ifdef NTN_DEBUG_L1
#define NDBGPR_L1(x...) 									printk(KERN_DEBUG x)
#else
#define NDBGPR_L1(x...) 									do { } while (0)
#endif

#ifdef NTN_DEBUG_L2
#define NDBGPR_L2(x...) 									printk(KERN_DEBUG x)
#else
#define NDBGPR_L2(x...) 									do { } while (0)
#endif


#ifdef YDEBUG
#define DBGPR(x...) 											printk(KERN_ALERT x)
#define DBGPR_REGS() 											dbgpr_regs()
#else
#define DBGPR(x...) 											do { } while (0)
#define DBGPR_REGS() 											do { } while (0)
#endif


#endif
