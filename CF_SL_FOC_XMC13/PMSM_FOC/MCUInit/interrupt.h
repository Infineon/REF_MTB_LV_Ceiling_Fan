/**
 * @file interrupt.h
 * @date 2015-10-05
 *
 * NOTE:
 * This file is generated by DAVE. Any manual modification done to this file will be lost when the code is
 * regenerated.
 */
/**
 * @cond
 ***********************************************************************************************************************
 * INTERRUPT v4.0.8 Helps the user to overwrite the provided ISR in system file
 *
 * Copyright (c) 2015, Infineon Technologies AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,are permitted provided that the
 * following conditions are met:
 *
 *   Redistributions of source code must retain the above copyright notice, this list of conditions and the  following
 *   disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
 *   following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 *   Neither the name of the copyright holders nor the names of its contributors may be used to endorse or promote
 *   products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE  FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY,OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT  OF THE
 * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * To improve the quality of the software, users are encouraged to share modifications, enhancements or bug fixes
 * with Infineon Technologies AG (dave@infineon.com).
 ***********************************************************************************************************************
 *
 * Change History
 * --------------
 *
 * 2015-02-16:
 *     - Initial version<br>
 *
 * 2015-05-08:
 *     - subpriority field is biased based on family<br>
 *
 * 2015-07-30:
 *     - Added a field named "irqctrl" for XMC1400 devices
 *
 * 2015-10-05:
 *     - Merged config elements into the APP structure
 * @endcond
 *
 */

#ifndef INTERRUPT_H
#define INTERRUPT_H

/***********************************************************************************************************************
 * HEADER FILES
 **********************************************************************************************************************/
#include "..\Configuration\pmsm_foc_variables_scaling.h"

#if(IR_Remote_Control == ENABLED)
#include <xmc_common.h>

#if (UC_SERIES == XMC14)
#include <xmc_scu.h>
#endif

#define IR_Recv_ISR IRQ_Hdlr_21

//#include "interrupt_conf.h"


/**
 * @addtogroup INTERRUPT
 * @{
 */

/*********************************************************************************************************************
 * MACROS
 ********************************************************************************************************************/

/*********************************************************************************************************************
 * ENUMS
 ********************************************************************************************************************/
/**
 * @ingroup INTERRUPT_enumerations
 * @{
 */
/**
 * Initialization status.
 */
typedef enum INTERRUPT_STATUS
{
  INTERRUPT_STATUS_SUCCESS = 0U,  /**< APP initialization success */
  INTERRUPT_STATUS_FAILURE = 1U   /**< APP initialization failure */
} INTERRUPT_STATUS_t;
/**
 * @}
 */

/*********************************************************************************************************************
 * DATA STRUCTURES
 *********************************************************************************************************************/
/**
 * @ingroup INTERRUPT_datastructures
 * @{
 */

/**
 * @brief This structure holds run-time configurations of INTERRUPT APP.
 */
typedef struct INTERRUPT
{
#if(UC_SERIES == XMC14)
  const XMC_SCU_IRQCTRL_t irqctrl;  /**< selects the interrupt source for a NVIC interrupt node*/
#endif	
  const IRQn_Type node;       /**< Mapped NVIC Node */
  const uint8_t priority; 	  /**< Node Interrupt Priority */
#if(UC_FAMILY == XMC4)
  const uint8_t subpriority;  /**< Node Interrupt SubPriority only valid for XMC4x */
#endif  
  const bool enable_at_init;  /**< Interrupt enable for Node */
} INTERRUPT_t;

/**
 * @}
 */

/*********************************************************************************************************************
 * API PROTOTYPES
 *********************************************************************************************************************/

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @ingroup INTERRUPT_apidoc
 * @{
 */

/**
 * @brief Get INTERRUPT APP version.
 * @return @ref DAVE_APP_VERSION_t APP version information (major, minor and
 *                                 patch number)
 *
 * \par<b>Description: </b><br>
 * The function can be used to check application software compatibility with a
 * specific version of the APP.
 * @code
 * #include <DAVE.h>
 *
 * int main(void)
 * {
 *   DAVE_APP_VERSION_t version;
 *   DAVE_Init();
 *   version = INTERRUPT_GetAppVersion();
 *   if(version.major != 4U)
 *   {
 *   }
 *   while(1)
 *   {}
 *   return 0;
 * }
 * @endcode<BR> </p>
 */
//DAVE_APP_VERSION_t INTERRUPT_GetAppVersion(void);
/**
 * @brief Initializes INTERRUPT APP instance.
 * @param handle Constant pointer to constant structure of type @ref INTERRUPT_t
 * @return @ref INTERRUPT_STATUS_t
 * <BR><P ALIGN="LEFT"><B>Example:</B>
 * Pre-requisite: Instantiate one instance of INTERRUPT APP
 * @code
 *  #include <DAVE.h>
 *
 *  int main(void)
 *  {
 *    DAVE_Init();  // INTERRUPT_Init(&INTERRUPT_0) is called within DAVE_Init()
 *    while(1)
 *    {}
 *    return 0;
 *  }
 * @endcode<BR> </p>
 *
 */
INTERRUPT_STATUS_t INTERRUPT_Init(const INTERRUPT_t *const handler);

/**
 * @brief Enables the IRQ.
 * @param handle Constant pointer to constant structure of type @ref INTERRUPT_t
 * @return None
 * <BR><P ALIGN="LEFT"><B>Example:</B>
 * Pre-requisite: Instantiate one instance of INTERRUPT APP
 * @code
 *  #include <DAVE.h>
 *
 *  int main(void)
 *  {
 *    DAVE_Init(); // INTERRUPT_Init() is called within DAVE_Init()
 *    INTERRUPT_Enable(&INTERRUPT_0);
 *    while(1)
 *    {}
 *    return 0;
 *  }
 * @endcode<BR> </p>
 */
__STATIC_INLINE void INTERRUPT_Enable(const INTERRUPT_t *const handler)
{
  XMC_ASSERT("Handler NULL", (handler != NULL));
  NVIC_EnableIRQ(handler->node);
}

/**
 * @brief Disables the IRQ.
 * @param handle Constant pointer to constant structure of type @ref INTERRUPT_t
 * @return None
 * <BR><P ALIGN="LEFT"><B>Example:</B>
 * Pre-requisite: Instantiate one instance of INTERRUPT APP
 * @code
 *  #include <DAVE.h>
 *
 *  int main(void)
 *  {
 *    DAVE_Init(); // INTERRUPT_Init() is called within DAVE_Init()
 *    INTERRUPT_Disable(&INTERRUPT_0);
 *    while(1)
 *    {}
 *    return 0;
 *  }
 * @endcode<BR> </p>
 */
__STATIC_INLINE void INTERRUPT_Disable(const INTERRUPT_t *const handler)
{
  XMC_ASSERT("Handler NULL", (handler != NULL));
  NVIC_DisableIRQ(handler->node);
}

/**
 * @brief Get the pending IRQ.
 * @param handle Constant pointer to constant structure of type @ref INTERRUPT_t
 * @return uint32_t IRQ node
 * <BR><P ALIGN="LEFT"><B>Example:</B>
 * Pre-requisite: Instantiate one instance of INTERRUPT APP
 * @code
 *  #include <DAVE.h>
 *
 *  int main(void)
 *  {
 *    uint32_t Status;
 *    DAVE_Init(); // INTERRUPT_Init() is called within DAVE_Init()
 *    Status = INTERRUPT_GetPending(&INTERRUPT_0);
 *    while(1)
 *    {}
 *    return 0;
 *  }
 * @endcode<BR> </p>
 */
__STATIC_INLINE uint32_t INTERRUPT_GetPending(const INTERRUPT_t *const handler)
{
  XMC_ASSERT("Handler NULL", (handler != NULL));
  return NVIC_GetPendingIRQ(handler->node);
}

/**
 * @brief Set the IRQ to pending state.
 * @param handle Constant pointer to constant structure of type @ref INTERRUPT_t
 * @return None
 * <BR><P ALIGN="LEFT"><B>Example:</B>
 * Pre-requisite: Instantiate one instance of INTERRUPT APP
 * @code
 *  #include <DAVE.h>
 *
 *  int main(void)
 *  {
 *    DAVE_Init(); // INTERRUPT_Init() is called within DAVE_Init()
 *    INTERRUPT_SetPending(&INTERRUPT_0);
 *    while(1)
 *    {}
 *    return 0;
 *  }
 * @endcode<BR> </p>
 */
__STATIC_INLINE void INTERRUPT_SetPending(const INTERRUPT_t *const handler)
{
  XMC_ASSERT("Handler NULL", (handler != NULL));
  NVIC_SetPendingIRQ(handler->node);
}

/**
 * @brief Clears the pending status of the IRQ.
 * @param handle Constant pointer to constant structure of type @ref INTERRUPT_t
 * @return None
 * <BR><P ALIGN="LEFT"><B>Example:</B>
 * Pre-requisite: Instantiate two instances of INTERRUPT APP
 * @code
 *  #include <DAVE.h>
 *
 *  uint32_t pend_IRQ;
 *  int main(void)
 *  {
 *    DAVE_Init(); // INTERRUPT_Init() is called within DAVE_Init()
 *    INTERRUPT_Enable(&INTERRUPT_0);
 *    while(1)
 *    {}
 *    return 0;
 *  }
 *
 *  void MyISR_handler(void)
 *  {
 *    INTERRUPT_Enable(&INTERRUPT_1);
 *    INTERRUPT_SetPending(&INTERRUPT_1);
 *    pend_IRQ = INTERRUPT_GetPending(&INTERRUPT_1);
 *    if(pend_IRQ)
 *    {
 *      INTERRUPT_Disable(&INTERRUPT_0);
 *      INTERRUPT_ClearPending(&INTERRUPT_1);
 *    }
 *  }
 * @endcode<BR> </p>
 */
__STATIC_INLINE void INTERRUPT_ClearPending(const INTERRUPT_t *const handler)
{
  XMC_ASSERT("Handler NULL", (handler != NULL));
  NVIC_ClearPendingIRQ(handler->node);
}

#if(UC_FAMILY == XMC4)
/**
 * @brief Get current running active status of the IRQ. This API is applicable
 *        only for XMC4000 devices.
 * @param handle Constant pointer to constant structure of type @ref INTERRUPT_t
 * @return uint32_t current active running IRQ node
 * <BR><P ALIGN="LEFT"><B>Example:</B>
 * Pre-requisite: Instantiate one instance of INTERRUPT APP
 * @code
 *  #include <DAVE.h>
 *
 *  int main(void)
 *  {
 *    uint32_t Status;
 *    DAVE_Init(); // INTERRUPT_Init() is called within DAVE_Init()
 *    Status = INTERRUPT_GetActive(&INTERRUPT_0);
 *    while(1)
 *    {}
 *    return 0;
 *  }
 * @endcode<BR> </p>
 */
__STATIC_INLINE uint32_t INTERRUPT_GetActive(const INTERRUPT_t *const handler)
{
  XMC_ASSERT("Handler NULL", (handler != NULL));
  return NVIC_GetActive(handler->node);
}

#endif

#ifdef __cplusplus
}
#endif

//#include "interrupt_extern.h"
extern const INTERRUPT_t irrecv_isr_interrupt;

#endif /*#if(IR_Remote_Control == ENABLED)*/

#endif /*#ifndef INTERRUPT_H*/
/**
 *@}
 */
/**
 * @}
 */
