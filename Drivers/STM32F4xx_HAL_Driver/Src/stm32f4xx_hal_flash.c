/**
  ******************************************************************************
  * @file    stm32f4xx_hal_flash.c
  * @author  MCD Application Team
  * @brief   FLASH HAL module driver.
<<<<<<< HEAD
  *          This file provides firmware functions to manage the following
  *          functionalities of the internal FLASH memory:
  *           + Program operations functions
  *           + Memory Control functions
  *           + Peripheral Errors functions
  *
=======
  *          This file provides firmware functions to manage the following 
  *          functionalities of the internal FLASH memory:
  *           + Program operations functions
  *           + Memory Control functions 
  *           + Peripheral Errors functions
  *         
>>>>>>> 4ae2208c09bd3a1352a92288a9b9e224d15faf00
  @verbatim
  ==============================================================================
                        ##### FLASH peripheral features #####
  ==============================================================================
<<<<<<< HEAD

  [..] The Flash memory interface manages CPU AHB I-Code and D-Code accesses
       to the Flash memory. It implements the erase and program Flash memory operations
       and the read and write protection mechanisms.

  [..] The Flash memory interface accelerates code execution with a system of instruction
       prefetch and cache lines.
=======
           
  [..] The Flash memory interface manages CPU AHB I-Code and D-Code accesses 
       to the Flash memory. It implements the erase and program Flash memory operations 
       and the read and write protection mechanisms.
      
  [..] The Flash memory interface accelerates code execution with a system of instruction
       prefetch and cache lines. 
>>>>>>> 4ae2208c09bd3a1352a92288a9b9e224d15faf00

  [..] The FLASH main features are:
      (+) Flash memory read operations
      (+) Flash memory program/erase operations
      (+) Read / write protections
      (+) Prefetch on I-Code
      (+) 64 cache lines of 128 bits on I-Code
      (+) 8 cache lines of 128 bits on D-Code
<<<<<<< HEAD


                     ##### How to use this driver #####
  ==============================================================================
    [..]
      This driver provides functions and macros to configure and program the FLASH
      memory of all STM32F4xx devices.

      (#) FLASH Memory IO Programming functions:
           (++) Lock and Unlock the FLASH interface using HAL_FLASH_Unlock() and
=======
      
      
                     ##### How to use this driver #####
  ==============================================================================
    [..]                             
      This driver provides functions and macros to configure and program the FLASH 
      memory of all STM32F4xx devices.
    
      (#) FLASH Memory IO Programming functions: 
           (++) Lock and Unlock the FLASH interface using HAL_FLASH_Unlock() and 
>>>>>>> 4ae2208c09bd3a1352a92288a9b9e224d15faf00
                HAL_FLASH_Lock() functions
           (++) Program functions: byte, half word, word and double word
           (++) There Two modes of programming :
            (+++) Polling mode using HAL_FLASH_Program() function
            (+++) Interrupt mode using HAL_FLASH_Program_IT() function
<<<<<<< HEAD

      (#) Interrupts and flags management functions :
           (++) Handle FLASH interrupts by calling HAL_FLASH_IRQHandler()
           (++) Wait for last FLASH operation according to its status
           (++) Get error flag status by calling HAL_SetErrorCode()

    [..]
=======
    
      (#) Interrupts and flags management functions : 
           (++) Handle FLASH interrupts by calling HAL_FLASH_IRQHandler()
           (++) Wait for last FLASH operation according to its status
           (++) Get error flag status by calling HAL_SetErrorCode()          

    [..] 
>>>>>>> 4ae2208c09bd3a1352a92288a9b9e224d15faf00
      In addition to these functions, this driver includes a set of macros allowing
      to handle the following operations:
       (+) Set the latency
       (+) Enable/Disable the prefetch buffer
       (+) Enable/Disable the Instruction cache and the Data cache
       (+) Reset the Instruction cache and the Data cache
       (+) Enable/Disable the FLASH interrupts
       (+) Monitor the FLASH flags status
<<<<<<< HEAD

=======
          
>>>>>>> 4ae2208c09bd3a1352a92288a9b9e224d15faf00
  @endverbatim
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2017 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file in
  * the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  ******************************************************************************
<<<<<<< HEAD
  */
=======
  */ 
>>>>>>> 4ae2208c09bd3a1352a92288a9b9e224d15faf00

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/** @addtogroup STM32F4xx_HAL_Driver
  * @{
  */

/** @defgroup FLASH FLASH
  * @brief FLASH HAL module driver
  * @{
  */

#ifdef HAL_FLASH_MODULE_ENABLED

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/** @addtogroup FLASH_Private_Constants
  * @{
  */
#define FLASH_TIMEOUT_VALUE       50000U /* 50 s */
/**
  * @}
<<<<<<< HEAD
  */
=======
  */         
>>>>>>> 4ae2208c09bd3a1352a92288a9b9e224d15faf00
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/** @addtogroup FLASH_Private_Variables
  * @{
  */
/* Variable used for Erase sectors under interruption */
FLASH_ProcessTypeDef pFlash;
/**
  * @}
  */

/* Private function prototypes -----------------------------------------------*/
/** @addtogroup FLASH_Private_Functions
  * @{
  */
/* Program operations */
static void   FLASH_Program_DoubleWord(uint32_t Address, uint64_t Data);
static void   FLASH_Program_Word(uint32_t Address, uint32_t Data);
static void   FLASH_Program_HalfWord(uint32_t Address, uint16_t Data);
static void   FLASH_Program_Byte(uint32_t Address, uint8_t Data);
static void   FLASH_SetErrorCode(void);

HAL_StatusTypeDef FLASH_WaitForLastOperation(uint32_t Timeout);
/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @defgroup FLASH_Exported_Functions FLASH Exported Functions
  * @{
  */
<<<<<<< HEAD

/** @defgroup FLASH_Exported_Functions_Group1 Programming operation functions
  *  @brief   Programming operation functions
  *
@verbatim
 ===============================================================================
                  ##### Programming operation functions #####
 ===============================================================================
    [..]
    This subsection provides a set of functions allowing to manage the FLASH
=======
  
/** @defgroup FLASH_Exported_Functions_Group1 Programming operation functions 
 *  @brief   Programming operation functions 
 *
@verbatim   
 ===============================================================================
                  ##### Programming operation functions #####
 ===============================================================================  
    [..]
    This subsection provides a set of functions allowing to manage the FLASH 
>>>>>>> 4ae2208c09bd3a1352a92288a9b9e224d15faf00
    program operations.

@endverbatim
  * @{
  */

/**
  * @brief  Program byte, halfword, word or double word at a specified address
  * @param  TypeProgram  Indicate the way to program at a specified address.
  *                           This parameter can be a value of @ref FLASH_Type_Program
  * @param  Address  specifies the address to be programmed.
  * @param  Data specifies the data to be programmed
<<<<<<< HEAD
  *
=======
  * 
>>>>>>> 4ae2208c09bd3a1352a92288a9b9e224d15faf00
  * @retval HAL_StatusTypeDef HAL Status
  */
HAL_StatusTypeDef HAL_FLASH_Program(uint32_t TypeProgram, uint32_t Address, uint64_t Data)
{
  HAL_StatusTypeDef status = HAL_ERROR;
<<<<<<< HEAD

  /* Process Locked */
  __HAL_LOCK(&pFlash);

  /* Check the parameters */
  assert_param(IS_FLASH_TYPEPROGRAM(TypeProgram));

  /* Wait for last operation to be completed */
  status = FLASH_WaitForLastOperation((uint32_t)FLASH_TIMEOUT_VALUE);

  if (status == HAL_OK)
  {
    if (TypeProgram == FLASH_TYPEPROGRAM_BYTE)
=======
  
  /* Process Locked */
  __HAL_LOCK(&pFlash);
  
  /* Check the parameters */
  assert_param(IS_FLASH_TYPEPROGRAM(TypeProgram));
  
  /* Wait for last operation to be completed */
  status = FLASH_WaitForLastOperation((uint32_t)FLASH_TIMEOUT_VALUE);
  
  if(status == HAL_OK)
  {
    if(TypeProgram == FLASH_TYPEPROGRAM_BYTE)
>>>>>>> 4ae2208c09bd3a1352a92288a9b9e224d15faf00
    {
      /*Program byte (8-bit) at a specified address.*/
      FLASH_Program_Byte(Address, (uint8_t) Data);
    }
<<<<<<< HEAD
    else if (TypeProgram == FLASH_TYPEPROGRAM_HALFWORD)
=======
    else if(TypeProgram == FLASH_TYPEPROGRAM_HALFWORD)
>>>>>>> 4ae2208c09bd3a1352a92288a9b9e224d15faf00
    {
      /*Program halfword (16-bit) at a specified address.*/
      FLASH_Program_HalfWord(Address, (uint16_t) Data);
    }
<<<<<<< HEAD
    else if (TypeProgram == FLASH_TYPEPROGRAM_WORD)
=======
    else if(TypeProgram == FLASH_TYPEPROGRAM_WORD)
>>>>>>> 4ae2208c09bd3a1352a92288a9b9e224d15faf00
    {
      /*Program word (32-bit) at a specified address.*/
      FLASH_Program_Word(Address, (uint32_t) Data);
    }
    else
    {
      /*Program double word (64-bit) at a specified address.*/
      FLASH_Program_DoubleWord(Address, Data);
    }
<<<<<<< HEAD

    /* Wait for last operation to be completed */
    status = FLASH_WaitForLastOperation((uint32_t)FLASH_TIMEOUT_VALUE);

    /* If the program operation is completed, disable the PG Bit */
    FLASH->CR &= (~FLASH_CR_PG);
  }

  /* Process Unlocked */
  __HAL_UNLOCK(&pFlash);

=======
    
    /* Wait for last operation to be completed */
    status = FLASH_WaitForLastOperation((uint32_t)FLASH_TIMEOUT_VALUE);
    
    /* If the program operation is completed, disable the PG Bit */
    FLASH->CR &= (~FLASH_CR_PG);  
  }
  
  /* Process Unlocked */
  __HAL_UNLOCK(&pFlash);
  
>>>>>>> 4ae2208c09bd3a1352a92288a9b9e224d15faf00
  return status;
}

/**
  * @brief   Program byte, halfword, word or double word at a specified address  with interrupt enabled.
  * @param  TypeProgram  Indicate the way to program at a specified address.
  *                           This parameter can be a value of @ref FLASH_Type_Program
  * @param  Address  specifies the address to be programmed.
  * @param  Data specifies the data to be programmed
<<<<<<< HEAD
  *
=======
  * 
>>>>>>> 4ae2208c09bd3a1352a92288a9b9e224d15faf00
  * @retval HAL Status
  */
HAL_StatusTypeDef HAL_FLASH_Program_IT(uint32_t TypeProgram, uint32_t Address, uint64_t Data)
{
  HAL_StatusTypeDef status = HAL_OK;
<<<<<<< HEAD
=======
  
  /* Process Locked */
  __HAL_LOCK(&pFlash);
>>>>>>> 4ae2208c09bd3a1352a92288a9b9e224d15faf00

  /* Check the parameters */
  assert_param(IS_FLASH_TYPEPROGRAM(TypeProgram));

  /* Enable End of FLASH Operation interrupt */
  __HAL_FLASH_ENABLE_IT(FLASH_IT_EOP);
<<<<<<< HEAD

=======
  
>>>>>>> 4ae2208c09bd3a1352a92288a9b9e224d15faf00
  /* Enable Error source interrupt */
  __HAL_FLASH_ENABLE_IT(FLASH_IT_ERR);

  pFlash.ProcedureOnGoing = FLASH_PROC_PROGRAM;
  pFlash.Address = Address;

<<<<<<< HEAD
  if (TypeProgram == FLASH_TYPEPROGRAM_BYTE)
  {
    /*Program byte (8-bit) at a specified address.*/
    FLASH_Program_Byte(Address, (uint8_t) Data);
  }
  else if (TypeProgram == FLASH_TYPEPROGRAM_HALFWORD)
=======
  if(TypeProgram == FLASH_TYPEPROGRAM_BYTE)
  {
    /*Program byte (8-bit) at a specified address.*/
      FLASH_Program_Byte(Address, (uint8_t) Data);
  }
  else if(TypeProgram == FLASH_TYPEPROGRAM_HALFWORD)
>>>>>>> 4ae2208c09bd3a1352a92288a9b9e224d15faf00
  {
    /*Program halfword (16-bit) at a specified address.*/
    FLASH_Program_HalfWord(Address, (uint16_t) Data);
  }
<<<<<<< HEAD
  else if (TypeProgram == FLASH_TYPEPROGRAM_WORD)
=======
  else if(TypeProgram == FLASH_TYPEPROGRAM_WORD)
>>>>>>> 4ae2208c09bd3a1352a92288a9b9e224d15faf00
  {
    /*Program word (32-bit) at a specified address.*/
    FLASH_Program_Word(Address, (uint32_t) Data);
  }
  else
  {
    /*Program double word (64-bit) at a specified address.*/
    FLASH_Program_DoubleWord(Address, Data);
  }

  return status;
}

/**
  * @brief This function handles FLASH interrupt request.
  * @retval None
  */
void HAL_FLASH_IRQHandler(void)
{
  uint32_t addresstmp = 0U;
<<<<<<< HEAD

  /* Check FLASH operation error flags */
#if defined(FLASH_SR_RDERR)
  if (__HAL_FLASH_GET_FLAG((FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | \
                            FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR | FLASH_FLAG_RDERR)) != RESET)
#else
  if (__HAL_FLASH_GET_FLAG((FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | \
                            FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR)) != RESET)
#endif /* FLASH_SR_RDERR */
  {
    if (pFlash.ProcedureOnGoing == FLASH_PROC_SECTERASE)
=======
  
  /* Check FLASH operation error flags */
#if defined(FLASH_SR_RDERR) 
  if(__HAL_FLASH_GET_FLAG((FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | \
    FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR | FLASH_FLAG_RDERR)) != RESET)
#else
  if(__HAL_FLASH_GET_FLAG((FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | \
    FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR)) != RESET)
#endif /* FLASH_SR_RDERR */
  {
    if(pFlash.ProcedureOnGoing == FLASH_PROC_SECTERASE)
>>>>>>> 4ae2208c09bd3a1352a92288a9b9e224d15faf00
    {
      /*return the faulty sector*/
      addresstmp = pFlash.Sector;
      pFlash.Sector = 0xFFFFFFFFU;
    }
<<<<<<< HEAD
    else if (pFlash.ProcedureOnGoing == FLASH_PROC_MASSERASE)
=======
    else if(pFlash.ProcedureOnGoing == FLASH_PROC_MASSERASE)
>>>>>>> 4ae2208c09bd3a1352a92288a9b9e224d15faf00
    {
      /*return the faulty bank*/
      addresstmp = pFlash.Bank;
    }
    else
    {
      /*return the faulty address*/
      addresstmp = pFlash.Address;
    }
<<<<<<< HEAD

    /*Save the Error code*/
    FLASH_SetErrorCode();

    /* FLASH error interrupt user callback */
    HAL_FLASH_OperationErrorCallback(addresstmp);

    /*Stop the procedure ongoing*/
    pFlash.ProcedureOnGoing = FLASH_PROC_NONE;
  }

  /* Check FLASH End of Operation flag  */
  if (__HAL_FLASH_GET_FLAG(FLASH_FLAG_EOP) != RESET)
  {
    /* Clear FLASH End of Operation pending bit */
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP);

    if (pFlash.ProcedureOnGoing == FLASH_PROC_SECTERASE)
    {
      /*Nb of sector to erased can be decreased*/
      pFlash.NbSectorsToErase--;

      /* Check if there are still sectors to erase*/
      if (pFlash.NbSectorsToErase != 0U)
=======
    
    /*Save the Error code*/
    FLASH_SetErrorCode();
    
    /* FLASH error interrupt user callback */
    HAL_FLASH_OperationErrorCallback(addresstmp);
    
    /*Stop the procedure ongoing*/
    pFlash.ProcedureOnGoing = FLASH_PROC_NONE;
  }
  
  /* Check FLASH End of Operation flag  */
  if(__HAL_FLASH_GET_FLAG(FLASH_FLAG_EOP) != RESET)
  {
    /* Clear FLASH End of Operation pending bit */
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP);
    
    if(pFlash.ProcedureOnGoing == FLASH_PROC_SECTERASE)
    {
      /*Nb of sector to erased can be decreased*/
      pFlash.NbSectorsToErase--;
      
      /* Check if there are still sectors to erase*/
      if(pFlash.NbSectorsToErase != 0U)
>>>>>>> 4ae2208c09bd3a1352a92288a9b9e224d15faf00
      {
        addresstmp = pFlash.Sector;
        /*Indicate user which sector has been erased*/
        HAL_FLASH_EndOfOperationCallback(addresstmp);
<<<<<<< HEAD

=======
        
>>>>>>> 4ae2208c09bd3a1352a92288a9b9e224d15faf00
        /*Increment sector number*/
        pFlash.Sector++;
        addresstmp = pFlash.Sector;
        FLASH_Erase_Sector(addresstmp, pFlash.VoltageForErase);
      }
      else
      {
        /*No more sectors to Erase, user callback can be called.*/
        /*Reset Sector and stop Erase sectors procedure*/
        pFlash.Sector = addresstmp = 0xFFFFFFFFU;
        pFlash.ProcedureOnGoing = FLASH_PROC_NONE;
<<<<<<< HEAD

        /* Flush the caches to be sure of the data consistency */
        FLASH_FlushCaches();

=======
        
        /* Flush the caches to be sure of the data consistency */
        FLASH_FlushCaches() ;
                
>>>>>>> 4ae2208c09bd3a1352a92288a9b9e224d15faf00
        /* FLASH EOP interrupt user callback */
        HAL_FLASH_EndOfOperationCallback(addresstmp);
      }
    }
<<<<<<< HEAD
    else
    {
      if (pFlash.ProcedureOnGoing == FLASH_PROC_MASSERASE)
      {
        /* MassErase ended. Return the selected bank */
        /* Flush the caches to be sure of the data consistency */
        FLASH_FlushCaches();
=======
    else 
    {
      if(pFlash.ProcedureOnGoing == FLASH_PROC_MASSERASE) 
      {
        /* MassErase ended. Return the selected bank */
        /* Flush the caches to be sure of the data consistency */
        FLASH_FlushCaches() ;
>>>>>>> 4ae2208c09bd3a1352a92288a9b9e224d15faf00

        /* FLASH EOP interrupt user callback */
        HAL_FLASH_EndOfOperationCallback(pFlash.Bank);
      }
      else
      {
        /*Program ended. Return the selected address*/
        /* FLASH EOP interrupt user callback */
        HAL_FLASH_EndOfOperationCallback(pFlash.Address);
      }
      pFlash.ProcedureOnGoing = FLASH_PROC_NONE;
    }
  }
<<<<<<< HEAD

  if (pFlash.ProcedureOnGoing == FLASH_PROC_NONE)
=======
  
  if(pFlash.ProcedureOnGoing == FLASH_PROC_NONE)
>>>>>>> 4ae2208c09bd3a1352a92288a9b9e224d15faf00
  {
    /* Operation is completed, disable the PG, SER, SNB and MER Bits */
    CLEAR_BIT(FLASH->CR, (FLASH_CR_PG | FLASH_CR_SER | FLASH_CR_SNB | FLASH_MER_BIT));

    /* Disable End of FLASH Operation interrupt */
    __HAL_FLASH_DISABLE_IT(FLASH_IT_EOP);
<<<<<<< HEAD

    /* Disable Error source interrupt */
    __HAL_FLASH_DISABLE_IT(FLASH_IT_ERR);
=======
    
    /* Disable Error source interrupt */
    __HAL_FLASH_DISABLE_IT(FLASH_IT_ERR);
    
    /* Process Unlocked */
    __HAL_UNLOCK(&pFlash);
>>>>>>> 4ae2208c09bd3a1352a92288a9b9e224d15faf00
  }
}

/**
  * @brief  FLASH end of operation interrupt callback
  * @param  ReturnValue The value saved in this parameter depends on the ongoing procedure
  *                  Mass Erase: Bank number which has been requested to erase
<<<<<<< HEAD
  *                  Sectors Erase: Sector which has been erased
=======
  *                  Sectors Erase: Sector which has been erased 
>>>>>>> 4ae2208c09bd3a1352a92288a9b9e224d15faf00
  *                    (if 0xFFFFFFFFU, it means that all the selected sectors have been erased)
  *                  Program: Address which was selected for data program
  * @retval None
  */
__weak void HAL_FLASH_EndOfOperationCallback(uint32_t ReturnValue)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(ReturnValue);
  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_FLASH_EndOfOperationCallback could be implemented in the user file
<<<<<<< HEAD
   */
=======
   */ 
>>>>>>> 4ae2208c09bd3a1352a92288a9b9e224d15faf00
}

/**
  * @brief  FLASH operation error interrupt callback
  * @param  ReturnValue The value saved in this parameter depends on the ongoing procedure
  *                 Mass Erase: Bank number which has been requested to erase
  *                 Sectors Erase: Sector number which returned an error
  *                 Program: Address which was selected for data program
  * @retval None
  */
__weak void HAL_FLASH_OperationErrorCallback(uint32_t ReturnValue)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(ReturnValue);
  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_FLASH_OperationErrorCallback could be implemented in the user file
<<<<<<< HEAD
   */
=======
   */ 
>>>>>>> 4ae2208c09bd3a1352a92288a9b9e224d15faf00
}

/**
  * @}
  */

<<<<<<< HEAD
/** @defgroup FLASH_Exported_Functions_Group2 Peripheral Control functions
  *  @brief   management functions
  *
@verbatim
 ===============================================================================
                      ##### Peripheral Control functions #####
 ===============================================================================
    [..]
    This subsection provides a set of functions allowing to control the FLASH
=======
/** @defgroup FLASH_Exported_Functions_Group2 Peripheral Control functions 
 *  @brief   management functions 
 *
@verbatim   
 ===============================================================================
                      ##### Peripheral Control functions #####
 ===============================================================================  
    [..]
    This subsection provides a set of functions allowing to control the FLASH 
>>>>>>> 4ae2208c09bd3a1352a92288a9b9e224d15faf00
    memory operations.

@endverbatim
  * @{
  */

/**
  * @brief  Unlock the FLASH control register access
  * @retval HAL Status
  */
HAL_StatusTypeDef HAL_FLASH_Unlock(void)
{
  HAL_StatusTypeDef status = HAL_OK;

<<<<<<< HEAD
  if (READ_BIT(FLASH->CR, FLASH_CR_LOCK) != RESET)
=======
  if(READ_BIT(FLASH->CR, FLASH_CR_LOCK) != RESET)
>>>>>>> 4ae2208c09bd3a1352a92288a9b9e224d15faf00
  {
    /* Authorize the FLASH Registers access */
    WRITE_REG(FLASH->KEYR, FLASH_KEY1);
    WRITE_REG(FLASH->KEYR, FLASH_KEY2);

    /* Verify Flash is unlocked */
<<<<<<< HEAD
    if (READ_BIT(FLASH->CR, FLASH_CR_LOCK) != RESET)
=======
    if(READ_BIT(FLASH->CR, FLASH_CR_LOCK) != RESET)
>>>>>>> 4ae2208c09bd3a1352a92288a9b9e224d15faf00
    {
      status = HAL_ERROR;
    }
  }

  return status;
}

/**
  * @brief  Locks the FLASH control register access
  * @retval HAL Status
  */
HAL_StatusTypeDef HAL_FLASH_Lock(void)
{
  /* Set the LOCK Bit to lock the FLASH Registers access */
  FLASH->CR |= FLASH_CR_LOCK;
<<<<<<< HEAD

  return HAL_OK;
=======
  
  return HAL_OK;  
>>>>>>> 4ae2208c09bd3a1352a92288a9b9e224d15faf00
}

/**
  * @brief  Unlock the FLASH Option Control Registers access.
  * @retval HAL Status
  */
HAL_StatusTypeDef HAL_FLASH_OB_Unlock(void)
{
<<<<<<< HEAD
  if ((FLASH->OPTCR & FLASH_OPTCR_OPTLOCK) != RESET)
=======
  if((FLASH->OPTCR & FLASH_OPTCR_OPTLOCK) != RESET)
>>>>>>> 4ae2208c09bd3a1352a92288a9b9e224d15faf00
  {
    /* Authorizes the Option Byte register programming */
    FLASH->OPTKEYR = FLASH_OPT_KEY1;
    FLASH->OPTKEYR = FLASH_OPT_KEY2;
  }
  else
  {
    return HAL_ERROR;
<<<<<<< HEAD
  }

  return HAL_OK;
=======
  }  
  
  return HAL_OK;  
>>>>>>> 4ae2208c09bd3a1352a92288a9b9e224d15faf00
}

/**
  * @brief  Lock the FLASH Option Control Registers access.
<<<<<<< HEAD
  * @retval HAL Status
=======
  * @retval HAL Status 
>>>>>>> 4ae2208c09bd3a1352a92288a9b9e224d15faf00
  */
HAL_StatusTypeDef HAL_FLASH_OB_Lock(void)
{
  /* Set the OPTLOCK Bit to lock the FLASH Option Byte Registers access */
  FLASH->OPTCR |= FLASH_OPTCR_OPTLOCK;
<<<<<<< HEAD

  return HAL_OK;
=======
  
  return HAL_OK;  
>>>>>>> 4ae2208c09bd3a1352a92288a9b9e224d15faf00
}

/**
  * @brief  Launch the option byte loading.
  * @retval HAL Status
  */
HAL_StatusTypeDef HAL_FLASH_OB_Launch(void)
{
  /* Set the OPTSTRT bit in OPTCR register */
  *(__IO uint8_t *)OPTCR_BYTE0_ADDRESS |= FLASH_OPTCR_OPTSTRT;

  /* Wait for last operation to be completed */
<<<<<<< HEAD
  return (FLASH_WaitForLastOperation((uint32_t)FLASH_TIMEOUT_VALUE));
=======
  return(FLASH_WaitForLastOperation((uint32_t)FLASH_TIMEOUT_VALUE)); 
>>>>>>> 4ae2208c09bd3a1352a92288a9b9e224d15faf00
}

/**
  * @}
  */

<<<<<<< HEAD
/** @defgroup FLASH_Exported_Functions_Group3 Peripheral State and Errors functions
  *  @brief   Peripheral Errors functions
  *
@verbatim
 ===============================================================================
                ##### Peripheral Errors functions #####
 ===============================================================================
=======
/** @defgroup FLASH_Exported_Functions_Group3 Peripheral State and Errors functions 
 *  @brief   Peripheral Errors functions 
 *
@verbatim   
 ===============================================================================
                ##### Peripheral Errors functions #####
 ===============================================================================  
>>>>>>> 4ae2208c09bd3a1352a92288a9b9e224d15faf00
    [..]
    This subsection permits to get in run-time Errors of the FLASH peripheral.

@endverbatim
  * @{
  */

/**
  * @brief  Get the specific FLASH error flag.
  * @retval FLASH_ErrorCode: The returned value can be a combination of:
  *            @arg HAL_FLASH_ERROR_RD: FLASH Read Protection error flag (PCROP)
<<<<<<< HEAD
  *            @arg HAL_FLASH_ERROR_PGS: FLASH Programming Sequence error flag
  *            @arg HAL_FLASH_ERROR_PGP: FLASH Programming Parallelism error flag
  *            @arg HAL_FLASH_ERROR_PGA: FLASH Programming Alignment error flag
  *            @arg HAL_FLASH_ERROR_WRP: FLASH Write protected error flag
  *            @arg HAL_FLASH_ERROR_OPERATION: FLASH operation Error flag
  */
uint32_t HAL_FLASH_GetError(void)
{
  return pFlash.ErrorCode;
}

/**
  * @}
  */
=======
  *            @arg HAL_FLASH_ERROR_PGS: FLASH Programming Sequence error flag 
  *            @arg HAL_FLASH_ERROR_PGP: FLASH Programming Parallelism error flag  
  *            @arg HAL_FLASH_ERROR_PGA: FLASH Programming Alignment error flag
  *            @arg HAL_FLASH_ERROR_WRP: FLASH Write protected error flag
  *            @arg HAL_FLASH_ERROR_OPERATION: FLASH operation Error flag 
  */
uint32_t HAL_FLASH_GetError(void)
{ 
   return pFlash.ErrorCode;
}  
  
/**
  * @}
  */    
>>>>>>> 4ae2208c09bd3a1352a92288a9b9e224d15faf00

/**
  * @brief  Wait for a FLASH operation to complete.
  * @param  Timeout maximum flash operationtimeout
  * @retval HAL Status
  */
HAL_StatusTypeDef FLASH_WaitForLastOperation(uint32_t Timeout)
<<<<<<< HEAD
{
  uint32_t tickstart = 0U;

  /* Clear Error Code */
  pFlash.ErrorCode = HAL_FLASH_ERROR_NONE;

=======
{ 
  uint32_t tickstart = 0U;
  
  /* Clear Error Code */
  pFlash.ErrorCode = HAL_FLASH_ERROR_NONE;
  
>>>>>>> 4ae2208c09bd3a1352a92288a9b9e224d15faf00
  /* Wait for the FLASH operation to complete by polling on BUSY flag to be reset.
     Even if the FLASH operation fails, the BUSY flag will be reset and an error
     flag will be set */
  /* Get tick */
  tickstart = HAL_GetTick();

<<<<<<< HEAD
  while (__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET)
  {
    if (Timeout != HAL_MAX_DELAY)
    {
      if ((Timeout == 0U) || ((HAL_GetTick() - tickstart) > Timeout))
      {
        return HAL_TIMEOUT;
      }
    }
=======
  while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET) 
  { 
    if(Timeout != HAL_MAX_DELAY)
    {
      if((Timeout == 0U)||((HAL_GetTick() - tickstart ) > Timeout))
      {
        return HAL_TIMEOUT;
      }
    } 
>>>>>>> 4ae2208c09bd3a1352a92288a9b9e224d15faf00
  }

  /* Check FLASH End of Operation flag  */
  if (__HAL_FLASH_GET_FLAG(FLASH_FLAG_EOP) != RESET)
  {
    /* Clear FLASH End of Operation pending bit */
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP);
  }
<<<<<<< HEAD
#if defined(FLASH_SR_RDERR)
  if (__HAL_FLASH_GET_FLAG((FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | \
                            FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR | FLASH_FLAG_RDERR)) != RESET)
#else
  if (__HAL_FLASH_GET_FLAG((FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | \
                            FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR)) != RESET)
=======
#if defined(FLASH_SR_RDERR)  
  if(__HAL_FLASH_GET_FLAG((FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | \
                           FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR | FLASH_FLAG_RDERR)) != RESET)
#else
  if(__HAL_FLASH_GET_FLAG((FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | \
                           FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR)) != RESET)
>>>>>>> 4ae2208c09bd3a1352a92288a9b9e224d15faf00
#endif /* FLASH_SR_RDERR */
  {
    /*Save the error code*/
    FLASH_SetErrorCode();
    return HAL_ERROR;
  }

  /* If there is no error flag set */
  return HAL_OK;
<<<<<<< HEAD

}
=======
  
}  
>>>>>>> 4ae2208c09bd3a1352a92288a9b9e224d15faf00

/**
  * @brief  Program a double word (64-bit) at a specified address.
  * @note   This function must be used when the device voltage range is from
  *         2.7V to 3.6V and Vpp in the range 7V to 9V.
  *
<<<<<<< HEAD
  * @note   If an erase and a program operations are requested simultaneously,
  *         the erase operation is performed before the program one.
  *
=======
  * @note   If an erase and a program operations are requested simultaneously,    
  *         the erase operation is performed before the program one.
  *  
>>>>>>> 4ae2208c09bd3a1352a92288a9b9e224d15faf00
  * @param  Address specifies the address to be programmed.
  * @param  Data specifies the data to be programmed.
  * @retval None
  */
static void FLASH_Program_DoubleWord(uint32_t Address, uint64_t Data)
{
  /* Check the parameters */
  assert_param(IS_FLASH_ADDRESS(Address));
<<<<<<< HEAD

=======
  
>>>>>>> 4ae2208c09bd3a1352a92288a9b9e224d15faf00
  /* If the previous operation is completed, proceed to program the new data */
  CLEAR_BIT(FLASH->CR, FLASH_CR_PSIZE);
  FLASH->CR |= FLASH_PSIZE_DOUBLE_WORD;
  FLASH->CR |= FLASH_CR_PG;

  /* Program first word */
<<<<<<< HEAD
  *(__IO uint32_t *)Address = (uint32_t)Data;
=======
  *(__IO uint32_t*)Address = (uint32_t)Data;
>>>>>>> 4ae2208c09bd3a1352a92288a9b9e224d15faf00

  /* Barrier to ensure programming is performed in 2 steps, in right order
    (independently of compiler optimization behavior) */
  __ISB();

  /* Program second word */
<<<<<<< HEAD
  *(__IO uint32_t *)(Address + 4) = (uint32_t)(Data >> 32);
=======
  *(__IO uint32_t*)(Address+4) = (uint32_t)(Data >> 32);
>>>>>>> 4ae2208c09bd3a1352a92288a9b9e224d15faf00
}


/**
  * @brief  Program word (32-bit) at a specified address.
  * @note   This function must be used when the device voltage range is from
  *         2.7V to 3.6V.
  *
<<<<<<< HEAD
  * @note   If an erase and a program operations are requested simultaneously,
  *         the erase operation is performed before the program one.
  *
=======
  * @note   If an erase and a program operations are requested simultaneously,    
  *         the erase operation is performed before the program one.
  *  
>>>>>>> 4ae2208c09bd3a1352a92288a9b9e224d15faf00
  * @param  Address specifies the address to be programmed.
  * @param  Data specifies the data to be programmed.
  * @retval None
  */
static void FLASH_Program_Word(uint32_t Address, uint32_t Data)
{
  /* Check the parameters */
  assert_param(IS_FLASH_ADDRESS(Address));
<<<<<<< HEAD

=======
  
>>>>>>> 4ae2208c09bd3a1352a92288a9b9e224d15faf00
  /* If the previous operation is completed, proceed to program the new data */
  CLEAR_BIT(FLASH->CR, FLASH_CR_PSIZE);
  FLASH->CR |= FLASH_PSIZE_WORD;
  FLASH->CR |= FLASH_CR_PG;

<<<<<<< HEAD
  *(__IO uint32_t *)Address = Data;
=======
  *(__IO uint32_t*)Address = Data;
>>>>>>> 4ae2208c09bd3a1352a92288a9b9e224d15faf00
}

/**
  * @brief  Program a half-word (16-bit) at a specified address.
  * @note   This function must be used when the device voltage range is from
  *         2.1V to 3.6V.
  *
<<<<<<< HEAD
  * @note   If an erase and a program operations are requested simultaneously,
  *         the erase operation is performed before the program one.
  *
=======
  * @note   If an erase and a program operations are requested simultaneously,    
  *         the erase operation is performed before the program one.
  *  
>>>>>>> 4ae2208c09bd3a1352a92288a9b9e224d15faf00
  * @param  Address specifies the address to be programmed.
  * @param  Data specifies the data to be programmed.
  * @retval None
  */
static void FLASH_Program_HalfWord(uint32_t Address, uint16_t Data)
{
  /* Check the parameters */
  assert_param(IS_FLASH_ADDRESS(Address));
<<<<<<< HEAD

=======
  
>>>>>>> 4ae2208c09bd3a1352a92288a9b9e224d15faf00
  /* If the previous operation is completed, proceed to program the new data */
  CLEAR_BIT(FLASH->CR, FLASH_CR_PSIZE);
  FLASH->CR |= FLASH_PSIZE_HALF_WORD;
  FLASH->CR |= FLASH_CR_PG;

<<<<<<< HEAD
  *(__IO uint16_t *)Address = Data;
=======
  *(__IO uint16_t*)Address = Data;
>>>>>>> 4ae2208c09bd3a1352a92288a9b9e224d15faf00
}

/**
  * @brief  Program byte (8-bit) at a specified address.
  * @note   This function must be used when the device voltage range is from
  *         1.8V to 3.6V.
  *
<<<<<<< HEAD
  * @note   If an erase and a program operations are requested simultaneously,
  *         the erase operation is performed before the program one.
  *
=======
  * @note   If an erase and a program operations are requested simultaneously,    
  *         the erase operation is performed before the program one.
  *  
>>>>>>> 4ae2208c09bd3a1352a92288a9b9e224d15faf00
  * @param  Address specifies the address to be programmed.
  * @param  Data specifies the data to be programmed.
  * @retval None
  */
static void FLASH_Program_Byte(uint32_t Address, uint8_t Data)
{
  /* Check the parameters */
  assert_param(IS_FLASH_ADDRESS(Address));
<<<<<<< HEAD

=======
  
>>>>>>> 4ae2208c09bd3a1352a92288a9b9e224d15faf00
  /* If the previous operation is completed, proceed to program the new data */
  CLEAR_BIT(FLASH->CR, FLASH_CR_PSIZE);
  FLASH->CR |= FLASH_PSIZE_BYTE;
  FLASH->CR |= FLASH_CR_PG;

<<<<<<< HEAD
  *(__IO uint8_t *)Address = Data;
=======
  *(__IO uint8_t*)Address = Data;
>>>>>>> 4ae2208c09bd3a1352a92288a9b9e224d15faf00
}

/**
  * @brief  Set the specific FLASH error flag.
  * @retval None
  */
static void FLASH_SetErrorCode(void)
<<<<<<< HEAD
{
  if (__HAL_FLASH_GET_FLAG(FLASH_FLAG_WRPERR) != RESET)
  {
    pFlash.ErrorCode |= HAL_FLASH_ERROR_WRP;

    /* Clear FLASH write protection error pending bit */
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_WRPERR);
  }

  if (__HAL_FLASH_GET_FLAG(FLASH_FLAG_PGAERR) != RESET)
  {
    pFlash.ErrorCode |= HAL_FLASH_ERROR_PGA;

    /* Clear FLASH Programming alignment error pending bit */
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_PGAERR);
  }

  if (__HAL_FLASH_GET_FLAG(FLASH_FLAG_PGPERR) != RESET)
  {
    pFlash.ErrorCode |= HAL_FLASH_ERROR_PGP;

    /* Clear FLASH Programming parallelism error pending bit */
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_PGPERR);
  }

  if (__HAL_FLASH_GET_FLAG(FLASH_FLAG_PGSERR) != RESET)
  {
    pFlash.ErrorCode |= HAL_FLASH_ERROR_PGS;

    /* Clear FLASH Programming sequence error pending bit */
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_PGSERR);
  }
#if defined(FLASH_SR_RDERR)
  if (__HAL_FLASH_GET_FLAG(FLASH_FLAG_RDERR) != RESET)
  {
    pFlash.ErrorCode |= HAL_FLASH_ERROR_RD;

    /* Clear FLASH Proprietary readout protection error pending bit */
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_RDERR);
  }
#endif /* FLASH_SR_RDERR */
  if (__HAL_FLASH_GET_FLAG(FLASH_FLAG_OPERR) != RESET)
  {
    pFlash.ErrorCode |= HAL_FLASH_ERROR_OPERATION;

=======
{ 
  if(__HAL_FLASH_GET_FLAG(FLASH_FLAG_WRPERR) != RESET)
  {
   pFlash.ErrorCode |= HAL_FLASH_ERROR_WRP;
   
   /* Clear FLASH write protection error pending bit */
   __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_WRPERR);
  }
  
  if(__HAL_FLASH_GET_FLAG(FLASH_FLAG_PGAERR) != RESET)
  {
   pFlash.ErrorCode |= HAL_FLASH_ERROR_PGA;
   
   /* Clear FLASH Programming alignment error pending bit */
   __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_PGAERR);
  }
  
  if(__HAL_FLASH_GET_FLAG(FLASH_FLAG_PGPERR) != RESET)
  {
    pFlash.ErrorCode |= HAL_FLASH_ERROR_PGP;
    
    /* Clear FLASH Programming parallelism error pending bit */
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_PGPERR);
  }
  
  if(__HAL_FLASH_GET_FLAG(FLASH_FLAG_PGSERR) != RESET)
  {
    pFlash.ErrorCode |= HAL_FLASH_ERROR_PGS;
    
    /* Clear FLASH Programming sequence error pending bit */
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_PGSERR);
  }
#if defined(FLASH_SR_RDERR) 
  if(__HAL_FLASH_GET_FLAG(FLASH_FLAG_RDERR) != RESET)
  {
    pFlash.ErrorCode |= HAL_FLASH_ERROR_RD;
    
    /* Clear FLASH Proprietary readout protection error pending bit */
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_RDERR);
  }
#endif /* FLASH_SR_RDERR */  
  if(__HAL_FLASH_GET_FLAG(FLASH_FLAG_OPERR) != RESET)
  {
    pFlash.ErrorCode |= HAL_FLASH_ERROR_OPERATION;
    
>>>>>>> 4ae2208c09bd3a1352a92288a9b9e224d15faf00
    /* Clear FLASH Operation error pending bit */
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPERR);
  }
}

/**
  * @}
  */

#endif /* HAL_FLASH_MODULE_ENABLED */

/**
  * @}
  */

/**
  * @}
  */

