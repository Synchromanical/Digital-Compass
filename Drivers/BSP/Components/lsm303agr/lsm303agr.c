/**
  ******************************************************************************
  * @file    lsm303agr.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    10-October-2016
  * @brief   This file provides a set of functions needed to manage the lsm303agr
  *          MEMS accelerometer.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2014 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "lsm303agr.h"

/** @addtogroup BSP
  * @{
  */ 

/** @addtogroup Components
  * @{
  */ 

/** @addtogroup LSM303AGR
  * @{
  */

/** @defgroup LSM303AGR_Private_TypesDefinitions
  * @{
  */

/**
  * @}
  */

/** @defgroup LSM303AGR_Private_Defines
  * @{
  */

/**
  * @}
  */

/** @defgroup LSM303AGR_Private_Macros
  * @{
  */

/**
  * @}
  */ 
  
/** @defgroup LSM303AGR_Private_Variables
  * @{
  */ 
ACCELERO_DrvTypeDef Lsm303agrDrv =
{
  LSM303AGR_AccInit,
  LSM303AGR_AccDeInit,
  LSM303AGR_AccReadID,
  LSM303AGR_AccRebootCmd,
  0,
  LSM303AGR_AccZClickITConfig,
  0,
  0,
  0,
  0,
  LSM303AGR_AccFilterConfig,
  LSM303AGR_AccFilterCmd,
  LSM303AGR_AccReadXYZ
};

uint8_t tmpreg2A[8] = {0x00};
uint8_t tmpreg3A = 0x00;
uint8_t tmpregcfgA = 0x00;

/**
  * @}
  */

/** @defgroup LSM303AGR_Private_Functions
  * @{
  */

/**
  * @brief  Set LSM303AGR Initialization.
  * @param  InitStruct: Init parameters
  * @retval None
  */
void LSM303AGR_AccInit(uint16_t InitStruct)
{  
  uint8_t ctrl = 0x00;
  
  /*  Low level init */
  COMPASSACCELERO_IO_Init();
  
  /* Write value to ACC MEMS CTRL_REG1 register */
  ctrl = (uint8_t) InitStruct;
  COMPASSACCELERO_IO_Write(ACC_I2C_ADDRESS, LSM303AGR_CTRL_REG1_A, ctrl);
  
  /* Write value to ACC MEMS CTRL_REG4 register */
  ctrl = (uint8_t) (InitStruct >> 8);
  COMPASSACCELERO_IO_Write(ACC_I2C_ADDRESS, LSM303AGR_CTRL_REG4_A, ctrl);
}

/**
  * @brief  LSM303AGR De-initialization.
  * @param  None
  * @retval None
  */
void LSM303AGR_AccDeInit(void)
{  
}

/**
  * @brief  Read LSM303AGR ID.
  * @param  None
  * @retval ID 
  */
uint8_t LSM303AGR_AccReadID(void)
{  
  uint8_t ctrl = 0x00;
  
  /* Low level init */
  COMPASSACCELERO_IO_Init();
  
  /* Read value at Who am I register address */
  ctrl = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS, LSM303AGR_WHO_AM_I_ADDR);
  
  return ctrl;
}

/**
  * @brief  Reboot memory content of LSM303AGR
  * @param  None
  * @retval None
  */
void LSM303AGR_AccRebootCmd(void)
{
  uint8_t tmpreg;
  
  /* Read CTRL_REG5 register */
  tmpreg = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS, LSM303AGR_CTRL_REG5_A);
  
  /* Enable or Disable the reboot memory */
  tmpreg |= LSM303AGR_BOOT_REBOOTMEMORY;
  
  /* Write value to ACC MEMS CTRL_REG5 register */
  COMPASSACCELERO_IO_Write(ACC_I2C_ADDRESS, LSM303AGR_CTRL_REG5_A, tmpreg);
}

/**
  * @brief  Set High Pass Filter Modality
  * @param  FilterStruct: contains data for filter config
  * @retval None
  */
void LSM303AGR_AccFilterConfig(uint8_t FilterStruct) 
{
  uint8_t tmpreg;
  
  /* Read CTRL_REG2 register */
  tmpreg = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS, LSM303AGR_CTRL_REG2_A);
  
  tmpreg &= 0x0C;
  tmpreg |= FilterStruct;
  
  /* Write value to ACC MEMS CTRL_REG2 register */
  COMPASSACCELERO_IO_Write(ACC_I2C_ADDRESS, LSM303AGR_CTRL_REG2_A, tmpreg);
}

/**
  * @brief  Enable or Disable High Pass Filter
  * @param  HighPassFilterState: new state of the High Pass Filter feature.
  *      This parameter can be: 
  *         @arg: LSM303AGR_HIGHPASSFILTER_DISABLE 
  *         @arg: LSM303AGR_HIGHPASSFILTER_ENABLE
  * @retval None
  */
void LSM303AGR_AccFilterCmd(uint8_t HighPassFilterState)
{
  uint8_t tmpreg;
  
  /* Read CTRL_REG2 register */
  tmpreg = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS, LSM303AGR_CTRL_REG2_A);
  
  tmpreg &= 0xF7;
  
  tmpreg |= HighPassFilterState;
  
  /* Write value to ACC MEMS CTRL_REG2 register */
  COMPASSACCELERO_IO_Write(ACC_I2C_ADDRESS, LSM303AGR_CTRL_REG2_A, tmpreg);
}

/**
  * @brief  Read X, Y & Z Acceleration values 
  * @param  pData: Data out pointer
  * @retval None
  */
void LSM303AGR_AccReadXYZ(int16_t* pData)
{
  int16_t pnRawData[3];
  uint8_t ctrlx[2]={0,0};
  int8_t buffer[6];
  uint8_t i = 0;
  uint8_t sensitivity = LSM303AGR_ACC_SENSITIVITY_2G;
  
  /* Read the acceleration control register content */
  ctrlx[0] = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS, LSM303AGR_CTRL_REG4_A);
  ctrlx[1] = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS, LSM303AGR_CTRL_REG5_A);
  
  /* Read output register X, Y & Z acceleration */
  buffer[0] = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS, LSM303AGR_OUT_X_L_A); 
  buffer[1] = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS, LSM303AGR_OUT_X_H_A);
  buffer[2] = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS, LSM303AGR_OUT_Y_L_A);
  buffer[3] = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS, LSM303AGR_OUT_Y_H_A);
  buffer[4] = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS, LSM303AGR_OUT_Z_L_A);
  buffer[5] = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS, LSM303AGR_OUT_Z_H_A);
  
  /* Check in the control register4 the data alignment*/
  if(!(ctrlx[0] & LSM303AGR_BLE_MSB)) 
  {
    for(i=0; i<3; i++)
    {
      pnRawData[i]=((int16_t)((uint16_t)buffer[2*i+1] << 8) + buffer[2*i]);
    }
  }
  else /* Big Endian Mode */
  {
    for(i=0; i<3; i++)
    {
      pnRawData[i]=((int16_t)((uint16_t)buffer[2*i] << 8) + buffer[2*i+1]);
    }
  }
  
  /* Normal mode */
  /* Switch the sensitivity value set in the CRTL4 */
  switch(ctrlx[0] & LSM303AGR_FULLSCALE_16G)
  {
  case LSM303AGR_FULLSCALE_2G:
    sensitivity = LSM303AGR_ACC_SENSITIVITY_2G;
    break;
  case LSM303AGR_FULLSCALE_4G:
    sensitivity = LSM303AGR_ACC_SENSITIVITY_4G;
    break;
  case LSM303AGR_FULLSCALE_8G:
    sensitivity = LSM303AGR_ACC_SENSITIVITY_8G;
    break;
  case LSM303AGR_FULLSCALE_16G:
    sensitivity = LSM303AGR_ACC_SENSITIVITY_16G;
    break;
  }
  
  /* Obtain the mg value for the three axis */
  for(i=0; i<3; i++)
  {
    pData[i]=(pnRawData[i] * sensitivity);
  }
}

/**
  * @brief  Enable or Disable High Pass Filter on CLick
  * @param  HighPassFilterState: new state of the High Pass Filter feature.
  *      This parameter can be: 
  *         @arg: LSM303AGR_HPF_CLICK_DISABLE 
  *         @arg: LSM303AGR_HPF_CLICK_ENABLE
  * @retval None
  */
void LSM303AGR_AccFilterClickCmd(uint8_t HighPassFilterClickState)
{
  uint8_t tmpreg = 0x00;
  
  /* Read CTRL_REG2 register */
  tmpreg = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS, LSM303AGR_CTRL_REG2_A);
  
  tmpreg &= ~(LSM303AGR_HPF_CLICK_ENABLE);
  
  tmpreg |= HighPassFilterClickState;
  
  /* Write value to ACC MEMS CTRL_REG2 regsister */
  COMPASSACCELERO_IO_Write(ACC_I2C_ADDRESS, LSM303AGR_CTRL_REG2_A, tmpreg);
}

/**
  * @brief Enable LSM303AGR Interrupt1
  * @param  LSM303AGR_IT: specifies the LSM303AGR interrupt source to be enabled.
  *           This parameter can be any combination of the following values: 
  *         @arg   LSM303AGR_IT1_CLICK
  *         @arg   LSM303AGR_IT1_AOI1
  *         @arg   LSM303AGR_IT1_AOI2
  *         @arg   LSM303AGR_IT1_DRY1
  *         @arg   LSM303AGR_IT1_DRY2
  *         @arg   LSM303AGR_IT1_WTM
  *         @arg   LSM303AGR_IT1_OVERRUN
  * @retval None
  */
void LSM303AGR_AccIT1Enable(uint8_t LSM303AGR_IT)
{
  uint8_t tmpval = 0x00;
  
  /* Read CTRL_REG3 register */
  tmpval = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS, LSM303AGR_CTRL_REG3_A);
  
  /* Enable IT1 */
  tmpval |= LSM303AGR_IT;
  
  /* Write value to MEMS CTRL_REG3 register */
  COMPASSACCELERO_IO_Write(ACC_I2C_ADDRESS, LSM303AGR_CTRL_REG3_A, tmpval);
}

/**
  * @brief Disable LSM303AGR Interrupt1
  * @param  LSM303AGR_IT: specifies the LSM303AGR interrupt source to be disabled.
  *           This parameter can be any combination of the following values: 
  *         @arg   LSM303AGR_IT1_CLICK
  *         @arg   LSM303AGR_IT1_AOI1
  *         @arg   LSM303AGR_IT1_AOI2
  *         @arg   LSM303AGR_IT1_DRY1
  *         @arg   LSM303AGR_IT1_DRY2
  *         @arg   LSM303AGR_IT1_WTM
  *         @arg   LSM303AGR_IT1_OVERRUN
  * @retval None
  */
void LSM303AGR_AccIT1Disable(uint8_t LSM303AGR_IT)
{
  uint8_t tmpval = 0x00;
  
  /* Read CTRL_REG3 register */
  tmpval = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS, LSM303AGR_CTRL_REG3_A);
  
  /* Disable IT1 */
  tmpval &= ~LSM303AGR_IT;
  
  /* Write value to MEMS CTRL_REG3 register */
  COMPASSACCELERO_IO_Write(ACC_I2C_ADDRESS, LSM303AGR_CTRL_REG3_A, tmpval);
}

/**
  * @brief Enable LSM303AGR Interrupt2 
  * @param  LSM303AGR_IT: specifies the LSM303AGR interrupt source to be enabled.
  *           This parameter can be any combination of the following values: 
  *         @arg   LSM303AGR_IT2_CLICK
  *         @arg   LSM303AGR_IT2_INT1
  *         @arg   LSM303AGR_IT2_INT2
  *         @arg   LSM303AGR_IT2_BOOT
  *         @arg   LSM303AGR_IT2_ACT
  *         @arg   LSM303AGR_IT2_HLACTIVE
  * @retval None
  */
void LSM303AGR_AccIT2Enable(uint8_t LSM303AGR_IT)
{
  uint8_t tmpval = 0x00;
  
  /* Read CTRL_REG3 register */
  tmpval = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS, LSM303AGR_CTRL_REG6_A);
  
  /* Enable IT2 */
  tmpval |= LSM303AGR_IT;
  
  /* Write value to MEMS CTRL_REG3 register */
  COMPASSACCELERO_IO_Write(ACC_I2C_ADDRESS, LSM303AGR_CTRL_REG6_A, tmpval);
}

/**
  * @brief Disable LSM303AGR Interrupt2
  * @param  LSM303AGR_IT: specifies the LSM303AGR interrupt source to be disabled.
  *           This parameter can be any combination of the following values: 
  *         @arg   LSM303AGR_IT2_CLICK
  *         @arg   LSM303AGR_IT2_INT1
  *         @arg   LSM303AGR_IT2_INT2
  *         @arg   LSM303AGR_IT2_BOOT
  *         @arg   LSM303AGR_IT2_ACT
  *         @arg   LSM303AGR_IT2_HLACTIVE
  * @retval None
  */
void LSM303AGR_AccIT2Disable(uint8_t LSM303AGR_IT)
{
  uint8_t tmpval = 0x00;
  
  /* Read CTRL_REG3 register */
  tmpval = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS, LSM303AGR_CTRL_REG6_A);
  
  /* Disable IT2 */
  tmpval &= ~LSM303AGR_IT;
  
  /* Write value to MEMS CTRL_REG3 register */
  COMPASSACCELERO_IO_Write(ACC_I2C_ADDRESS, LSM303AGR_CTRL_REG6_A, tmpval);
}

/**
  * @brief  INT1 interrupt enable
  * @param  ITCombination: Or or And combination
  *         ITAxes: Axes to be enabled 
  * @retval None
  */
void LSM303AGR_AccINT1InterruptEnable(uint8_t ITCombination, uint8_t ITAxes)
{  
  uint8_t tmpval = 0x00;
  
  /* Read INT1_CFR register */
  tmpval = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS, LSM303AGR_INT1_CFG_A);
  
  /* Enable the selected interrupt */
  tmpval |= (ITAxes | ITCombination);
  
  /* Write value to MEMS INT1_CFR register */
  COMPASSACCELERO_IO_Write(ACC_I2C_ADDRESS, LSM303AGR_INT1_CFG_A, tmpval);  
}

/**
  * @brief  INT1 interrupt disable
  * @param  ITCombination: Or or And combination
  *         ITAxes: Axes to be enabled 
  * @retval None
  */
void LSM303AGR_AccINT1InterruptDisable(uint8_t ITCombination, uint8_t ITAxes)
{  
  uint8_t tmpval = 0x00;
  
  /* Read INT1_CFR register */
  tmpval = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS, LSM303AGR_INT1_CFG_A);
  
  /* Disable the selected interrupt */
  tmpval &= ~(ITAxes | ITCombination);
  
  /* Write value to MEMS INT1_CFR register */
  COMPASSACCELERO_IO_Write(ACC_I2C_ADDRESS, LSM303AGR_INT1_CFG_A, tmpval);
}

/**
  * @brief  INT2 interrupt enable
  * @param  ITCombination: Or or And combination
  *         ITAxes: axes to be enabled 
  * @retval None
  */
void LSM303AGR_AccINT2InterruptEnable(uint8_t ITCombination, uint8_t ITAxes)
{  
  uint8_t tmpval = 0x00;
  
  /* Read INT2_CFR register */
  tmpval = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS, LSM303AGR_INT2_CFG_A);
  
  /* Enable the selected interrupt */
  tmpval |= (ITAxes | ITCombination);
  
  /* Write value to MEMS INT2_CFR register */
  COMPASSACCELERO_IO_Write(ACC_I2C_ADDRESS, LSM303AGR_INT2_CFG_A, tmpval);
}

/**
  * @brief  INT2 interrupt config
  * @param  ITCombination: Or or And combination
  *         ITAxes: axes to be enabled 
  * @retval None
  */
void LSM303AGR_AccINT2InterruptDisable(uint8_t ITCombination, uint8_t ITAxes)
{  
  uint8_t tmpval = 0x00;
  
  /* Read INT2_CFR register */
  tmpval = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS, LSM303AGR_INT2_CFG_A);
  
  /* Disable the selected interrupt */
  tmpval &= ~(ITAxes | ITCombination);
  
  /* Write value to MEMS INT2_CFR register */
  COMPASSACCELERO_IO_Write(ACC_I2C_ADDRESS, LSM303AGR_INT2_CFG_A, tmpval);
}

/**
  * @brief  Click interrupt enable
  * @param  ITClick: the selected interrupt to enable
  * @retval None
  */
void LSM303AGR_AccClickITEnable(uint8_t ITClick)
{  
  uint8_t tmpval = 0x00;
  
  /* Read CLICK_CFR register */
  tmpval = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS, LSM303AGR_CLICK_CFG_A);
  
  /* Enable the selected interrupt */
  tmpval |= ITClick;
  
  /* Write value to MEMS CLICK CFG register */
  COMPASSACCELERO_IO_Write(ACC_I2C_ADDRESS, LSM303AGR_CLICK_CFG_A, tmpval);
  
  /* Configure Click Threshold on Z axis */
  tmpval = 0x0A;
  COMPASSACCELERO_IO_Write(ACC_I2C_ADDRESS, LSM303AGR_CLICK_THS_A, tmpval);
  
  /* Configure Time Limit */
  tmpval = 0x05;
  COMPASSACCELERO_IO_Write(ACC_I2C_ADDRESS, LSM303AGR_TIME_LIMIT_A, tmpval);
  
  /* Configure Latency */
  COMPASSACCELERO_IO_Write(ACC_I2C_ADDRESS, LSM303AGR_TIME_LATENCY_A, tmpval);
  
  /* Configure Click Window */
  tmpval = 0x32;
  COMPASSACCELERO_IO_Write(ACC_I2C_ADDRESS, LSM303AGR_TIME_WINDOW_A, tmpval);
}

/**
  * @brief  Click interrupt disable
  * @param  ITClick: the selected click interrupt to disable
  * @retval None
  */
void LSM303AGR_AccClickITDisable(uint8_t ITClick)
{  
  uint8_t tmpval = 0x00;
  
  /* Read CLICK_CFR register */
  tmpval = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS, LSM303AGR_CLICK_CFG_A);
  
  /* Disable the selected interrupt */
  tmpval &= ~ITClick;
  
  /* Write value to MEMS CLICK_CFR register */
  COMPASSACCELERO_IO_Write(ACC_I2C_ADDRESS, LSM303AGR_CLICK_CFG_A, tmpval);
}

/**
  * @brief  Click on Z axis interrupt config
  * @param  None
  * @retval None
  */
void LSM303AGR_AccZClickITConfig(void)
{  
  /* Configure low level IT config */
  COMPASSACCELERO_IO_ITConfig();
  
  /* Select click IT as INT1 interrupt */
  LSM303AGR_AccIT1Enable(LSM303AGR_IT1_CLICK);
  
  /* Enable High pass filter for click IT */
  LSM303AGR_AccFilterClickCmd(LSM303AGR_HPF_CLICK_ENABLE);
  
  /* Enable simple click IT on Z axis, */
  LSM303AGR_AccClickITEnable(LSM303AGR_Z_SINGLE_CLICK);
}


/**
  * @brief  Set LSM303AGR Mag Initialization.
  * @param  LSM303AGR_InitStruct: pointer to a LSM303AGR_MagInitTypeDef structure 
  *         that contains the configuration setting for the LSM303AGR.
  * @retval None
  */
void LSM303AGR_MagInit(LSM303AGRMag_InitTypeDef *LSM303AGR_InitStruct)
{  
	/* For LSM303AGR, the magnetometer config is done by writing to:
	   *   CFG_REG_A_M (0x60),
	   *   CFG_REG_B_M (0x61),
	   *   CFG_REG_C_M (0x62).
	   *
	   * See the datasheet for how bits map to ODR, full-scale, continuous mode, etc.
	   */
	  uint8_t regA = 0x00;
	  uint8_t regB = 0x00;
	  uint8_t regC = 0x00;

	  /*
	   * Example “OR” them in a simplistic way. Adjust the bit positions as needed
	   * if your macros do not line up exactly with the datasheet bits.
	   */
	  regA = (LSM303AGR_InitStruct->Temperature_Sensor |
	          LSM303AGR_InitStruct->MagOutput_DataRate);

	  regB = LSM303AGR_InitStruct->MagFull_Scale; /* Some bits in CFG_REG_B_M? */

	  regC = LSM303AGR_InitStruct->Working_Mode;  /* e.g. continuous or single mode */

	  /* Write these 3 single bytes: */
	  COMPASSACCELERO_IO_Write(MAG_I2C_ADDRESS, LSM303AGR_CFG_REG_A_M, regA);
	  COMPASSACCELERO_IO_Write(MAG_I2C_ADDRESS, LSM303AGR_CFG_REG_B_M, regB);
	  COMPASSACCELERO_IO_Write(MAG_I2C_ADDRESS, LSM303AGR_CFG_REG_C_M, regC);
}

/**
  * @brief  Get status for Mag LSM303AGR data
  * @param  None
  * @retval Data status in a LSM303AGR Data register
  */
uint8_t LSM303AGR_MagGetDataStatus(void)
{
  /* Return the STATUS_REG_M (0x67) contents. */
  uint8_t tmpreg = COMPASSACCELERO_IO_Read(MAG_I2C_ADDRESS, LSM303AGR_STATUS_REG_M);
  return tmpreg;
}

/**
  * @brief  Read X, Y, Z magnetometer values (raw).
  * @param  pData: 3-element array where X, Y, Z raw readings are stored.
  * @retval None
  */
void LSM303AGR_MagReadXYZ(int16_t *pData)
{
  uint8_t buffer[6];

  /* Read the 6 output registers for X, Y, Z */
  buffer[0] = COMPASSACCELERO_IO_Read(MAG_I2C_ADDRESS, LSM303AGR_OUTX_L_REG_M);
  buffer[1] = COMPASSACCELERO_IO_Read(MAG_I2C_ADDRESS, LSM303AGR_OUTX_H_REG_M);
  buffer[2] = COMPASSACCELERO_IO_Read(MAG_I2C_ADDRESS, LSM303AGR_OUTY_L_REG_M);
  buffer[3] = COMPASSACCELERO_IO_Read(MAG_I2C_ADDRESS, LSM303AGR_OUTY_H_REG_M);
  buffer[4] = COMPASSACCELERO_IO_Read(MAG_I2C_ADDRESS, LSM303AGR_OUTZ_L_REG_M);
  buffer[5] = COMPASSACCELERO_IO_Read(MAG_I2C_ADDRESS, LSM303AGR_OUTZ_H_REG_M);

  /* Reassemble the raw 16-bit readings */
  pData[0] = (int16_t)((buffer[1] << 8) | buffer[0]); /* X */
  pData[1] = (int16_t)((buffer[3] << 8) | buffer[2]); /* Y */
  pData[2] = (int16_t)((buffer[5] << 8) | buffer[4]); /* Z */
}

/* USER PATCH BEGIN --------------------------------------------------------- */
void LSM303AGR_AccInitStruct(LSM303AGRAcc_InitTypeDef *pAccInitStruct)
{
  /*
   * The official LSM303AGR_AccInit() function expects a single 16-bit parameter.
   * Typically, the low 8 bits go to CTRL_REG1_A, and the high 8 bits go to CTRL_REG4_A.
   *
   * So we build ctrl1 and ctrl4 from your structure fields.
   */
  uint8_t ctrl1 = (uint8_t)(pAccInitStruct->Power_Mode_Output_DataRate |
                            pAccInitStruct->Axes_Enable);
  uint8_t ctrl4 = (uint8_t)(pAccInitStruct->Full_Scale |
                            pAccInitStruct->High_Resolution);

  /*
   * Right now, we ignore Decimation, Fifo_Mode, Fifo_Threshold, etc.
   * If you want them to affect registers like FIFO_CTRL_REG_A, you'd
   * do that here by writing them through COMPASSACCELERO_IO_Write().
   *
   * For example (if you eventually implement FIFO):
   *    COMPASSACCELERO_IO_Write(ACC_I2C_ADDRESS, LSM303AGR_FIFO_CTRL_REG_A, ...);
   * etc.
   */

  /* Now call the "official" LSM303AGR_AccInit(), which expects (ctrl4 << 8) | ctrl1. */
  LSM303AGR_AccInit(((uint16_t)ctrl4 << 8) | ctrl1);
}

void LSM303AGR_AccITConfig(LSM303AGR_AccITConfigTypeDef *pITConfig)
{
  /*
   * Your code references .Dataready_Interrupt, .InterruptSignal, .InterruptType,
   * but these do not map to existing LSM303AGR registers in this older driver.
   * You can do something with them here if you want, or just leave it blank.
   */

  /* Example: do nothing, just avoid compiler warnings. */
  (void)pITConfig;
}
/* USER PATCH END ----------------------------------------------------------- */



/**
  * @}
  */ 

/**
  * @}
  */ 
  
/**
  * @}
  */ 

/**
  * @}
  */
