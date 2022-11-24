/**
  ******************************************************************************
  * @file    rf_driver_ll_bubble_rfip.c
  * @author  RF Application Team - EMEA
  * @brief   SPIRIT3 Bubble RFIP module APIs
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
#include "rf_driver_ll_bubble_rfip.h"
#include "rf_driver_ll_utils.h"


// ---------------> Internal (private) BUBBLE_RFIP functions

// End Of Internal (private) BUBBLE_RFIP functions <---------------

/**
* @brief  Get the IP version of the Bubble block.
* @param  None.
* @retval IP version.
*/
SBubbleVersion BubbleGetVersion(void)
{
  SBubbleVersion BubbleVersion;

  BubbleVersion.product = READ_REG_FIELD(BUBBLE->RFIP_VERSION, BUBBLE_RFIP_VERSION_PRODUCT);
  BubbleVersion.version = READ_REG_FIELD(BUBBLE->RFIP_VERSION, BUBBLE_RFIP_VERSION_VERSION);
  BubbleVersion.revision = READ_REG_FIELD(BUBBLE->RFIP_VERSION, BUBBLE_RFIP_VERSION_REVISION);

  return BubbleVersion;
}

/**
* @brief  Configure the RF IP of Bubble.
* @param  SBubbleRFConfig configuration structure.
* @retval None.
*/
void BubbleRFConfigInit(SBubbleRFConfig* pxSBubbleRFConfigStruct)
{
  assert_param(IS_EDICAL(pxSBubbleRFConfigStruct->EnergyDetectorIcal));
  assert_param(IS_AGC_REF_MODE(pxSBubbleRFConfigStruct->AgcReferenceMode));
  assert_param(IS_AGC_REF_LVL(pxSBubbleRFConfigStruct->AgcReferenceLevel));
  assert_param(IS_AGC_SPUP_LVL(pxSBubbleRFConfigStruct->AgcSpeedUpLevel));
  assert_param(IS_AGCLPF2_CAL(pxSBubbleRFConfigStruct->LPF2_Cal));
  assert_param(IS_AGC_MODE(pxSBubbleRFConfigStruct->AgcMode));
  assert_param(IS_AGC_HOLD_MODE(pxSBubbleRFConfigStruct->AgcHoldMode));

  //Set Energy Detector Calibration value
  BubbleSetEdIcal(pxSBubbleRFConfigStruct->EnergyDetectorIcal);

  //Set AGC reference mode
  BubbleSetAgcRefMode(pxSBubbleRFConfigStruct->AgcReferenceMode);

  //Set AGC reference level
  BubbleSetAgcRefLvl(pxSBubbleRFConfigStruct->AgcReferenceLevel);

  //Set AGC Speed UP Level
  BubbleSetAgcSpupLvl(pxSBubbleRFConfigStruct->AgcSpeedUpLevel);

  //Set AGC Speed UP state
  BubbleAgcSpupEnable(pxSBubbleRFConfigStruct->AgcSpeedUpEnable);

  //Configure LPF1
  BubbleSetLpf1Cal(pxSBubbleRFConfigStruct->LPF1_Cal);

  //Configure LPF2
  BubbleSetLpf2Cal(pxSBubbleRFConfigStruct->LPF2_Cal);

  //Configure Clock divider
  BubbleSetClkDiv(pxSBubbleRFConfigStruct->ClockDivider);

  //Configure Energy Detector switch
  BubbleSetEdSwitch(pxSBubbleRFConfigStruct->EnergyDetectorSwitch);

  //Configure AGC Reset Mode
  BubbleSetAgcResetMode(pxSBubbleRFConfigStruct->AgcResetMode);

  //Configure AGC Hold Mode
  BubbleSetAgcHoldMode(pxSBubbleRFConfigStruct->AgcHoldMode);

  //Configure Agc Mode
  BubbleSetAgcMode(pxSBubbleRFConfigStruct->AgcMode);
}

/**
* @brief  Get the Bubble RF IP configuration.
* @param  None.
* @retval SBubbleRFConfig configuration structure.
*/
SBubbleRFConfig BubbleGetRFConfig(void)
{
  SBubbleRFConfig SBubbleCurrentRFConfig;

  SBubbleCurrentRFConfig.EnergyDetectorIcal = BubbleGetEdIcal();
  SBubbleCurrentRFConfig.AgcReferenceMode = BubbleGetAgcRefMode();
  SBubbleCurrentRFConfig.AgcReferenceLevel = BubbleGetAgcRefLvl();
  SBubbleCurrentRFConfig.AgcSpeedUpLevel = BubbleGetAgcSpupLvl();
  SBubbleCurrentRFConfig.AgcSpeedUpEnable = BubbleAgcSpupGetState();
  SBubbleCurrentRFConfig.LPF1_Cal = BubbleGetLpf1Cal();
  SBubbleCurrentRFConfig.LPF2_Cal = BubbleGetLpf2Cal();
  SBubbleCurrentRFConfig.ClockDivider = BubbleGetClkDiv();
  SBubbleCurrentRFConfig.EnergyDetectorSwitch = BubbleGetEdSwitch();
  SBubbleCurrentRFConfig.AgcResetMode = BubbleGetAgcResetMode();
  SBubbleCurrentRFConfig.AgcHoldMode = BubbleGetAgcHoldMode();
  SBubbleCurrentRFConfig.AgcMode = BubbleGetAgcMode();

  return SBubbleCurrentRFConfig;
}

/**
* @brief  Configure the frame of Bubble RF IP.
* @param  SBubbleFrameInit configuration structure.
* @retval None.
*/
void BubbleFrameInit(SBubbleFrameInit* pxSBubbleFrameInitStruct)
{
  assert_param(IS_BUBBLE_PAYLOAD_LENGTH(pxSBubbleFrameInitStruct->PayloadLength));

  //Set the Slow Clock Cycle Per Bit counter
  BubbleSetSlowClkCyclePerBitCnt(pxSBubbleFrameInitStruct->SlowClkCyclePerBitCnt);

  //Set the Paylaod Length
  BubbleSetPayloadLength(pxSBubbleFrameInitStruct->PayloadLength);

  //Set the SYNC threshold
  BubbleSetSyncThrCnt(pxSBubbleFrameInitStruct->SyncThr);

  //Set the SYNC Length
  BubbleSetSyncLength(pxSBubbleFrameInitStruct->SyncLength);

  //Set the Preamble threshold counter
  BubbleSetPreambleThrCnt(pxSBubbleFrameInitStruct->PreambleThrCnt);

  //Set the Preamble Enable State
  BubbleSetPreambleEnable(pxSBubbleFrameInitStruct->PreambleEnable);

  //Set the SYNC counter timeout
  BubbleSetFrameSyncCntTimeout(pxSBubbleFrameInitStruct->FrameSyncCntTimeout);

  //Configure the SYNC pattern
  BubbleSetSync((uint32_t)(pxSBubbleFrameInitStruct->FrameSyncPattenHigh<<16) +\
                 (uint32_t)(pxSBubbleFrameInitStruct->FrameSyncPatternLow));

}


/**
* @brief  Get the frame configuration of Bubble RF IP.
* @param  None.
* @retval SBubbleFrameInit configuration structure.
*/
SBubbleFrameInit BubbleGetFrameInfo(void)
{
  SBubbleFrameInit SBubbleCurrentFrameInfo;

  SBubbleCurrentFrameInfo.SlowClkCyclePerBitCnt = BubbleGetSlowClkCyclePerBitCnt();
  SBubbleCurrentFrameInfo.PayloadLength = BubbleGetPayloadLength();
  SBubbleCurrentFrameInfo.SyncThr = BubbleGetSyncThrCnt();
  SBubbleCurrentFrameInfo.SyncLength = BubbleGetSyncLength();
  SBubbleCurrentFrameInfo.PreambleThrCnt = BubbleGetPreambleThrCnt();
  SBubbleCurrentFrameInfo.PreambleEnable = BubbleGetPreambleEnable();
  SBubbleCurrentFrameInfo.FrameSyncCntTimeout = BubbleGetFrameSyncCntTimeout();
  SBubbleCurrentFrameInfo.FrameSyncPattenHigh = BubbleGetSyncPatternHigh();
  SBubbleCurrentFrameInfo.FrameSyncPatternLow = BubbleGetSyncPatternLow();

  return SBubbleCurrentFrameInfo;
}


/**
* @brief  Configure the IRQ of Bubble.
* @param  BubbleIrqEnable configuration structure.
* @retval None.
*/
void BubbleIrqConfig(BubbleIrqEnable IrqEnable)
{
  assert_param(IS_IRQ_ENABLE(IrqEnable));
  if(IrqEnable == IRQ_BIT_SYNC_DETECTED)
  {
  	MODIFY_REG_FIELD(BUBBLE->IRQ_ENABLE,BUBBLE_IRQ_ENABLE_BIT_SYNC_DETECTED_E,0x01);
  	MODIFY_REG_FIELD(BUBBLE->IRQ_ENABLE,BUBBLE_IRQ_ENABLE_FRAME_SYNC_COMPLETE_E,0x00);
  	MODIFY_REG_FIELD(BUBBLE->IRQ_ENABLE,BUBBLE_IRQ_ENABLE_FRAME_COMPLETE_E,0x00);
  	MODIFY_REG_FIELD(BUBBLE->IRQ_ENABLE,BUBBLE_IRQ_ENABLE_FRAME_VALID_E,0x00);
  }
  else if(IrqEnable == IRQ_FRAME_SYNC_COMPLETE)
  {
  	MODIFY_REG_FIELD(BUBBLE->IRQ_ENABLE,BUBBLE_IRQ_ENABLE_BIT_SYNC_DETECTED_E,0x00);
  	MODIFY_REG_FIELD(BUBBLE->IRQ_ENABLE,BUBBLE_IRQ_ENABLE_FRAME_SYNC_COMPLETE_E,0x01);
  	MODIFY_REG_FIELD(BUBBLE->IRQ_ENABLE,BUBBLE_IRQ_ENABLE_FRAME_COMPLETE_E,0x00);
  	MODIFY_REG_FIELD(BUBBLE->IRQ_ENABLE,BUBBLE_IRQ_ENABLE_FRAME_VALID_E,0x00);
  }
  else if(IrqEnable == IRQ_FRAME_COMPLETE)
  {
  	MODIFY_REG_FIELD(BUBBLE->IRQ_ENABLE,BUBBLE_IRQ_ENABLE_BIT_SYNC_DETECTED_E,0x00);
  	MODIFY_REG_FIELD(BUBBLE->IRQ_ENABLE,BUBBLE_IRQ_ENABLE_FRAME_SYNC_COMPLETE_E,0x00);
  	MODIFY_REG_FIELD(BUBBLE->IRQ_ENABLE,BUBBLE_IRQ_ENABLE_FRAME_COMPLETE_E,0x01);
  	MODIFY_REG_FIELD(BUBBLE->IRQ_ENABLE,BUBBLE_IRQ_ENABLE_FRAME_VALID_E,0x00);
  }
  else if(IrqEnable == IRQ_FRAME_VALID)
  {
  	MODIFY_REG_FIELD(BUBBLE->IRQ_ENABLE,BUBBLE_IRQ_ENABLE_BIT_SYNC_DETECTED_E,0x00);
  	MODIFY_REG_FIELD(BUBBLE->IRQ_ENABLE,BUBBLE_IRQ_ENABLE_FRAME_SYNC_COMPLETE_E,0x00);
  	MODIFY_REG_FIELD(BUBBLE->IRQ_ENABLE,BUBBLE_IRQ_ENABLE_FRAME_COMPLETE_E,0x00);
  	MODIFY_REG_FIELD(BUBBLE->IRQ_ENABLE,BUBBLE_IRQ_ENABLE_FRAME_VALID_E,0x01);
  }
}


/**
* @brief  Get the RX status of Bubble.
* @param  None.
* @retval BubbleStatus configuration structure.
*/
BubbleStatus BubbleGetStatus(void)
{
  BubbleStatus status = NO_STATUS;
  if(READ_REG_FIELD(BUBBLE->STATUS,BUBBLE_STATUS_BIT_SYNC_DETECTED_F))
  	status = BIT_SYNC_DETECTED_F;
  if(READ_REG_FIELD(BUBBLE->STATUS,BUBBLE_STATUS_FRAME_SYNC_COMPLETE_F))
  	status = FRAME_SYNC_COMPLETE_F;
  if(READ_REG_FIELD(BUBBLE->STATUS,BUBBLE_STATUS_FRAME_COMPLETE_F))
  	status = FRAME_COMPLETE_F;
  if(READ_REG_FIELD(BUBBLE->STATUS,BUBBLE_STATUS_FRAME_VALID_F))
  	status = FRAME_VALID_F;

  return status;
}


/**
* @brief  Reset the RX status of Bubble.
* @param  None.
* @retval None.
*/
void BubbleClearStatus(void)
{
  MODIFY_REG_FIELD(BUBBLE->STATUS,BUBBLE_STATUS_BIT_SYNC_DETECTED_F,0x01);
  MODIFY_REG_FIELD(BUBBLE->STATUS,BUBBLE_STATUS_FRAME_SYNC_COMPLETE_F,0x01);
  MODIFY_REG_FIELD(BUBBLE->STATUS,BUBBLE_STATUS_FRAME_COMPLETE_F,0x01);
  MODIFY_REG_FIELD(BUBBLE->STATUS,BUBBLE_STATUS_FRAME_VALID_F,0x01);
}


/**
* @brief  Read the error status of Bubble.
* @param  None.
* @retval BubbleError value.
*/
BubbleError BubbleReadError(void)
{
  return (BubbleError)(READ_REG_FIELD(BUBBLE->STATUS,BUBBLE_STATUS_ERROR_F));
}


/**
* @brief  Clear the error status of Bubble.
* @param  None.
* @retval None.
*/
void BubbleClearErrorStatus(void)
{
  MODIFY_REG_FIELD(BUBBLE->STATUS, BUBBLE_STATUS_ERROR_F, 0x01);
}


/**
* @brief  Set the Slow Clock Cycle Bit Counter.
* @param  ClkCyclePerBit.
* @retval None.
*/
void BubbleSetSlowClkCyclePerBitCnt(uint8_t ClkCyclePerBit)
{
  MODIFY_REG_FIELD(BUBBLE->FRAME_CONFIG0, BUBBLE_FRAME_CONFIG0_SLOW_CLK_CYCLE_PER_BIT_CNT, ClkCyclePerBit);
}


/**
* @brief  Get the Slow Clock Cycle Bit Counter.
* @param  None.
* @retval ClkCyclePerBit.
*/
uint8_t BubbleGetSlowClkCyclePerBitCnt(void)
{
  return (uint8_t)(READ_REG_FIELD(BUBBLE->FRAME_CONFIG0,BUBBLE_FRAME_CONFIG0_SLOW_CLK_CYCLE_PER_BIT_CNT));
}


/**
* @brief  Set the Payload Length.
* @param  length.
* @retval None.
*/
void BubbleSetPayloadLength(uint8_t length)
{
  assert_param(IS_BUBBLE_PAYLOAD_LENGTH(length));
  MODIFY_REG_FIELD(BUBBLE->FRAME_CONFIG0, BUBBLE_FRAME_CONFIG0_PAYLOAD_LENGTH,length);
}


/**
* @brief  Get the payload length.
* @param  None.
* @retval Length.
*/
uint8_t BubbleGetPayloadLength(void)
{
  return (uint8_t)(READ_REG_FIELD(BUBBLE->FRAME_CONFIG0, BUBBLE_FRAME_CONFIG0_PAYLOAD_LENGTH));
}


/**
* @brief  Set SYNC threshold counter.
* @param  SyncThrCnt.
* @retval None.
*/
void BubbleSetSyncThrCnt(uint8_t SyncThrCnt)
{
  MODIFY_REG_FIELD(BUBBLE->FRAME_CONFIG0,BUBBLE_FRAME_CONFIG0_SYNC_THRESHOLD_COUNT,SyncThrCnt);
}


/**
* @brief  Get the SYNC threshold counter.
* @param  None.
* @retval Sync threshold counter.
*/
uint8_t BubbleGetSyncThrCnt(void)
{
  return (uint8_t)(READ_REG_FIELD(BUBBLE->FRAME_CONFIG0, BUBBLE_FRAME_CONFIG0_SYNC_THRESHOLD_COUNT));
}


/**
* @brief  Set the Bubble SYNC length.
* @param  SyncLen.
* @retval None.
*/
void BubbleSetSyncLength(uint8_t SyncLen)
{
  MODIFY_REG_FIELD(BUBBLE->FRAME_CONFIG0, BUBBLE_FRAME_CONFIG0_SYNC_LENGTH, SyncLen);
}


/**
* @brief  Get the Bubble SYNC length.
* @param  None.
* @retval SyncLen.
*/
uint8_t BubbleGetSyncLength(void)
{
  return (uint8_t)(READ_REG_FIELD(BUBBLE->FRAME_CONFIG0,BUBBLE_FRAME_CONFIG0_SYNC_LENGTH));
}


/**
* @brief  Set the Preamble threshold counter.
* @param  cnt : counter value.
* @retval None.
*/
void BubbleSetPreambleThrCnt(uint8_t cnt)
{
  MODIFY_REG_FIELD(BUBBLE->FRAME_CONFIG0,BUBBLE_FRAME_CONFIG0_PREAMBLE_THRESHOLD_COUNT,cnt);
}


/**
* @brief  Get the Preamble threshold counter.
* @param  None.
* @retval Counter value.
*/
uint8_t BubbleGetPreambleThrCnt(void)
{
  return (uint8_t)(READ_REG_FIELD(BUBBLE->FRAME_CONFIG0,BUBBLE_FRAME_CONFIG0_PREAMBLE_THRESHOLD_COUNT));
}


/**
* @brief  Set the preamble enable.
* @param  state.
* @retval None.
*/
void BubbleSetPreambleEnable(FunctionalState state)
{
  if(state == ENABLE)
  {
    MODIFY_REG_FIELD(BUBBLE->FRAME_CONFIG1,BUBBLE_FRAME_CONFIG1_PREAMBLE_ENABLE,0x1);
  }
  else
  {
    MODIFY_REG_FIELD(BUBBLE->FRAME_CONFIG1,BUBBLE_FRAME_CONFIG1_PREAMBLE_ENABLE,0x0);
  }
}


/**
* @brief  Get teh preamble enable state.
* @param  None.
* @retval Preamble enable state.
*/
FunctionalState BubbleGetPreambleEnable(void)
{
  if(READ_REG_FIELD(BUBBLE->FRAME_CONFIG1,BUBBLE_FRAME_CONFIG1_PREAMBLE_ENABLE))
    return ENABLE;
  else
    return DISABLE;
}


/**
* @brief  Set Frame Sync counter timeout.
* @param  timeout.
* @retval None.
*/
void BubbleSetFrameSyncCntTimeout(uint8_t timeout)
{
  MODIFY_REG_FIELD(BUBBLE->FRAME_CONFIG1,BUBBLE_FRAME_CONFIG1_FRAME_SYNC_COUNTER_TIMEOUT,timeout);
}


/**
* @brief  Get the Frame Sync counter timeout value.
* @param  None.
* @retval Counter timeout.
*/
uint8_t BubbleGetFrameSyncCntTimeout(void)
{
  return (uint8_t)(READ_REG_FIELD(BUBBLE->FRAME_CONFIG1,BUBBLE_FRAME_CONFIG1_FRAME_SYNC_COUNTER_TIMEOUT));
}


/**
* @brief  Set the Kp parameter for clock recovery.
* @param  v_kp.
* @retval None.
*/
void BubbleSetKp(uint8_t v_kp)
{
  MODIFY_REG_FIELD(BUBBLE->FRAME_CONFIG1,BUBBLE_FRAME_CONFIG1_KP,v_kp);
}


/**
* @brief  Get teh Kp parameter.
* @param  None.
* @retval Kp.
*/
uint8_t BubbleGetKp(void)
{
  return (uint8_t)(READ_REG_FIELD(BUBBLE->FRAME_CONFIG1,BUBBLE_FRAME_CONFIG1_KP));
}


/**
* @brief  Set the Ki parameter for clock recovery.
* @param  v_ki.
* @retval None.
*/
void BubbleSetKi(uint8_t v_ki)
{
  MODIFY_REG_FIELD(BUBBLE->FRAME_CONFIG1,BUBBLE_FRAME_CONFIG1_KI,v_ki);
}


/**
* @brief  Get the Ki parameter.
* @param  None.
* @retval Ki.
*/
uint8_t BubbleGetKi(void)
{
  return (uint8_t)(READ_REG_FIELD(BUBBLE->FRAME_CONFIG1,BUBBLE_FRAME_CONFIG1_KI));
}


/**
* @brief  Set the Bubble Sync word.
* @param  32 bits SYNC word.
* @retval None.
*/
void BubbleSetSync(uint32_t sync)
{
  uint16_t SyncHigh = (uint16_t)(sync>>16);
  uint16_t SyncLow = (uint16_t)(sync&0x0000FFFF);
  MODIFY_REG_FIELD(BUBBLE->FRAME_SYNC_CONFIG, BUBBLE_FRAME_SYNC_CONFIG_FRAME_SYNC_PATTERN_H, SyncHigh);
  MODIFY_REG_FIELD(BUBBLE->FRAME_SYNC_CONFIG, BUBBLE_FRAME_SYNC_CONFIG_FRAME_SYNC_PATTERN_L, SyncLow);
}


/**
* @brief  Get the Bubble SYNC word.
* @param  None.
* @retval 32 bits SYNC word (manchester coded).
*/
uint32_t BubbleGetSync(void)
{
  uint16_t SyncHigh = READ_REG_FIELD(BUBBLE->FRAME_SYNC_CONFIG,BUBBLE_FRAME_SYNC_CONFIG_FRAME_SYNC_PATTERN_H);
  uint16_t SyncLow = READ_REG_FIELD(BUBBLE->FRAME_SYNC_CONFIG,BUBBLE_FRAME_SYNC_CONFIG_FRAME_SYNC_PATTERN_L);

  return (uint32_t)(SyncHigh<<16 + SyncLow);
}


/**
* @brief  Get the High SYNC pattern.
* @param  None.
* @retval 16bits HIGH SYNC pattern (manchester encoded).
*/
uint16_t BubbleGetSyncPatternHigh(void)
{
  uint16_t SyncHigh = READ_REG_FIELD(BUBBLE->FRAME_SYNC_CONFIG,BUBBLE_FRAME_SYNC_CONFIG_FRAME_SYNC_PATTERN_H);
  return SyncHigh;
}


/**
* @brief  Get the Low SYNC pattern.
* @param  None.
* @retval 16bits LOW SYNC pattern (manchester encoded).
*/
uint16_t BubbleGetSyncPatternLow(void)
{
  uint16_t SyncLow = READ_REG_FIELD(BUBBLE->FRAME_SYNC_CONFIG,BUBBLE_FRAME_SYNC_CONFIG_FRAME_SYNC_PATTERN_L);
  return SyncLow;
}


/**
* @brief  Set the Bubble Wake Up level.
* @param  xWakeUpLvl.
* @retval None.
*/
void BubbleSetWakeUpLevel(WakeUpLevel xWakeUpLvl)
{
  assert_param(IS_WAKEUP(xWakeUpLvl));
  MODIFY_REG_FIELD(BUBBLE->RFIP_CONFIG,BUBBLE_RFIP_CONFIG_WAKEUP_LEVEL,xWakeUpLvl);
}


/**
* @brief  Get the Bubble Wake Up level.
* @param  None.
* @retval WakeUpLevel.
*/
WakeUpLevel BubbleGetWakeUpLevel(void)
{
  return (WakeUpLevel)READ_REG_FIELD(BUBBLE->RFIP_CONFIG,BUBBLE_RFIP_CONFIG_WAKEUP_LEVEL);
}


/**
* @brief  Enable or Disable the BUBBLE feature.
* @param  state.
* @retval None.
*/
void BubbleSetState(FunctionalState state)
{
  if(state == ENABLE)
  {
    SET_BIT(BUBBLE->RFIP_CONFIG,BUBBLE_RFIP_CONFIG_BUBBLE_ENABLE);
  }
  else
  {
    CLEAR_BIT(BUBBLE->RFIP_CONFIG,BUBBLE_RFIP_CONFIG_BUBBLE_ENABLE);
  }
}


/**
* @brief  Get the BUBBLE feature state.
* @param  None.
* @retval state.
*/
FunctionalState BubbleGetState(void)
{
  if(READ_REG_FIELD(BUBBLE->RFIP_CONFIG,BUBBLE_RFIP_CONFIG_BUBBLE_ENABLE))
    return ENABLE;
  else
    return DISABLE;
}


/**
* @brief  Set the Energy Detector calibration value.
* @param  calibration value.
* @retval None.
*/
void BubbleSetEdIcal(EdIcal value)
{
  assert_param(IS_EDICAL(value));
  MODIFY_REG_FIELD(BUBBLE->RF_CONFIG,BUBBLE_RF_CONFIG_ED_ICAL,value);
}


/**
* @brief  Get the ED calibration value.
* @param  None.
* @retval calibration value.
*/
EdIcal BubbleGetEdIcal(void)
{
  return (EdIcal)READ_REG_FIELD(BUBBLE->RF_CONFIG,BUBBLE_RF_CONFIG_ED_ICAL);
}


/**
* @brief  Set the AGC reference mode.
* @param  mode.
* @retval None.
*/
void BubbleSetAgcRefMode(AgcRefMode mode)
{
  assert_param(IS_AGC_REF_MODE(mode));
  MODIFY_REG_FIELD(BUBBLE->RF_CONFIG,BUBBLE_RF_CONFIG_AGC_REF_MODE,mode);
}


/**
* @brief  Get the AGC reference mode.
* @param  None.
* @retval mode.
*/
AgcRefMode BubbleGetAgcRefMode(void)
{
  return (AgcRefMode)READ_REG_FIELD(BUBBLE->RF_CONFIG,BUBBLE_RF_CONFIG_AGC_REF_MODE);
}


/**
* @brief  Set the AGC reference level.
* @param  level.
* @retval None.
*/
void BubbleSetAgcRefLvl(AgcRefLevel level)
{
  assert_param(IS_AGC_REF_LVL(level));
  MODIFY_REG_FIELD(BUBBLE->RF_CONFIG,BUBBLE_RF_CONFIG_AGC_REF_LVL,level);
}


/**
* @brief  Get the AGC reference level.
* @param  None.
* @retval level.
*/
AgcRefLevel BubbleGetAgcRefLvl(void)
{
  return (AgcRefLevel)READ_REG_FIELD(BUBBLE->RF_CONFIG,BUBBLE_RF_CONFIG_AGC_REF_LVL);
}


/**
* @brief  Set the AGC Speed Up Level (if Speed Up feature is ON).
* @param  level.
* @retval None.
*/
void BubbleSetAgcSpupLvl(AgcSpupLevel level)
{
  assert_param(IS_AGC_SPUP_LVL(level));
  MODIFY_REG_FIELD(BUBBLE->RF_CONFIG,BUBBLE_RF_CONFIG_AGC_SPUP_LVL,level);
}


/**
* @brief  Get the AGC Speed Up level
* @param  None.
* @retval level.
*/
AgcSpupLevel BubbleGetAgcSpupLvl(void)
{
  return (AgcSpupLevel)READ_REG_FIELD(BUBBLE->RF_CONFIG,BUBBLE_RF_CONFIG_AGC_SPUP_LVL);
}


/**
* @brief  Set the AGC Speed Up feature state
* @param  state.
* @retval None.
*/
void BubbleAgcSpupEnable(FunctionalState state)
{
  if(state == ENABLE)
  {
    SET_BIT(BUBBLE->RF_CONFIG,BUBBLE_RF_CONFIG_AGC_SPUP_EN);
  }
  else
  {
    CLEAR_BIT(BUBBLE->RF_CONFIG,BUBBLE_RF_CONFIG_AGC_SPUP_EN);
  }
}


/**
* @brief  Get the AGC Speed Up feature state.
* @param  None.
* @retval state.
*/
FunctionalState BubbleAgcSpupGetState(void)
{
  if(READ_REG_FIELD(BUBBLE->RF_CONFIG,BUBBLE_RF_CONFIG_AGC_SPUP_EN))
    return ENABLE;
  else
    return DISABLE;
}


/**
* @brief  Set the LPF1 configuration.
* @param  value.
* @retval None.
*/
void BubbleSetLpf1Cal(uint8_t value)
{
  MODIFY_REG_FIELD(BUBBLE->RF_CONFIG,BUBBLE_RF_CONFIG_LPF1_CAL,value);
}


/**
* @brief  Get the LPF1 configuration.
* @param  None.
* @retval value.
*/
uint8_t BubbleGetLpf1Cal(void)
{
  return (uint8_t)READ_REG_FIELD(BUBBLE->RF_CONFIG,BUBBLE_RF_CONFIG_LPF1_CAL);
}


/**
* @brief  Set the LPF2 configuration.
* @param  value.
* @retval None.
*/
void BubbleSetLpf2Cal(Lpf2Cal value)
{
  assert_param(IS_AGCLPF2_CAL(value));
  MODIFY_REG_FIELD(BUBBLE->RF_CONFIG,BUBBLE_RF_CONFIG_LPF2_CAL,value);
}


/**
* @brief  Get the LPF2 configuration.
* @param  None.
* @retval calibration configuration.
*/
Lpf2Cal BubbleGetLpf2Cal(void)
{
  return (Lpf2Cal)READ_REG_FIELD(BUBBLE->RF_CONFIG,BUBBLE_RF_CONFIG_LPF2_CAL);
}


/**
* @brief  Set the Clock divider
* @param  div.
* @retval None.
*/
void BubbleSetClkDiv(uint8_t div)
{
  MODIFY_REG_FIELD(BUBBLE->RF_CONFIG,BUBBLE_RF_CONFIG_CLKDIV,div);
}


/**
* @brief  Get the clock divider value.
* @param  None.
* @retval div.
*/
uint8_t BubbleGetClkDiv(void)
{
  return (uint8_t)READ_REG_FIELD(BUBBLE->RF_CONFIG,BUBBLE_RF_CONFIG_CLKDIV);
}


/**
* @brief  Set the ED switch.
* @param  state.
* @retval None.
*/
void BubbleSetEdSwitch(FunctionalState state)
{
  if(state == ENABLE)
  {
    SET_BIT(BUBBLE->RF_CONFIG,BUBBLE_RF_CONFIG_ED_SWITCH);
  }
  else
  {
    CLEAR_BIT(BUBBLE->RF_CONFIG,BUBBLE_RF_CONFIG_ED_SWITCH);
  }
}


/**
* @brief  Get the ED switch feature state.
* @param  None.
* @retval state.
*/
FunctionalState BubbleGetEdSwitch(void)
{
  if(READ_REG_FIELD(BUBBLE->RF_CONFIG,BUBBLE_RF_CONFIG_ED_SWITCH))
    return ENABLE;
  else
    return DISABLE;
}


/**
* @brief  Set the AGC reset Mode.
* @param  state.
* @retval None.
*/
void BubbleSetAgcResetMode(AgcResetMode mode)
{
  if(mode == AGC_RESET_MODE_NEVER)
  {
    SET_BIT(BUBBLE->AGC_CONFIG,BUBBLE_AGC_CONFIG_AGC_RESET_MODE);
  }
  else
  {
    CLEAR_BIT(BUBBLE->AGC_CONFIG,BUBBLE_AGC_CONFIG_AGC_RESET_MODE);
  }
}


/**
* @brief  Get the AGC reset mode.
* @param  None.
* @retval state.
*/
AgcResetMode BubbleGetAgcResetMode(void)
{
  if(READ_REG_FIELD(BUBBLE->AGC_CONFIG,BUBBLE_AGC_CONFIG_AGC_RESET_MODE))
    return AGC_RESET_MODE_NEVER;
  else
    return AGC_RESET_MODE_AFTER_FRAME;
}


/**
* @brief  Set the AGC Hold mode.
* @param  mode.
* @retval None.
*/
void BubbleSetAgcHoldMode(AgcHoldType mode)
{
  assert_param(IS_AGC_HOLD_MODE(mode));
  MODIFY_REG_FIELD(BUBBLE->AGC_CONFIG,BUBBLE_AGC_CONFIG_AGC_HOLD_MODE,mode);
}


/**
* @brief  Get the AGC Hold mode.
* @param  None.
* @retval hold mode.
*/
AgcHoldType BubbleGetAgcHoldMode(void)
{
  return (AgcHoldType)READ_REG_FIELD(BUBBLE->AGC_CONFIG,BUBBLE_AGC_CONFIG_AGC_HOLD_MODE);
}


/**
* @brief  Set the AGC mode.
* @param  mode.
* @retval None.
*/
void BubbleSetAgcMode(AgcMode mode)
{
  assert_param(IS_AGC_MODE(mode));
  MODIFY_REG_FIELD(BUBBLE->AGC_CONFIG,BUBBLE_AGC_CONFIG_AGC_MODE,mode);
}


/**
* @brief  Get the AGC mode.
* @param  None.
* @retval mode.
*/
AgcMode BubbleGetAgcMode(void)
{
  return (AgcMode)READ_REG_FIELD(BUBBLE->AGC_CONFIG,BUBBLE_AGC_CONFIG_AGC_MODE);
}


/**
* @brief  Set the payload.
* @param  buffer and length.
* @retval None.
*/
void BubbleGetPayload(uint8_t* buff)
{
  uint32_t payload0 = READ_REG_FIELD(BUBBLE->PAYLOAD_0,BUBBLE_PAYLOAD_0_PAYLOAD_0);
  uint32_t payload1 = READ_REG_FIELD(BUBBLE->PAYLOAD_1,BUBBLE_PAYLOAD_1_PAYLOAD_1);
  uint8_t length = BubbleGetPayloadLength();

  for( uint8_t i=0; i<4 ; i++)
  {
    buff[i] = ((payload0>>(i*8)) & 0xFF);
  }

  if(length > 4)
  {
    for(uint8_t j=0; j<length-4; j++)
    {
	buff[j+4] = ((payload1>>(j*8)) & 0xFF);
    }
  }
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
