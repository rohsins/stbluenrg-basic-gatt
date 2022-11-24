/**
  ******************************************************************************
  * @file    rf_driver_ll_packet_subghz.c
  * @author  RF Application Team
  * @brief   SPIRIT3 MRSubG packet handler APIs
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

#include "rf_driver_ll_packet_subghz.h"
#include "rf_driver_ll_radio_subghz.h"
#include "rf_driver_ll_utils.h"

/**
 * @defgroup PktWMbus_Private_Variables                  Pkt WMBUS Private Variables
 * @{
 */
static WMbusSubmode s_cWMbusSubmode = WMBUS_SUBMODE_NOT_CONFIGURED;


/**
 * @brief  Set the PREAMBLE field Length.
 * @param  cPreambleLength length of PREAMBLE field in pairs of bits (0 to 2046 bits).
 * @retval None.
 */
void MRSubG_SetPreambleLength(uint16_t cPreambleLength){
  assert_param(IS_PREAMBLE_LEN(cPreambleLength));

  MODIFY_REG_FIELD(MR_SUBG_GLOB_STATIC->PCKT_CONFIG, MR_SUBG_GLOB_STATIC_PCKT_CONFIG_PREAMBLE_LENGTH, cPreambleLength);
}

/**
 * @brief  Return the PREAMBLE field Length.
 * @param  None.
 * @retval uint16_t Preamble field length in in pairs of bits (0 to 2046 bits).
 */
uint16_t MRSubG_GetPreambleLength(void){
  return READ_REG_FIELD(MR_SUBG_GLOB_STATIC->PCKT_CONFIG, MR_SUBG_GLOB_STATIC_PCKT_CONFIG_PREAMBLE_LENGTH);
}

/**
 * @brief  Set the PREAMBLE pattern to be applied.
 * @param  cPreambleSeq the pattern to apply
 * @retval None.
 */
void MRSubG_SetPreambleSeq(MRSubG_PreambleSeq cPreambleSeq){
  MODIFY_REG_FIELD(MR_SUBG_GLOB_STATIC->PCKT_CONFIG, MR_SUBG_GLOB_STATIC_PCKT_CONFIG_PREAMBLE_SEQ, (uint8_t)cPreambleSeq);
}

/**
 * @brief  Return the PREAMBLE pattern.
 * @param  None.
 * @retval MRSubG_PreambleSeq Preamble pattern
 */
MRSubG_PreambleSeq MRSubG_GetPreambleSeq(void){
  return (MRSubG_PreambleSeq)(READ_REG_FIELD(MR_SUBG_GLOB_STATIC->PCKT_CONFIG, MR_SUBG_GLOB_STATIC_PCKT_CONFIG_PREAMBLE_SEQ));
}

/**
 * @brief  Set the POSTAMBLE field Length.
 * @param  cPostambleLength length of POSTAMBLE field in pairs of bits (0 to 126 bits).
 * @retval None.
 */
void MRSubG_SetPostambleLength(uint16_t cPostambleLength){
  assert_param(IS_POSTAMBLE_LEN(cPostambleLength));

  MODIFY_REG_FIELD(MR_SUBG_GLOB_STATIC->PCKT_CONFIG, MR_SUBG_GLOB_STATIC_PCKT_CONFIG_POSTAMBLE_LENGTH, cPostambleLength);
}

/**
 * @brief  Return the POSTAMBLE field Length.
 * @param  None.
 * @retval uint16_t Postamble field length in in pairs of bits (0 to 126 bits).
 */
uint16_t MRSubG_GetPostambleLength(void){
  return READ_REG_FIELD(MR_SUBG_GLOB_STATIC->PCKT_CONFIG, MR_SUBG_GLOB_STATIC_PCKT_CONFIG_POSTAMBLE_LENGTH);
}

/**
 * @brief  Set the POSTAMBLE pattern to be applied.
 * @param  cPostambleSeq the pattern to apply
 * @retval None.
 */
void MRSubG_SetPostamblSeq(MRSubG_PostambleSeq cPostambleSeq){
  MODIFY_REG_FIELD(MR_SUBG_GLOB_STATIC->PCKT_CONFIG, MR_SUBG_GLOB_STATIC_PCKT_CONFIG_POSTAMBLE_SEQ, (uint8_t)cPostambleSeq);
}

/**
 * @brief  Return the POSTAMBLE pattern.
 * @param  None.
 * @retval MRSubG_PostambleSeq Postamble pattern
 */
MRSubG_PostambleSeq MRSubG_GetPostambleSeq(void){
  return (MRSubG_PostambleSeq)(READ_REG_FIELD(MR_SUBG_GLOB_STATIC->PCKT_CONFIG, MR_SUBG_GLOB_STATIC_PCKT_CONFIG_POSTAMBLE_SEQ));
}

/**
 * @brief  Set the SYNC_PRESENT bit.
 * @param  cSyncPresent the enable/disable value for the bit
 * @retval None.
 */
void MRSubG_SetSyncPresent(FunctionalState cSyncPresent){
  MODIFY_REG_FIELD(MR_SUBG_GLOB_STATIC->PCKT_CONFIG, MR_SUBG_GLOB_STATIC_PCKT_CONFIG_SYNC_PRESENT, cSyncPresent);
}

/**
 * @brief  Get the SYNC_PRESENT bit.
 * @param  None.
 * @retval The enable/disable value for the bit.
 */
FunctionalState MRSubG_GetSyncPresent(void){
  return (FunctionalState)(READ_REG_FIELD(MR_SUBG_GLOB_STATIC->PCKT_CONFIG, MR_SUBG_GLOB_STATIC_PCKT_CONFIG_SYNC_PRESENT));
}

/**
 * @brief  Set the SECONDARY_SYNC_SEL bit.
	     In TX mode: this bit selects which synchro word is sent on the frame between SYNC and SEC_SYNC
   	     In RX mode: it enables the detection of SEC_SYNC in parallel of SYNC word.
 * @param  cSecondarySync the enable/disable value for the bit
 * @retval None.
 */
void MRSubG_SetSecondarySync(FunctionalState cSecondarySync){
  MODIFY_REG_FIELD(MR_SUBG_GLOB_STATIC->PCKT_CONFIG, MR_SUBG_GLOB_STATIC_PCKT_CONFIG_SECONDARY_SYNC_SEL, cSecondarySync);
}

/**
 * @brief  Get the SECONDARY_SYNC_SEL bit.
 * @param  None.
 * @retval The enable/disable value for the bit.
 */
FunctionalState MRSubG_GetSecondarySync(void){
  return (FunctionalState)(READ_REG_FIELD(MR_SUBG_GLOB_STATIC->PCKT_CONFIG, MR_SUBG_GLOB_STATIC_PCKT_CONFIG_SECONDARY_SYNC_SEL));
}

/**
 * @brief  Set the SYNC field Length.
 * @param  cSyncLength length of SYNC field in bits.
 * @retval None.
 */
void MRSubG_SetSyncLength(uint8_t cSyncLength){
  assert_param(IS_SYNC_LEN(cSyncLength));

  MODIFY_REG_FIELD(MR_SUBG_GLOB_STATIC->PCKT_CONFIG, MR_SUBG_GLOB_STATIC_PCKT_CONFIG_SYNC_LEN, cSyncLength);
}

/**
 * @brief  Return the SYNC field Length.
 * @param  None.
 * @retval uint8_t Sync field length in bits.
 */
uint8_t MRSubG_GetSyncLength(void){
  return READ_REG_FIELD(MR_SUBG_GLOB_STATIC->PCKT_CONFIG, MR_SUBG_GLOB_STATIC_PCKT_CONFIG_SYNC_LEN);
}

/**
* @brief  Set the SYNC word.
* @param  lSyncWord SYNC word given as a 32 bits aligned word.
* @retval None.
*/
void MRSubG_SetSyncWord(uint32_t lSyncWord){
  WRITE_REG(MR_SUBG_GLOB_STATIC->SYNC, lSyncWord);
}

/**
* @brief  Get the SYNC word.
* @param  None
* @retval The Sync word.
*/
uint32_t MRSubG_GetSyncWord(void){
   return READ_REG(MR_SUBG_GLOB_STATIC->SYNC);
}


/**
* @brief  Set the SEC_SYNC word.
* @param  lSecSyncWord SEC_SYNC word given as a 32 bits aligned word.
* @retval None.
*/
void MRSubG_SetSecondarySyncWord(uint32_t lSecSyncWord){
  WRITE_REG(MR_SUBG_GLOB_STATIC->SEC_SYNC, lSecSyncWord);
}

/**
* @brief  Get the SEC_SYNC word.
* @param  None
* @retval The Secondary Sync word.
*/
uint32_t MRSubG_GetSecondarySyncWord(void){
   return READ_REG(MR_SUBG_GLOB_STATIC->SEC_SYNC);
}

/**
 * @brief  Enable or Disable WHITENING for SPIRIT3 packets.
 * @param  xNewState new state for WHITENING mode.
 *         This parameter can be S_ENABLE or S_DISABLE.
 * @retval None.
 */
void MRSubG_PacketHandlerWhitening(FunctionalState xNewState){
  assert_param(IS_FUNCTIONAL_STATE(xNewState));

  if(xNewState == ENABLE)
    SET_BIT(MR_SUBG_GLOB_STATIC->PCKT_CTRL, MR_SUBG_GLOB_STATIC_PCKT_CTRL_WHIT_EN);
  else
    CLEAR_BIT(MR_SUBG_GLOB_STATIC->PCKT_CTRL, MR_SUBG_GLOB_STATIC_PCKT_CTRL_WHIT_EN);
}

/**
 * @brief  Enable or Disable BYTE_SWAP for SPIRIT3 packets.
 * @param  xNewState new state for BYTE_SWAP mode.
 *         This parameter can be S_ENABLE or S_DISABLE.
 * @retval None.
 */
void MRSubG_PacketHandlerByteSwap(FunctionalState xNewState){
  assert_param(IS_FUNCTIONAL_STATE(xNewState));

  if(xNewState == ENABLE)
    SET_BIT(MR_SUBG_GLOB_STATIC->PCKT_CTRL, MR_SUBG_GLOB_STATIC_PCKT_CTRL_BYTE_SWAP);
  else
    CLEAR_BIT(MR_SUBG_GLOB_STATIC->PCKT_CTRL, MR_SUBG_GLOB_STATIC_PCKT_CTRL_BYTE_SWAP);
}

/**
 * @brief  Set the Fec and Whitening order
 * @param  xWhitFecOrder new Whitening/FEC order.
 * @retval None.
 */
void MRSubG_PacketHandlerWhiteningFecOrder(MRSubG_WhiteningBeforeFECType xWhitFecOrder){
  assert_param(IS_WHITEFEC_SELECTION(xWhitFecOrder));

  if(xWhitFecOrder == WHITENING_THEN_FEC)
    SET_BIT(MR_SUBG_GLOB_STATIC->PCKT_CTRL, MR_SUBG_GLOB_STATIC_PCKT_CTRL_WHIT_BF_FEC);
  else
    CLEAR_BIT(MR_SUBG_GLOB_STATIC->PCKT_CTRL, MR_SUBG_GLOB_STATIC_PCKT_CTRL_WHIT_BF_FEC);
}


/**
 * @brief  Set the Whitening initialization value
 * @param  whit_init Whitening initialization value. 
 *         In 802.15.4 standard, this initialization seed must be programmed to 0x1FF. 
 * @retval None.
 */
void MRSubG_PacketHandlerSetWhiteningInit(uint16_t whit_init){
  MODIFY_REG_FIELD(MR_SUBG_GLOB_STATIC->PCKT_CTRL, MR_SUBG_GLOB_STATIC_PCKT_CTRL_WHIT_INIT, whit_init);
}

/**
* @brief  Set the type of coding and decoding for the packets.
* @param  cCoding coding/decoding selection.
* @retval None.
*/
void MRSubG_PacketHandlerCoding(MRSubG_PcktCoding cCoding){
  MODIFY_REG_FIELD(MR_SUBG_GLOB_STATIC->PCKT_CTRL, MR_SUBG_GLOB_STATIC_PCKT_CTRL_CODING_SEL, cCoding);
}

/**
* @brief  Set the Manchester type for the packets.
* @param  manType the Manchester encoding polarity.
*	0: �0� is encoded with �01� and �1� is encoded with �10�
*	1: �0� is encoded with �10� and �1� is encoded with �01�
* @retval None.
*/
void MRSubG_PacketHandlerManchesterType(MRSubG_ManchesterPolarity manType){
  MODIFY_REG_FIELD(MR_SUBG_GLOB_STATIC->PCKT_CTRL, MR_SUBG_GLOB_STATIC_PCKT_CTRL_MANCHESTER_TYPE, manType);
}

/**
* @brief  Set the CRC mode.
* @param  xPktCrcMode This parameter can be one among the @ref PktCrcMode .
* @retval None.
*/
void MRSubG_PacketHandlerSetCrcMode(MRSubG_PcktCrcMode xPktCrcMode){
  MODIFY_REG_FIELD(MR_SUBG_GLOB_STATIC->PCKT_CONFIG, MR_SUBG_GLOB_STATIC_PCKT_CONFIG_CRC_MODE, xPktCrcMode);
}

/**
* @brief  Get the CRC mode.
* @param  None.
* @retval MRSubG_PcktCrcMode CRC mode in the @ref MRSubG_PcktCrcMode enum.
*/
MRSubG_PcktCrcMode MRSubG_PacketHandlerGetCrcMode(void){
  return (MRSubG_PcktCrcMode)(READ_REG(MR_SUBG_GLOB_STATIC->PCKT_CONFIG) & MR_SUBG_GLOB_STATIC_PCKT_CONFIG_CRC_MODE);
}

/**
 * @brief  Set the PCKTLEN field of the PCKTLEN_CONFIG register.
 * @param  nPacketLen payload length in bytes.
 * @retval None.
 */
void MRSUBG_SetPacketLength(uint16_t nPacketLen){
  MODIFY_REG_FIELD(MR_SUBG_GLOB_DYNAMIC->PCKTLEN_CONFIG, MR_SUBG_GLOB_DYNAMIC_PCKTLEN_CONFIG_PCKTLEN, nPacketLen);
}

/**
 * @brief  Get the PCKTLEN field of the PCKTLEN_CONFIG register.
 * @param  None.
 * @retval Payload length in bytes.
 */
uint16_t MRSUBG_GetPacketLength(void){
  return READ_REG(MR_SUBG_GLOB_DYNAMIC->PCKTLEN_CONFIG) & MR_SUBG_GLOB_DYNAMIC_PCKTLEN_CONFIG_PCKTLEN;
}

/**
 * @brief  Set the FIX_VAR_LEN field of the PCKT_CONFI register.
 * @param  lenType lenght mode.
 *		0: FIXED length mode (no LENGTH field added in the frame in TX and no decode in RX)
 *		1: VARIABLE length mode (LENGTH field put in the frame in TX and decoded in RX)
 * @retval None.
 */
void MRSUBG_SetFixedVariableLength(MRSubG_LengthMode lenType){
  MODIFY_REG_FIELD(MR_SUBG_GLOB_STATIC->PCKT_CONFIG, MR_SUBG_GLOB_STATIC_PCKT_CONFIG_FIX_VAR_LEN, lenType);
}

/**
 * @brief  Get the FIX_VAR_LEN field of the PCKT_CONFI register.
 * @param  None.
 * @retval The Fixed/Variable length mode.
 */
MRSubG_LengthMode MRSUBG_GetFixedVariableLength(void){
  return (MRSubG_LengthMode)(READ_REG_FIELD(MR_SUBG_GLOB_STATIC->PCKT_CONFIG, MR_SUBG_GLOB_STATIC_PCKT_CONFIG_FIX_VAR_LEN));
}

/**
 * @brief  Set the payload length for the Basic packet format.
 * @param  nPayloadLength payload length in bytes.
 * @retval None.
 */
void MRSubG_PktBasicSetPayloadLength(uint16_t nPayloadLength){
  MODIFY_REG_FIELD(MR_SUBG_GLOB_STATIC->DATABUFFER_SIZE, MR_SUBG_GLOB_STATIC_DATABUFFER_SIZE_DATABUFFER_SIZE, nPayloadLength);
  MRSUBG_SetPacketLength(nPayloadLength);
}

/**
 * @brief  Return the payload length for the Basic packet format.
 * @param  None.
 * @retval Payload length in bytes.
 */
uint16_t MRSubG_PktBasicGetPayloadLength(void){
 return READ_REG(MR_SUBG_GLOB_STATIC->DATABUFFER_SIZE) & MR_SUBG_GLOB_STATIC_DATABUFFER_SIZE_DATABUFFER_SIZE;
}

/**
 * @brief  Indicates if the LENGTH field is defined on 1 byte or 2 bytes.
 * @param  lenWidth the length field to set.
 * @retval None.
 */
void MRSubG_SetLenWidth(MRSubG_LenWidthhMode lenWidth){
  MODIFY_REG_FIELD(MR_SUBG_GLOB_STATIC->PCKT_CONFIG, MR_SUBG_GLOB_STATIC_PCKT_CONFIG_LEN_WIDTH, lenWidth);
}

/**
 * @brief  Return the LENGTH field, defined on 1 byte or 2 bytes.
 * @param  None.
 * @retval The LENGTH field.
 */
MRSubG_LenWidthhMode MRSubG_GetLenWidth(){
  return (MRSubG_LenWidthhMode)(READ_REG_FIELD(MR_SUBG_GLOB_STATIC->PCKT_CONFIG, MR_SUBG_GLOB_STATIC_PCKT_CONFIG_LEN_WIDTH));
}

/**
 * @brief  Initialize the SPIRIT3 Basic packet according to the specified parameters in the MRSubG_PcktBasicFields struct.
 * @param  pxPktBasicInit Basic packet init structure.
 *         This parameter is a pointer to @ref MRSubG_PcktBasicFields.
 * @retval None.
 */
void MRSubG_PacketBasicInit(MRSubG_PcktBasicFields* pxPktBasicInit){

  /* Check the parameters */
  assert_param(IS_PREAMBLE_LEN(pxPktBasicInit->PreambleLength));
  assert_param(IS_SYNC_LEN(pxPktBasicInit->SyncLength));
  assert_param(IS_SFUNCTIONAL_STATE(pxPktBasicInit->ExtendedPktLenField));
  assert_param(IS_SFUNCTIONAL_STATE(pxPktBasicInit->FixVarLength));
  assert_param(IS_SFUNCTIONAL_STATE(pxPktBasicInit->AddressField));
  assert_param(IS_SFUNCTIONAL_STATE(pxPktBasicInit->Fec));
  assert_param(IS_SFUNCTIONAL_STATE(pxPktBasicInit->DataWhitening));

  /* Set the Packet Format to Basic Packet */
  MODIFY_REG_FIELD(MR_SUBG_GLOB_STATIC->PCKT_CTRL, MR_SUBG_GLOB_STATIC_PCKT_CTRL_PCKT_FORMAT, PKT_BASIC);

  /* Set the SYNC */
  MRSubG_SetSyncPresent(pxPktBasicInit->SyncPresent);
  MRSubG_SetSyncLength(pxPktBasicInit->SyncLength);
  MRSubG_SetSyncWord(pxPktBasicInit->SyncWord);
  MRSubG_SetSecondarySync(DISABLE);

  /* Set the PREAMBLE */
  MRSubG_SetPreambleLength(pxPktBasicInit->PreambleLength);
  MRSubG_SetPreambleSeq(pxPktBasicInit->PreambleSequence);

  MRSubG_PacketHandlerWhitening(pxPktBasicInit->DataWhitening);
  MRSubG_PacketHandlerCoding(pxPktBasicInit->Coding);
  MRSubG_PacketHandlerSetCrcMode(pxPktBasicInit->CrcMode);

  /* Set the Fixed or Variable Packet Length mode */
  MRSUBG_SetFixedVariableLength(pxPktBasicInit->FixVarLength);

  /* Set the Packet Length width. Relevant only if FIX_VAR_LEN=1  */
  MRSubG_SetLenWidth(pxPktBasicInit->LengthWidth);

  MRSubG_SetPostambleLength(pxPktBasicInit->PostambleLength);
  MRSubG_SetPostamblSeq(pxPktBasicInit->PostambleSequence);
}

/**
 * @brief  Initialize the S2LP WMBUS packet according to the specified parameters in the PktWMbusInit struct.
 * @param  pxPktWMbusInit pointer to a PktWMbusInit structure that contains the configuration information for the specified S2LP WMBUS PACKET FORMAT.
 *         This parameter is a pointer to @ref MRSubG_WMBUS_PcktFields.
 * @retval None.
 */
void MRSubG_WMBus_PacketInit(MRSubG_WMBUS_PcktFields* pxPktWMbusInit){
  /* Check the parameters */
  assert_param(IS_WMBUS_SUBMODE(pxPktWMbusInit->xWMbusSubmode));

  /* Set the Packet Format to WMBus Packet and disable HW CRC */
  MODIFY_REG_FIELD(MR_SUBG_GLOB_STATIC->PCKT_CTRL, MR_SUBG_GLOB_STATIC_PCKT_CTRL_PCKT_FORMAT, PKT_BASIC);
  MRSubG_PacketHandlerSetCrcMode(PKT_NO_CRC);

  s_cWMbusSubmode = pxPktWMbusInit->xWMbusSubmode;

  if(s_cWMbusSubmode==WMBUS_SUBMODE_S1_S2_LONG_HEADER) {
    MRSubG_SetPreambleLength(((uint16_t)pxPktWMbusInit->PreambleLength) + WMBUS_PREAMBLE_LEN_S1S2LONGHEADER);
    
    /* Set the SYNC */
    MRSubG_SetSyncPresent(ENABLE);
    MRSubG_SetSyncLength(WMBUS_SYNC_LEN_S1S2LONGHEADER);
    MRSubG_SetSyncWord(WMBUS_SYNCWORD_S1S2LONGHEADER);    

    /* Set the Coding type */
    MRSubG_PacketHandlerCoding(CODING_MANCHESTER);

    /* Constellation map setting */
    MRSubG_SetConstellationMapping((uint8_t)2); /* See Table 42 of the ref. manual */

    MRSubG_SetPostamblSeq(POST_SEQ_1010);
  }
  else if(s_cWMbusSubmode==WMBUS_SUBMODE_S1_M_S2_T2_OTHER_TO_METER) {
    MRSubG_SetPreambleLength(((uint16_t)pxPktWMbusInit->PreambleLength) + WMBUS_PREAMBLE_LEN_S1MS2T2OTHERTOMETER);

    /* Set the SYNC */
    MRSubG_SetSyncPresent(ENABLE);
    MRSubG_SetSyncLength(WMBUS_SYNC_LEN_S1MS2T2OTHERTOMETER);
    MRSubG_SetSyncWord(WMBUS_SYNCWORD_S1MS2T2OTHERTOMETER);
    
    /* Set the Coding type */
    MRSubG_PacketHandlerCoding(CODING_MANCHESTER);

    /* Constellation map setting */
    MRSubG_SetConstellationMapping((uint8_t)2); /* See Table 42 of the ref. manual */

    MRSubG_SetPostamblSeq(POST_SEQ_1010);
  }
  else if(s_cWMbusSubmode==WMBUS_SUBMODE_T1_T2_METER_TO_OTHER) {
    MRSubG_SetPreambleLength(((uint16_t)pxPktWMbusInit->PreambleLength) + WMBUS_PREAMBLE_LEN_T1T2METERTOOTHER);

    /* Set the SYNC */
    MRSubG_SetSyncPresent(ENABLE);
    MRSubG_SetSyncLength(WMBUS_SYNC_LEN_S1MS2T2OTHERTOMETER);
    MRSubG_SetSyncWord(WMBUS_SYNCWORD_T1T2METERTOOTHER);
    
    /* Set the Coding type */
    MRSubG_PacketHandlerCoding(CODING_3o6);
  }
  else if(s_cWMbusSubmode==WMBUS_SUBMODE_R2_SHORT_HEADER) {
    MRSubG_SetPreambleLength(((uint16_t)pxPktWMbusInit->PreambleLength) + WMBUS_PREAMBLE_LEN_R2);

    /* Set the SYNC */
    MRSubG_SetSyncPresent(ENABLE);    
    MRSubG_SetSyncLength(WMBUS_SYNC_LEN_R2);
    MRSubG_SetSyncWord(WMBUS_SYNCWORD_R2);

    /* Set the Coding type */
    MRSubG_PacketHandlerCoding(CODING_MANCHESTER);

    /* Constellation map setting */
    MRSubG_SetConstellationMapping((uint8_t)2); /* See Table 42 of the ref. manual */
  }

  MRSubG_SetPostambleLength(pxPktWMbusInit->PostambleLength);
}

/**
 * @brief  Initialize the SPIRIT3 802.15.4 packet according to the specified parameters in the MRSubG_802_15_4_PcktFields struct.
 * @param  px802_15_4PktInit 802.15.4 packet init structure.
 *         This parameter is a pointer to @ref MRSubG_802_15_4_PcktFields.
 * @retval 1 in case of errors.
 */
int MRSubG_802_15_4_PacketInit(MRSubG_802_15_4_PcktFields* px802_15_4PktInit){

  /* Check the parameters */
  assert_param(IS_PREAMBLE_LEN(px802_15_4PktInit->PreambleLength));
  assert_param(IS_SYNC_LEN(px802_15_4PktInit->SyncLength));
  assert_param(IS_SFUNCTIONAL_STATE(px802_15_4PktInit->Fec));
  assert_param(IS_SFUNCTIONAL_STATE(px802_15_4PktInit->DataWhitening));

  /* Set the Packet Format to 802.15.4 Packet */
  MODIFY_REG_FIELD(MR_SUBG_GLOB_STATIC->PCKT_CTRL, MR_SUBG_GLOB_STATIC_PCKT_CTRL_PCKT_FORMAT, PKT_802_15_4);

   /* Set the SYNC */
  MRSubG_SetSyncPresent(ENABLE);
  MRSubG_SetSyncLength(px802_15_4PktInit->SyncLength);
  MRSubG_SetSyncWord(px802_15_4PktInit->SyncWord);

  /* Set the PREAMBLE */
  MRSubG_SetPreambleLength(px802_15_4PktInit->PreambleLength);
  MRSubG_SetPreambleSeq(px802_15_4PktInit->PreambleSequence);

  /* Check if CRC is correctly set */
  MRSubG_PcktCrcMode crcSet = px802_15_4PktInit->CrcMode;

  /* Supported CRC configurations are NO CRC or MODE3 or MODE5 */
  if (crcSet == PKT_NO_CRC || crcSet == PKT_CRC_MODE_16BITS_2 || crcSet == PKT_CRC_MODE_32BITS){
    /* Set the FCS bit value and CRC_INIT field according to the CRC */
    if(crcSet == PKT_CRC_MODE_16BITS_2){
	  SET_BIT(MR_SUBG_GLOB_STATIC->PCKT_CTRL, MR_SUBG_GLOB_STATIC_PCKT_CTRL_FCS_TYPE_4G);
	  MODIFY_REG(MR_SUBG_GLOB_STATIC->CRC_INIT, MR_SUBG_GLOB_STATIC_CRC_INIT_CRC_INIT_VAL, 0x00000000);
    }
    else if(crcSet == PKT_CRC_MODE_32BITS){
	  CLEAR_BIT(MR_SUBG_GLOB_STATIC->PCKT_CTRL, MR_SUBG_GLOB_STATIC_PCKT_CTRL_FCS_TYPE_4G);
	  MODIFY_REG(MR_SUBG_GLOB_STATIC->CRC_INIT, MR_SUBG_GLOB_STATIC_CRC_INIT_CRC_INIT_VAL, 0xFFFFFFFF);
    }

    MRSubG_PacketHandlerSetCrcMode(px802_15_4PktInit->CrcMode);
  }
  else
    /* CRC not supported in 802.15.4*/
    return 1;

  /* Set the FEC type */
  MODIFY_REG_FIELD(MR_SUBG_GLOB_STATIC->PCKT_CTRL, MR_SUBG_GLOB_STATIC_PCKT_CTRL_FEC_TYPE_4G, px802_15_4PktInit->FecType);

  /* Init the Whitening to 0x1FF*/
  MRSubG_PacketHandlerSetWhiteningInit(0x1FF);

  /* Set the Interleaving */
  MODIFY_REG_FIELD(MR_SUBG_GLOB_STATIC->PCKT_CTRL, MR_SUBG_GLOB_STATIC_PCKT_CTRL_INT_EN_4G, px802_15_4PktInit->Interleaving);

  if(px802_15_4PktInit->Coding != CODING_MANCHESTER && px802_15_4PktInit->Coding !=CODING_3o6)
    MRSubG_PacketHandlerCoding(px802_15_4PktInit->Coding);
  else
    /* The SW is not supposed to select Manchester nor 3 out of 6 coding for the 802.15.4 packets.
     * It is planned to work only with Basic packet format. */
    return 1;

  /* In 802.15.4 the LEN_WIDTH bit must be set */
  MRSubG_SetLenWidth(BYTES_LEN_2);

  /* In 802.15.4, this bit is considered only in transmission */
  MRSubG_PacketHandlerWhitening(px802_15_4PktInit->DataWhitening);

  /* Byte swapping MUST NOT be enabled for 802.15.4 configuration */
  MRSubG_PacketHandlerByteSwap(DISABLE);

  /* This mode is the only one supposed to be used with 802.15.4 packet format */
  MRSubG_SetTXMode(TX_NORMAL);

  /* This mode is the only one supposed to be used with 802.15.4 packet format */
  MRSubG_SetRXMode(RX_NORMAL);

  return 0;
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
