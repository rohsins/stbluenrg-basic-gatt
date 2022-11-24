#ifndef RF_DRIVER_LL_RADIO_SUBGHZ_H
#define RF_DRIVER_LL_RADIO_SUBGHZ_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "spirit3.h"
#include "system_util.h"
#include "rf_driver_hal_power_manager.h"

#ifdef  USE_FULL_ASSERT
#include "rf_driver_assert.h"
#else
#define assert_param(expr) ((void)0U)
#endif

static const uint32_t s_Channel_Filter_Bandwidth[99]=
{
  1600000,1510000,1422000,1332000,1244000,1154000,1066000, \
  976000,888000,800000,755000,711000,666000,622000,577000, \
  533000,488000,444000,400000,377000,355000,333000,311000, \
  288000,266000,244000,222000,200000,188000,178000,166000, \
  155000,144000,133000,122000,111000,100000,94400,88900,83300, \
  77800,72200,66700,61100,55600,50000,47200,44400,41600,38900, \
  36100,33300,30500,27800,25000,23600,22200,20800,19400,18100, \
  16600,15300,13900,12500,11800,11100,10400,9700,9000,8300,7600, \
  6900,6125,5910,5550,5200,4870,4500,4100,3800,3500,3125,2940, \
  2780,2600,2400,2200,2100,1900,1700
};

#define HIGH_BAND_FACTOR	4	/*!< Band select factor for high band. Factor B in the equation of the user manual */
#define LOW_BAND_FACTOR		8	/*!< Band select factor for middle band. Factor B in the equation of the user manual */

#define LOW_BAND_LOWER_LIMIT		413000000   /*!< Lower limit of the low band */
#define LOW_BAND_UPPER_LIMIT		479000000   /*!< Upper limit of the low band */
#define HIGH_BAND_LOWER_LIMIT		826000000   /*!< Lower limit of the high band */
#define HIGH_BAND_UPPER_LIMIT		958000000   /*!< Upper limit of the high band */

#define MINIMUM_DATARATE		100	/*!< Minimum datarate supported by SPIRIT3 100 bps */
#define MAXIMUM_DATARATE		300000  /*!< Maximum datarate supported by SPIRIT3 300 ksps (600 kbps) */

#define MAX_DBM		0x51
#define GAIN_RX_CHAIN   64

#define S_ABS(a) ((a)>0?(a):-(a))

#define IS_FREQUENCY_BAND(FREQUENCY) (IS_FREQUENCY_BAND_HIGH(FREQUENCY) || \
                                      IS_FREQUENCY_BAND_LOW(FREQUENCY))

#define IS_FREQUENCY_BAND_HIGH(FREQUENCY) ((FREQUENCY)>=HIGH_BAND_LOWER_LIMIT && \
                                           (FREQUENCY)<=HIGH_BAND_UPPER_LIMIT)


#define IS_FREQUENCY_BAND_LOW(FREQUENCY) ((FREQUENCY)>=LOW_BAND_LOWER_LIMIT && \
                                             (FREQUENCY)<=LOW_BAND_UPPER_LIMIT)

#define IS_MODULATION(MOD) (((MOD) == MOD_NO_MOD) || \
					((MOD) == MOD_2FSK) || \
					((MOD) == MOD_4FSK) || \
					((MOD) == MOD_2GFSK05) || \
					((MOD) == MOD_2GFSK1)  || \
					((MOD) == MOD_4GFSK05) || \
					((MOD) == MOD_4GFSK1) || \
					((MOD) == MOD_ASK) || \
					((MOD) == MOD_OOK) || \
					((MOD) == MOD_POLAR) || \
					((MOD) == MOD_CW) )


#define IS_DATARATE(DATARATE)      (DATARATE>=MINIMUM_DATARATE && DATARATE<=((uint64_t)MAXIMUM_DATARATE)

/**
  * @brief  SPIRIT3 Commands codes enumeration
  */
typedef enum
{
  CMD_NOPE =  ((uint8_t)(0x00)),                /* No action. This 'no' command can be requested at any time, whatever the on-going command or in IDLE */
  CMD_TX =  ((uint8_t)(0x01)),			/* Start a TX sequence */
  CMD_RX =  ((uint8_t)(0x02)),			/* Start a RX sequence */
  CMD_LOCKRX =  ((uint8_t)(0x03)),              /* Start a RF sequence up to PLL locked based on RX frequency */
  CMD_LOCKTX =  ((uint8_t)(0x04)),              /* Start a RF sequence up to PLL locked based on TX frequency */
  CMD_SABORT =  ((uint8_t)(0x05)),              /* Stop any on-going RX/TX/LOCKRX/LOCKTX command */
  CMD_CALIB_SAFEASK =  ((uint8_t)(0x05)),       /* Launch a PA Safe-ASK calibration to get the max safe PA code to be used */
  CMD_RELOAD_RX_TIMER =  ((uint8_t)(0x06)),     /* Reload with a new timeout and new stop conditions and restart the RX Timer */
  CMD_CALIB_AGC_AAF =  ((uint8_t)(0x0B))        /* Start needed HW features to run an AGC_ATTEN trim or an AAF cut off frequency trim sequence at SW level */
} MRSubGCmd;

/**
  * @brief  SPIRIT3 Radio FSM state enumaration
  */
typedef enum
{
  STATE_IDLE = 0x00,
  STATE_ENA_RF_REG = 0x01,
  STATE_WAIT_ACTIVE2 = 0x02,
  STATE_ACTIVE2 = 0x03,
  STATE_ENA_CURR = 0x04,
  STATE_SYNTH_SETUP = 0x05,
  STATE_CALIB_VCO = 0x06,
  STATE_LOCKRXTX = 0x07,
  STATE_LOCKONTX = 0x08,
  STATE_EN_PA = 0x09,
  STATE_TX = 0x0A,
  STATE_PA_DWN_ANA = 0x0B,
  STATE_END_TX = 0x0C,
  STATE_LOCKONRX = 0x0D,
  STATE_EN_RX = 0x0E,
  STATE_EN_LNA = 0x0F,
  STATE_RX = 0x10,
  STATE_END_RX = 0x11,
  STATE_SYNTH_PWDN = 0x12
} MRSubGFSMState;

/**
 * @brief  SPIRIT3 Modulation enumeration
 */
typedef enum {
  MOD_2FSK         = 0x00, /*!< 2-FSK modulation selected */
  MOD_4FSK         = 0x01, /*!< 4-FSK modulation selected */
  MOD_2GFSK05      = 0x12, /*!< 2GFSK modulation selected with BT = 0.5 */
  MOD_2GFSK1       = 0x02, /*!< 2GFSK modulation selected with BT = 1 */
  MOD_4GFSK05 	 = 0x13, /*!< 4GFSK modulation selected with BT = 0.5 */
  MOD_4GFSK1 	 = 0x03, /*!< 4GFSK modulation selected with BT = 1 */
  MOD_ASK      	 = 0x05, /*!< ASK modulation selected. */
  MOD_OOK      	 = 0x05, /*!< OOK modulation selected. */
  MOD_POLAR        = 0x06, /*!< Polar modulation selected. */
  MOD_CW           = 0x07, /*!< CW modulation selected */
} MRSubGModSelect;

/**
* @brief  BT Values for GFSK
*/
typedef enum {
  BT_1 = 0,
  BT_05 = 1
} MRSubG_BTSelect;

/**
* @brief  SPIRIT3 PA modes
*/
typedef enum{
  PA_LEGACY     = 0x00, /* SPIRIT 'legacy' mode */
  PA_FIR 	= 0x01, /* FIR active (to be used in ASK/OOK modulation only) */
  PA_DIRECT     = 0x02  /* Direct mode */
} MRSubG_PAMode;

/**
* @brief  SPIRIT3 PA drive modes
*/
typedef enum{
  PA_DRV_TX             = 0x01, /* up to 10dBm */
  PA_DRV_TX_HP          = 0x02, /* up to 14dBm */
  PA_DRV_TX_TX_HP       = 0x03  /* up to 20dBm */
} MRSubG_PA_DRVMode;

/**
 * @brief  SPIRIT3 Transmission modes
 */
typedef enum {
  ISI_EQ_DISABLED = 0x00,
  ISI_EQ_SP = 0x01,
  ISI_EQ_DP = 0x02
} MRSubG_ISIEQMode;


/**
 * @brief  SPIRIT3 Transmission modes
 */
typedef enum{
  TX_NORMAL 		= 0x00, /* Only payload is provided through RAM buffers Rest of the frame built from configuration registers (PREAMBLE, SYNC, CRC...) */
  TX_DIRECT_BUFFERS 	= 0x01, /* Full bit stream (including PREAMBLE, SYNC, CRC...) to be provided through RAM buffers */
  TX_DIRECT_GPIO 		= 0x02, /* Full bit stream (including PREAMBLE, SYNC, CRC...) to be provided serially through the TX DATA GPIO */
  TX_PN9 			= 0x03  /* IInternal PN9 generator send a polynomial bit stream on the antenna. */
} MRSubGTXMode;

/**
 * @brief  SPIRIT3 Reception modes
 */
typedef enum{
  RX_NORMAL 		= 0x00, /* Only payload is stored into the RAM buffers. CRC and packet length are readable in dedicated status registers */
  RX_DIRECT_BUFFERS 	= 0x01, /* Full bit stream is stored into the RAM buffers. */
  RX_DIRECT_GPIO 		= 0x02, /* Full bit stream is provided serially through the RX DATA GPIO */
  RX_IQ_SAMPLING 		= 0x03, /* Raw I/Q sampling taken at the output of the Channel filter inside the demodulator are stored in RAM. */
  RX_FREQDETEC_SAMPLING = 0x04, /* Raw data taken at the output of the frequency detector inside the demodulator (detection of the instantaneous frequency changes) are stored in RAM.*/
  RX_SOFTBIT_SAMPLING 	= 0x05  /* Raw data taken at the output of the post-filter inside the demodulator (soft bits before the 0/1 detection) are stored in RAM. */
} MRSubGRXMode;

/**
 * @brief  SPIRIT3 Sub-GHz Radio Config structure definition
 */
typedef struct {
  uint32_t          lFrequencyBase;     /*!< Specifies the base carrier frequency (in Hz) */
  MRSubGModSelect   xModulationSelect;  /*!< Specifies the modulation @ref MRSubGModSelect */
  uint32_t          lDatarate;          /*!< Specifies the datarate expressed in sps.*/
  uint32_t          lFreqDev;           /*!< Specifies the frequency deviation expressed in Hz. */
  uint32_t          lBandwidth;         /*!< Specifies the channel filter bandwidth expressed in Hz. */
} SMRSubGConfig;


/**
 * @brief  SPIRIT3 Sub-GHz radio RF version structure definition
 */
typedef struct {
  uint8_t		revision;	/*!< Revision of the RFIP (to be used for metal fixes)*/
  uint8_t		version;	/*!< Version of the RFIP (to be used for cut upgrades)*/
  uint8_t		product;	/*!< Used for major upgrades (new protocols support / new features)*/
} SMRSubGVersion;

/**
 * @brief  SPIRIT3 MR_SUBG public interface
 */
SMRSubGVersion MRSubGGetVersion(void);

uint8_t MRSubG_Init(SMRSubGConfig* pxSRadioInitStruct);
void MRSubG_StrobeCommand(MRSubGCmd xCommandCode);

void MRSubG_SetTXMode(MRSubGTXMode rxMode);
void MRSubG_SetRXMode(MRSubGRXMode rxMode);

void MRSubG_GetInfo(SMRSubGConfig* pxSRadioInitStruct);
void MRSubG_SetSynthWord(uint32_t lSynthWord);
uint32_t MRSubG_GetSynthWord(void);
void MRSubG_SetChannel(uint8_t cChannel);
uint8_t MRSubG_GetChannel(void);
void MRSubG_SetChannelSpace(uint8_t lChannelSpace);
uint8_t MRSubG_GetChannelSpace(void);
void MRSubG_SetPAMode(MRSubG_PAMode paMode);
void MRSubG_SetPADriveMode(MRSubG_PA_DRVMode paDrvMode);
void MRSubG_SetFrequencyBase(uint32_t lFBase);
uint32_t MRSubG_GetFrequencyBase(void);
void MRSubG_SetDatarate(uint32_t lDatarate);
uint32_t MRSubG_GetDatarate(void);
void MRSubG_SetFrequencyDev(uint32_t lFDev);
uint32_t MRSubG_GetFrequencyDev(void);
void MRSubG_SetChannelBW(uint32_t lBandwidth);
uint32_t MRSubG_GetChannelBW(void);
void MRSubG_SetModulation(MRSubGModSelect xModulation);
MRSubGModSelect MRSubG_GetModulation(void);
int32_t MRSubG_GetRssidBm(void);
void MRSubG_SetRSSIThreshold(int16_t rssiTh);
int32_t MRSubG_GetRSSIThreshold(void);
void MRSubG_SetConstellationMapping(uint8_t nConMap);
void MRSubG_SwapSymbolMapping(FunctionalState xNewState);
void MRSubG_SetBTSel(MRSubG_BTSelect bSel);
void MRSubG_SetFrequencyInterpolator(FunctionalState xNewState);
void MRSubG_SetISIEqualizer(MRSubG_ISIEQMode isiEq);
void MRSubG_SetMaxPALevel(FunctionalState xNewState);
void MRSubG_SetPADegen(FunctionalState xNewState);
void MRSubG_SetPALeveldBm(uint8_t cIndex, int8_t lPowerdBm, MRSubG_PA_DRVMode drvMode, uint8_t enableMaxPA);
int8_t MRSubG_GetPALeveldBm(void);
uint8_t MRSubG_GetPALevelMaxIndex(void);
void MRSubG_SetPALevelMaxIndex(uint8_t cIndex);
void MRSubG_SetFixSMPS(void);
void MRSubG_SetAlmostEmptyThresholdTx(uint8_t cThrTx);
void MRSubG_SetAlmostFullThresholdRx(uint8_t cThrRx);
uint32_t MRSubG_GetBytesOfTransaction(void);
MRSubGFSMState MRSubG_GetRadioFSMState(void);
PowerSaveLevels RADIO_STACK_SleepCheck(void);

#ifdef __cplusplus
}
#endif

#endif
