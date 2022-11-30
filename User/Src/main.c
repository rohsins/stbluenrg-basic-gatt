#include <stdio.h>
#include <string.h>
#include "rf_device_it.h"
#include "ble_const.h"
#include "bluenrg_lp_stack.h"
#include "ble_user_config.h"
#include "rf_driver_hal_power_manager.h"
#include "rf_driver_hal_vtimer.h"
#include "bleplat.h"
#include "nvm_db.h"
#include "pka_manager.h"
#include "rng_manager.h"
#include "aes_manager.h"
#include "ble_controller.h"
#include "rf_driver_ll_bus.h"

#include "gap_profile.h"
//#include "gatt_db.h"

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "freertos_ble.h"

#define BLE_STATE_IDLE         0
#define BLE_STATE_ADVERTISING  1
#define BLE_STATE_CONNECTED    2
#define BLE_STATE_SLEEP        3
#define BLE_STATE_DISCONNECTED 4

NO_INIT(uint32_t dyn_alloc_a[DYNAMIC_MEMORY_SIZE>>2]);

SemaphoreHandle_t radioActivitySemaphoreHandle;
SemaphoreHandle_t BLETickSemaphoreHandle;

static uint8_t DeviceState;
static uint16_t ConnectionInterval;

static const uint16_t AdvertisingInterval = 100;

static uint8_t AdvertisementData[] = { 0x02, AD_TYPE_FLAGS, FLAG_BIT_LE_GENERAL_DISCOVERABLE_MODE | FLAG_BIT_BR_EDR_NOT_SUPPORTED,
	6, AD_TYPE_COMPLETE_LOCAL_NAME, 'R', '-', 'B', 'L', 'E' };

static Advertising_Set_Parameters_t AdvertisingSetParamters[1];
	
void InitDevice(void)
{
  uint8_t ret;
  uint16_t ServiceHandle;
  uint16_t DeviceNameCharHandle;
  uint16_t AppearanceCharHandle;
  uint8_t BLEAddress[CONFIG_DATA_PUBADDR_LEN] = {0x67,0x77,0x88,0xE1,0x80,0x02};
	uint8_t DeviceName[] = { 'H', 'A', 'R', 'D', 'W', 'A', 'R', 'E', '-', 'R', '&', 'D' };
  
  ret = aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET, CONFIG_DATA_PUBADDR_LEN, BLEAddress);
	
	if (ret != BLE_STATUS_SUCCESS)
	{
		// Error
	}
	else
	{
		// Success
	}
  
	ret = aci_hal_set_tx_power_level(0, 24); // 24 = 0dbm transmit power
	
  if(ret != BLE_STATUS_SUCCESS) 
	{
//  Error
    while(1); // go to limbo
  }

  ret = aci_gatt_srv_init();
	if (ret != 	BLE_STATUS_SUCCESS)
	{
		// Error
	}
	
	ret = aci_gap_init(GAP_PERIPHERAL_ROLE, 0, sizeof DeviceName, 0 , &ServiceHandle, &DeviceNameCharHandle, &AppearanceCharHandle);
  if (ret != BLE_STATUS_SUCCESS)
  {
//    Error
  }
  else
  {
//    Success
  }
	
	ret = Gap_profile_set_dev_name(0, sizeof DeviceName, DeviceName);
	if (ret != BLE_STATUS_SUCCESS)
  {
//    Error
  }
  else
  {
//    Success
  }
	
//	ret = Add_RC_Service();
	
	ret = aci_gap_set_advertising_configuration(0, GAP_MODE_GENERAL_DISCOVERABLE, ADV_PROP_CONNECTABLE | ADV_PROP_SCANNABLE | ADV_PROP_LEGACY,
		(AdvertisingInterval * 1000) / 625, (AdvertisingInterval * 1000) / 625,
		ADV_CH_ALL,
		PUBLIC_ADDR, NULL,
		ADV_NO_WHITE_LIST_USE,
		0,
		LE_1M_PHY,
		0,
		LE_1M_PHY,
		0,
		0);
	
	ret = aci_gap_set_advertising_data(0, ADV_COMPLETE_DATA,  sizeof AdvertisementData, AdvertisementData);
}

void StartAdvertising (void)
{
	uint8_t ret;
	
	AdvertisingSetParamters[0].Advertising_Handle = 0;
	AdvertisingSetParamters[0].Duration = 0;
	AdvertisingSetParamters[0].Max_Extended_Advertising_Events = 0;
	
	ret = aci_gap_set_advertising_enable(ENABLE, 1, AdvertisingSetParamters);
	if (ret != BLE_STATUS_SUCCESS)
	{
		// Error
	}
	else
	{
		// Success
		DeviceState = BLE_STATE_ADVERTISING;
	}
}

void InitModules(void)
{
  uint8_t ret;
  BLE_STACK_InitTypeDef BLEStackInitParams = BLE_STACK_INIT_PARAMETERS;
  
  LL_AHB_EnableClock(LL_AHB_PERIPH_PKA|LL_AHB_PERIPH_RNG);
  
  BLECNTR_InitGlobal();
  
  HAL_VTIMER_InitType VTIMERInitStruct = {HS_STARTUP_TIME, INITIAL_CALIBRATION, CALIBRATION_INTERVAL};
  HAL_VTIMER_Init(&VTIMERInitStruct);
  
  BLEPLAT_Init();  
	
  if (PKAMGR_Init() == PKAMGR_ERROR)
  {
      while(1);
  }
  if (RNGMGR_Init() != RNGMGR_SUCCESS)
  {
      while(1);
  }
  
  AESMGR_Init();
  
  ret = BLE_STACK_Init(&BLEStackInitParams);
  if (ret != BLE_STATUS_SUCCESS) {
    // Error
    while(1);
  }
}

void ModulesTick(void)
{
  HAL_VTIMER_Tick();
  BLE_STACK_Tick();
  NVMDB_Tick();
}

void initIO (void)
{
	LL_PWR_EnablePDA(LL_PWR_PUPD_IO0);
  LL_PWR_EnablePDA(LL_PWR_PUPD_IO1);
  LL_PWR_EnablePDA(LL_PWR_PUPD_IO2);
  LL_PWR_EnablePDA(LL_PWR_PUPD_IO3);
  LL_PWR_EnablePDA(LL_PWR_PUPD_IO4);
  LL_PWR_EnablePDA(LL_PWR_PUPD_IO5);
  LL_PWR_EnablePUA(LL_PWR_PUPD_IO6);
  LL_PWR_EnablePDA(LL_PWR_PUPD_IO7);
  LL_PWR_EnablePDA(LL_PWR_PUPD_IO8);
  LL_PWR_EnablePUA(LL_PWR_PUPD_IO9);
  LL_PWR_EnablePDA(LL_PWR_PUPD_IO10);
  LL_PWR_EnablePDA(LL_PWR_PUPD_IO11); 
  LL_PWR_EnablePDA(LL_PWR_PUPD_IO12);
  LL_PWR_EnablePDA(LL_PWR_PUPD_IO13); 
  LL_PWR_EnablePDA(LL_PWR_PUPD_IO14);
  LL_PWR_EnablePDA(LL_PWR_PUPD_IO15);
  
  LL_PWR_EnablePDB(LL_PWR_PUPD_IO0);
  LL_PWR_EnablePDB(LL_PWR_PUPD_IO1);
  LL_PWR_EnablePDB(LL_PWR_PUPD_IO2);
  LL_PWR_EnablePDB(LL_PWR_PUPD_IO3);
  LL_PWR_EnablePDB(LL_PWR_PUPD_IO4);
  LL_PWR_EnablePDB(LL_PWR_PUPD_IO5);
  LL_PWR_EnablePDB(LL_PWR_PUPD_IO6);
  LL_PWR_EnablePDB(LL_PWR_PUPD_IO7);
  LL_PWR_EnablePUB(LL_PWR_PUPD_IO8);
  LL_PWR_EnablePUB(LL_PWR_PUPD_IO9);
  LL_PWR_EnablePDB(LL_PWR_PUPD_IO10);
  LL_PWR_EnablePDB(LL_PWR_PUPD_IO11);
  LL_PWR_EnablePDB(LL_PWR_PUPD_IO14);
  LL_PWR_EnablePDB(LL_PWR_PUPD_IO15);
}

int main (void) {
	
	WakeupSourceConfig_TypeDef WakeupIO;
	PowerSaveLevels StopLevel;
	
	while (SystemInit(SYSCLK_16M, BLE_SYSCLK_16M) != 0) {
		vTaskDelay(1000);
	};
	
//	initIO();
  InitModules();
	InitDevice();
	
	WakeupIO.IO_Mask_High_polarity = 0;
	WakeupIO.IO_Mask_Low_polarity = 0;
	WakeupIO.RTC_enable = 0;
	WakeupIO.LPU_enable = 0;
	
	DeviceState = BLE_STATE_IDLE;
	
	StartAdvertising();
	DeviceState = BLE_STATE_ADVERTISING;
	
	while (1)
	{
		ModulesTick();
		
//		switch(DeviceState) {
//			case BLE_STATE_DISCONNECTED:
//				DeviceState = BLE_STATE_IDLE;
//				break;
//			case BLE_STATE_CONNECTED:
//				DeviceState = BLE_STATE_SLEEP;
//				break;
//			case BLE_STATE_IDLE:
//				break;
//			default:
//				HAL_PWR_MNGR_Request(POWER_SAVE_LEVEL_STOP_NOTIMER, WakeupIO, &StopLevel);
//    }
		
		HAL_PWR_MNGR_Request(POWER_SAVE_LEVEL_STOP_NOTIMER, WakeupIO, &StopLevel);
	}

	return 0;
}

PowerSaveLevels App_PowerSaveLevel_Check(PowerSaveLevels level)
{
	if(BLE_STACK_SleepCheck() == POWER_SAVE_LEVEL_RUNNING && HAL_VTIMER_SleepCheck() != TRUE)
	{
		return POWER_SAVE_LEVEL_RUNNING;
	}
  
  return POWER_SAVE_LEVEL_STOP_NOTIMER;
}

void hci_le_connection_complete_event (uint8_t Status,
	uint16_t ConnectionHandle,
	uint8_t Role, uint8_t PeerAddressType, uint8_t PeerAddress[6],
	uint16_t ConnectionIntervalParam, uint16_t ConnectionLatency, uint16_t SupervisionTimeout,
	uint8_t MasterClockAccuracy)
{
	if (Status == BLE_STATUS_SUCCESS)	
	{
		DeviceState = BLE_STATE_CONNECTED;
		ConnectionInterval = ConnectionIntervalParam;
	}
}

void hci_le_enhanced_connection_complete_event (uint8_t Status,
	uint16_t ConnectionHandle,
	uint8_t Role, uint8_t PeerAddressType, uint8_t PeerAddress[6], uint8_t LocalResolvablePrivateAddress[6], uint8_t PeerResolvablePrivateAddress[6],
	uint16_t ConnectionIntervalParam, uint16_t ConnectionLatency, uint16_t SupervisionTimeout,
	uint8_t MasterClockAccuracy)
{
	hci_le_connection_complete_event(
		Status, ConnectionHandle, Role, PeerAddressType,
		PeerAddress, ConnectionIntervalParam, ConnectionLatency,
		SupervisionTimeout, MasterClockAccuracy
	);
}

void hci_disconnection_complete_event (uint8_t Status,
	uint16_t ConnectionHandle,
	uint8_t Reason)
{
	DeviceState = BLE_STATE_DISCONNECTED;
	
	StartAdvertising();
	DeviceState = BLE_STATE_ADVERTISING;
}

void hci_hardware_error_event(uint8_t Hardware_Code)
{
  if (Hardware_Code <= 0x03)
  {
    NVIC_SystemReset();
  }
}

void aci_hal_fw_error_event(uint8_t FW_Error_Type,
                            uint8_t Data_Length,
                            uint8_t Data[])
{
  if (FW_Error_Type <= 0x03)
  {
    uint16_t connHandle;
    
    connHandle = LE_TO_HOST_16(Data);
    
    aci_gap_terminate(connHandle, BLE_ERROR_TERMINATED_REMOTE_USER); 
  }
}

void aci_gatt_srv_attribute_modified_event(uint16_t Connection_Handle,
                                       uint16_t Attr_Handle,
                                       uint16_t Attr_Data_Length,
                                       uint8_t Attr_Data[])
{
  
}

void aci_l2cap_connection_update_resp_event(uint16_t Connection_Handle,
                                            uint16_t Result)
{
  if(Result) {
//    PRINTF("> Connection parameters rejected.\n");
  } else  {
//    PRINTF("> Connection parameters accepted.\n");
  }
}
