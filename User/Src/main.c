#include <stdio.h>
#include <string.h>
#include "rf_device_it.h"
#include "ble_const.h"
#include "bluenrg_lp_stack.h"
#include "Beacon_config.h"
#include "rf_driver_hal_power_manager.h"
#include "rf_driver_hal_vtimer.h"
#include "bleplat.h"
#include "nvm_db.h"
#include "pka_manager.h"
#include "rng_manager.h"
#include "aes_manager.h"
#include "ble_controller.h"
#include "rf_driver_ll_bus.h"


/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "freertos_ble.h"

SemaphoreHandle_t radioActivitySemaphoreHandle;
SemaphoreHandle_t BLETickSemaphoreHandle;


#define BLE_BEACON_VERSION_STRING "1.1"

/* Advertising interval for legacy advertising (0.625 ms units) 
  For iBeacon this should be set to 100 ms. */
#define LEGACY_ADV_INTERVAL     160  /* 100 ms */
#define EXT_ADV_INTERVAL        160 /* 100 ms */

/* Set to 1 to enable the name AD data in extended advertising events (if
  extended advertising events are used).  */
#define DEVICE_NAME_IN_ADV 0

/* PHY used in extended advertising events. One between: LE_1M_PHY,
  LE_2M_PHY and LE_CODED_PHY.  */
#define EXT_ADV_PHY LE_CODED_PHY

/*-----------------------------------------------------------*/
/* Wait time of the test task (numbe rof ticks) */
#define TEST_PERIOD         			    ( 200 / portTICK_PERIOD_MS )
#define ADV_CHANGE_PERIOD         			( 500 / portTICK_PERIOD_MS )

/* This is the length of advertising data to be set for legacy advertising.
   It does not include the device name, which is sent only in extended
  advertising events. */
#define SHORT_ADV_DATA_LENGTH    27

static uint8_t adv_data[] = {
  /* Advertising data: Flags AD Type not supported if broadcaster only */
  /* 0x02, 0x01, 0x06, */
  /* Advertising data: manufacturer specific data */
  26, //len
  AD_TYPE_MANUFACTURER_SPECIFIC_DATA,  //manufacturer type
  0x4C, 0x00, //Company identifier code
  0x02,       // ID
  0x15,       //Length of the remaining payload
  0xE2, 0x0A, 0x39, 0xF4, 0x73, 0xF5, 0x4B, 0xC4, //Location UUID
  0xA1, 0x2F, 0x17, 0xD1, 0xAD, 0x07, 0xA9, 0x61,
  0x00, 0x05, // Major number 
  0x00, 0x07, // Minor number 
  (uint8_t)-56,         // Tx power measured at 1 m of distance (in dBm)
#if DEVICE_NAME_IN_ADV
  15,       // Length of following AD data
  0x09,'E','x','t','e','n','d','e','d','B','e','a','c','o','n'
#endif
};


FILE __stdout;
FILE __stdin;

/* KEIL fputc implementation template allowing to redirect printf output towards serial port (UART/USB) */
int fputc(int c, FILE *f)
{  
//  BSP_COM_Write((uint8_t *)&c, 1);
  
  return 1;
}


int fgetc (FILE *f) {
  int data = 0;
//  while (Read_Buffer_Pop((uint8_t *)&data) != SUCCESS);
  return data;
}


NO_INIT(uint32_t dyn_alloc_a[DYNAMIC_MEMORY_SIZE>>2]);

void InitModules(void)
{
  uint8_t ret;
  BLE_STACK_InitTypeDef BLE_STACK_InitParams = BLE_STACK_INIT_PARAMETERS;
  
  LL_AHB_EnableClock(LL_AHB_PERIPH_PKA|LL_AHB_PERIPH_RNG);

  
  BLECNTR_InitGlobal();
  
  HAL_VTIMER_InitType VTIMER_InitStruct = {HS_STARTUP_TIME, INITIAL_CALIBRATION, CALIBRATION_INTERVAL};
  HAL_VTIMER_Init(&VTIMER_InitStruct);
  
  BLEPLAT_Init();  
  if (PKAMGR_Init() == PKAMGR_ERROR)
  {
      while(1);
  }
  if (RNGMGR_Init() != RNGMGR_SUCCESS)
  {
      while(1);
  }
  
  /* Init the AES block */
  AESMGR_Init();
  
  /* BlueNRG-LP stack init */
  ret = BLE_STACK_Init(&BLE_STACK_InitParams);
  if (ret != BLE_STATUS_SUCCESS) {
    printf("Error in BLE_STACK_Init() 0x%02x\r\n", ret);
    while(1);
  }
  
}

void ModulesTick(void)
{
  /* Timer tick */
  HAL_VTIMER_Tick();
  
  /* Bluetooth stack tick */
  BLE_STACK_Tick();
  
  /* NVM manager tick */
  NVMDB_Tick();
}

void InitDevice(void)
{
  uint8_t ret;
  uint16_t service_handle;
  uint16_t dev_name_char_handle;
  uint16_t appearance_char_handle;
  uint8_t address[CONFIG_DATA_PUBADDR_LEN] = {0x66,0x77,0x88,0xE1,0x80,0x02};
  
  aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET, CONFIG_DATA_PUBADDR_LEN, address);
  
  /* Set the TX Power to 0 dBm */
  ret = aci_hal_set_tx_power_level(0, 24);
  if(ret != 0) {
//    PRINTF ("Error in aci_hal_set_tx_power_level() 0x%04xr\n", ret);
    while(1);
  }

  
  /* Init the GAP: broadcaster role */
  ret = aci_gap_init(GAP_BROADCASTER_ROLE, 0x00, 0x08, PUBLIC_ADDR, &service_handle, &dev_name_char_handle, &appearance_char_handle);
  if (ret != 0)
  {
//    PRINTF ("Error in aci_gap_init() 0x%04x\r\n", ret);
  }
  else
  {
//    PRINTF ("aci_gap_init() --> SUCCESS\r\n");
  }
}

static void StartBeacon (void)
{
	uint8_t ret;
  Advertising_Set_Parameters_t Advertising_Set_Parameters[2];
  uint8_t adv_sets = 0;
  
#ifdef LEGACY_ADV
   
  /* Set advertising configuration for legacy advertising in broadcast mode */  
  ret = aci_gap_set_advertising_configuration(0, GAP_MODE_BROADCAST,
                                              ADV_PROP_LEGACY,
                                              LEGACY_ADV_INTERVAL, LEGACY_ADV_INTERVAL,
                                              ADV_CH_ALL,
                                              0,NULL, /* No peer address */
                                              ADV_NO_WHITE_LIST_USE,
                                              0, /* 0 dBm */
                                              LE_1M_PHY, /* Primary advertising PHY */
                                              0, /* 0 skips */
                                              LE_1M_PHY, /* Secondary advertising PHY. Not used with legacy advertising. */
                                              0, /* SID */
                                              0 /* No scan request notifications */);
  if (ret != BLE_STATUS_SUCCESS)
  {
//    PRINTF("Error in aci_gap_set_advertising_configuration() 0x%02x\r\n", ret);
    return;
  }
  
  ret = aci_gap_set_advertising_data(0, ADV_COMPLETE_DATA, SHORT_ADV_DATA_LENGTH, adv_data);
  if (ret != BLE_STATUS_SUCCESS)
  {
//    PRINTF("Error in aci_gap_set_advertising_data() 0x%02x\r\n", ret);
    return;
  }
  
  printf("Legacy advertising configured\n");
  
  Advertising_Set_Parameters[adv_sets].Advertising_Handle = 0;
  Advertising_Set_Parameters[adv_sets].Duration = 0;
  Advertising_Set_Parameters[adv_sets].Max_Extended_Advertising_Events = 0;
  
  adv_sets++;
  
#endif
  
#ifdef EXTENDED_ADV
  /* Set advertising configuration for extended advertising in broadcast mode */  
  ret = aci_gap_set_advertising_configuration(1, GAP_MODE_BROADCAST,
                                              ADV_PROP_NONE,
                                              EXT_ADV_INTERVAL, EXT_ADV_INTERVAL,
                                              ADV_CH_ALL,
                                              0,NULL, /* No peer address */
                                              ADV_NO_WHITE_LIST_USE,
                                              0, /* 0 dBm */
                                              (EXT_ADV_PHY==LE_2M_PHY)?LE_1M_PHY:EXT_ADV_PHY, /* Primary advertising PHY */
                                              0, /* 0 skips */
                                              EXT_ADV_PHY, /* Secondary advertising PHY */
                                              0, /* SID */
                                              0 /* No scan request notifications */);
  if (ret != BLE_STATUS_SUCCESS)
  {
//    PRINTF("Error in aci_gap_set_advertising_configuration() 0x%02x\r\n", ret);
    return;
  }
  
  ret = aci_gap_set_advertising_data(1, ADV_COMPLETE_DATA, sizeof(adv_data), adv_data);
  if (ret != BLE_STATUS_SUCCESS)
  {
//    PRINTF("Error in aci_gap_set_advertising_data() 0x%02x\r\n", ret);
    return;
  }
  
  printf("Extended advertising configured\n");
  
  Advertising_Set_Parameters[adv_sets].Advertising_Handle = 1;
  Advertising_Set_Parameters[adv_sets].Duration = 0;
  Advertising_Set_Parameters[adv_sets].Max_Extended_Advertising_Events = 0;
  
  adv_sets++;
  
#endif /* EXTENDED_ADV */

  /* Enable advertising */
  ret = aci_gap_set_advertising_enable(ENABLE, adv_sets, Advertising_Set_Parameters);
  if (ret != BLE_STATUS_SUCCESS)
  {
//    PRINTF ("Error in aci_gap_set_advertising_enable() 0x%02x\r\n", ret);
    return;
  }
  
  printf("Advertising started\n");
}

static void BLETask (void * params)
{
  xSemaphoreTake(BLETickSemaphoreHandle, portMAX_DELAY);
  
  InitDevice();
  
  StartBeacon();
  
  xSemaphoreGive(BLETickSemaphoreHandle);
  
  while(1)
  {
    xSemaphoreTake(BLETickSemaphoreHandle, portMAX_DELAY);
    ModulesTick();    
    xSemaphoreGive(BLETickSemaphoreHandle);
		
    if(BLE_STACK_SleepCheck() != POWER_SAVE_LEVEL_RUNNING && HAL_VTIMER_SleepCheck() == TRUE)
    {
      xSemaphoreTake(radioActivitySemaphoreHandle, portMAX_DELAY);
    }
  }
}

static void changeADVDataTask( void *pvParameters )
{
  TickType_t xNextWakeTime;
  uint8_t Random_Number[8];
  
  /* Initialise xNextWakeTime - this only needs to be done once. */
  xNextWakeTime = xTaskGetTickCount();
  
  for( ;; )
  {
    /* Place this task in the blocked state until it is time to run again.
    The block time is specified in ticks, the constant used converts ticks
    to ms.  While in the Blocked state this task will not consume any CPU
    time. */
    vTaskDelayUntil( &xNextWakeTime, ADV_CHANGE_PERIOD );
    
    BLE_ACI_PROTECTED(hci_le_rand(Random_Number));
    
    adv_data[25] = Random_Number[0];
    
    /* In this case there is no need to disable advertising before updating buffer
       content, since only one byte is changed (atomic operation) and there is no
       risk to have inconsistent data. */
    
    BLE_ACI_PROTECTED(aci_gap_set_advertising_data(0, ADV_COMPLETE_DATA, SHORT_ADV_DATA_LENGTH, adv_data));
    
    
#if EXTENDED_ADV
    BLE_ACI_PROTECTED(aci_gap_set_advertising_data(1, ADV_COMPLETE_DATA, sizeof(adv_data), adv_data));
#endif
    
//    PRINTF("ADV change %d\r\n", adv_data[25]);
    
  }  
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
//	vTaskDelay(3000); // safety stuff
	
	while (SystemInit(SYSCLK_64M, BLE_SYSCLK_32M) != 0) {
		vTaskDelay(1000);
	};
	
	radioActivitySemaphoreHandle = xSemaphoreCreateBinary();
	BLETickSemaphoreHandle = xSemaphoreCreateMutex();
	
	if (radioActivitySemaphoreHandle==NULL || BLETickSemaphoreHandle == NULL){
    while(1) {
//			vTaskDelay(1000); // go to limbo
		}
  }
	
	initIO();
    
  InitModules(); 
	
	xTaskCreate(BLETask, "BLEStack", 650, NULL, tskIDLE_PRIORITY + 1, NULL);
  xTaskCreate(changeADVDataTask, "ADV", 150, NULL, tskIDLE_PRIORITY + 2, NULL );
  
  /* Start the tasks and timer running. */
  vTaskStartScheduler();
	
	return 0;
}
