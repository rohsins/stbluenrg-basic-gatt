#ifndef _CUSTOM_BLE_STACK_CONF_H_
#define _CUSTOM_BLE_STACK_CONF_H_

/* --------------------- BLE stack configuration options --------------------------------------------------- */
/* This specific set of modular configuration options is used for configuring the Bluetooth Low Energy stack 
   ONLY when BLE_STACK_CUSTOM_CONF is defined on user application prepropoessor options */

#define CONTROLLER_MASTER_ENABLED                 (0U) 
#define CONTROLLER_PRIVACY_ENABLED                (0U) 
#define SECURE_CONNECTIONS_ENABLED                (0U) 
#define CONTROLLER_DATA_LENGTH_EXTENSION_ENABLED  (0U) 
#define CONTROLLER_2M_CODED_PHY_ENABLED           (0U)

#if defined(EXTENDED_ADV) || defined(PERIODIC_ADV)
/* ExtendedAdv demo configuration or PeriodicAdv demo configuration */
#define CONTROLLER_EXT_ADV_SCAN_ENABLED           (1U) 
#else
#define CONTROLLER_EXT_ADV_SCAN_ENABLED           (0U) 
#endif 

#define L2CAP_COS_ENABLED                         (0U) 

#if defined(PERIODIC_ADV) 
/* PeriodicAdv demo configuration */
#define CONTROLLER_PERIODIC_ADV_ENABLED           (1U) 
#else
#define CONTROLLER_PERIODIC_ADV_ENABLED           (0U) 
#endif 

#if defined(CTE_TAG) 
#define CONTROLLER_CTE_ENABLED                    (1U)
#else
#define CONTROLLER_CTE_ENABLED                    (0U)
#endif
#define CONTROLLER_POWER_CONTROL_ENABLED          (0U) 
#define CONNECTION_ENABLED                        (0U)

#endif // _CUSTOM_BLE_STACK_CONF_H_
