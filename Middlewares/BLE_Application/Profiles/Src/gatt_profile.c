/******************** (C) COPYRIGHT 2020 STMicroelectronics ********************
* File Name          : gatt_profile.c
* Author             : SRA - BLE stack team
* Description        : Generic Attribute Profile Service (GATT)
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/******************************************************************************
 * INCLUDE HEADER FILES
 *****************************************************************************/
#include "gatt_profile.h"
#include <string.h>
#include "osal.h"
#include "bluenrg_lp_api.h"
#include "rf_driver_ll_gpio.h"

#define C_CHAR_UUID 0xba,0x5c,0xf7,0x93,0x3b,0x12,0xd3,0x89,0xe4,0x11,0x0d,0x9b,0x2e,0xf6,0x0e,0xed
static uint16_t controlPointHandle;

/******************************************************************************
 * LOCAL VARIABLES
 *****************************************************************************/
/**
 *@defgroup Service Changed Characteristic value.
 *@{
 */
/**
 * Characteristic value buffer.
 */
static const uint8_t gatt_chr_srv_changed_buff[GATT_CHR_SERVICE_CHANGED_VALUE_LEN] =
                                                {0x00U, 0x00U, 0xffU, 0xffU};

static const ble_gatt_val_buffer_def_t gatt_chr_srv_changed_value_buff = {
    .buffer_len = GATT_CHR_SERVICE_CHANGED_VALUE_LEN,
    .buffer_p = (uint8_t *)gatt_chr_srv_changed_buff,
};
/**
 *@}
 */

/**
 *@defgroup Client Supported Feature Characteristic value.
 *@{
 */
/**
 * The following buffer store the Client Supported Feature characteristic
 * value, as described in 7.2 CLIENT SUPPORTED FEATURES - BLUETOOTH CORE
 * SPECIFICATION Version 5.1 | Vol 3, Part G page 2403.
 * The assigned bits are reported in Table 7.6: Client Supported Features
 * bit assignments.
 */
static uint8_t gatt_client_supp_feature_buff[BLE_GATT_SRV_CLIENT_SUP_FEATURE_SIZE_X_CONN(GATT_SRV_MAX_CONN)];

static uint8_t gatt_client_custom_feature_buff[BLE_GATT_SRV_CLIENT_SUP_FEATURE_SIZE_X_CONN(GATT_SRV_MAX_CONN)];

static const ble_gatt_val_buffer_def_t gatt_client_supp_feature_val_buff = {
    .buffer_len = BLE_GATT_SRV_CLIENT_SUP_FEATURE_SIZE_X_CONN(GATT_SRV_MAX_CONN),
    .buffer_p = gatt_client_supp_feature_buff,
};

static const ble_gatt_val_buffer_def_t gatt_client_custom_feature_val_buff = {
    .buffer_len = BLE_GATT_SRV_CLIENT_SUP_FEATURE_SIZE_X_CONN(GATT_SRV_MAX_CONN),
    .buffer_p = gatt_client_custom_feature_buff,
};

/**
 *@}
 */

/**
 * Service Changed CCCD.
 */
BLE_GATT_SRV_CCCD_DECLARE(gatt_chr_srv_changed,
                          GATT_SRV_MAX_CONN,
                          BLE_GATT_SRV_CCCD_PERM_DEFAULT,
                          BLE_GATT_SRV_OP_MODIFIED_EVT_ENABLE_FLAG);

static const ble_gatt_chr_def_t gatt_chrs[] = {
    {
        /**< Service Changed Characteristic. */
        .properties = BLE_GATT_SRV_CHAR_PROP_INDICATE,
        .permissions = BLE_GATT_SRV_PERM_NONE,
        .uuid = BLE_UUID_INIT_16(BLE_GATT_SRV_SERVICE_CHANGE_CHR_UUID),
        .val_buffer_p = (ble_gatt_val_buffer_def_t *)&gatt_chr_srv_changed_value_buff,
        .descrs = {
            .descrs_p = &BLE_GATT_SRV_CCCD_DEF_NAME(gatt_chr_srv_changed),
            .descr_count = 1U,
        },
    },
    {
        /**< Client Supported Features Characteristic. */
        .properties = BLE_GATT_SRV_CHAR_PROP_READ | BLE_GATT_SRV_CHAR_PROP_WRITE,
        .permissions = BLE_GATT_SRV_PERM_NONE,
        .uuid = BLE_UUID_INIT_16(BLE_GATT_SRV_CLIENT_SUPP_FEATURE_CHR_UUID),
        .val_buffer_p = (ble_gatt_val_buffer_def_t *)&gatt_client_supp_feature_val_buff,
    },
    {
        /**< Database Hash Characteristic. Value buffer is allocated into the stack. */
        .properties = BLE_GATT_SRV_CHAR_PROP_READ,
        .permissions = BLE_GATT_SRV_PERM_NONE,
        .uuid = BLE_UUID_INIT_16(BLE_GATT_SRV_DB_HASH_CHR_UUID),
    },
		{
				/** Custom Characteristic **/
        .properties = BLE_GATT_SRV_CHAR_PROP_READ | BLE_GATT_SRV_CHAR_PROP_WRITE, // | BLE_GATT_SRV_CHAR_PROP_WRITE_NO_RESP,
        .permissions = BLE_GATT_SRV_PERM_NONE,
        .min_key_size = BLE_GATT_SRV_MAX_ENCRY_KEY_SIZE,
        .uuid = BLE_UUID_INIT_128(C_CHAR_UUID),
//				.val_buffer_p = (ble_gatt_val_buffer_def_t *)&gatt_client_custom_feature_val_buff,
    }
};

static const ble_gatt_srv_def_t gatt_srvc = {
    .type = BLE_GATT_SRV_PRIMARY_SRV_TYPE,
    .uuid = BLE_UUID_INIT_16(BLE_GATT_SRV_GATT_SERVICE_UUID),
    .chrs = {
        .chrs_p = (ble_gatt_chr_def_t *)gatt_chrs,
        .chr_count = 4U,
    },
};

tBleStatus Gatt_profile_init()
{
	tBleStatus result = aci_gatt_srv_add_service((ble_gatt_srv_def_t *)&gatt_srvc);
	controlPointHandle = aci_gatt_srv_get_char_decl_handle((ble_gatt_chr_def_t *)&gatt_chrs[3]); // Custom characteristics control point
	
	return result;
}

tBleStatus Gatt_profile_set_access_permission(uint16_t attr_h, uint8_t perm)
{
    uint16_t cccd_handle;

    cccd_handle = aci_gatt_srv_get_descriptor_handle(
                            &BLE_GATT_SRV_CCCD_DEF_NAME(gatt_chr_srv_changed));
    if ((cccd_handle != BLE_ATT_INVALID_ATTR_HANDLE) && (cccd_handle == attr_h))
    {
        /**
         * Clear previous set access permissions.
         */
        BLE_GATT_SRV_CCCD_DEF_NAME(gatt_chr_srv_changed).properties &=
                                        ~BLE_GATT_SRV_CHAR_PROP_ACCESS_PERM_MASK;
        BLE_GATT_SRV_CCCD_DEF_NAME(gatt_chr_srv_changed).properties |= (perm &
                                        BLE_GATT_SRV_CHAR_PROP_ACCESS_PERM_MASK);

        return BLE_STATUS_SUCCESS;
    }
    else
    {
        return BLE_STATUS_FAILED;
    }
}

tBleStatus Gatt_profile_set_security_permission(uint16_t attr_h, uint8_t perm)
{
    uint16_t cccd_handle;

    cccd_handle = aci_gatt_srv_get_descriptor_handle(
                            &BLE_GATT_SRV_CCCD_DEF_NAME(gatt_chr_srv_changed));
    if ((cccd_handle != BLE_ATT_INVALID_ATTR_HANDLE) && (cccd_handle == attr_h))
    {
        if ((perm & (BLE_GATT_SRV_PERM_ENCRY_READ |
                     BLE_GATT_SRV_PERM_AUTHEN_READ)) != 0U)
        {
            /**
             * <<Readable with no authentication or authorization.
             *   Writable with authentication and authorization defined by a
             *   higher layer specification or is implementation specific.>>
             *
             * Table 3.10: Client Characteristic Configuration declaration
             * BLUETOOTH CORE SPECIFICATION Version 5.2 | Vol 3, Part G page 1556
             */
            return BLE_STATUS_NOT_ALLOWED;
        }

        BLE_GATT_SRV_CCCD_DEF_NAME(gatt_chr_srv_changed).permissions = perm;

        return BLE_STATUS_SUCCESS;
    }
    else
    {
        return BLE_STATUS_FAILED;
    }
}

uint16_t Gatt_profile_get_service_handle(void)
{
    return aci_gatt_srv_get_service_handle((ble_gatt_srv_def_t *)&gatt_srvc);
}

ble_gatt_srv_def_t *Gatt_profile_get_service_definition_p(void)
{
    return (ble_gatt_srv_def_t *)&gatt_srvc;
}

static uint8_t leds_value[] = { 0x01, 0x02 };

extern uint8_t isBusy;

void aci_gatt_srv_write_event(uint16_t Connection_Handle, uint8_t Resp_Needed, uint16_t Attribute_Handle, uint16_t Data_Length, uint8_t Data[])
{
	uint8_t att_err = BLE_ATT_ERR_NONE;

	if (Attribute_Handle == controlPointHandle + 1)
	{
		leds_value[0] = Data[0];
		leds_value[1] = Data[1];
		
		/** Set GPIO value */
		LL_GPIO_TogglePin(GPIOB, LL_GPIO_PIN_10);
		isBusy ^= 1;
	}
		
	if (Resp_Needed == 1U)
	{
			aci_gatt_srv_resp(Connection_Handle, 0, att_err, 0, NULL);
	}
}

void aci_gatt_srv_read_event(uint16_t Connection_Handle, uint16_t Attribute_Handle, uint16_t Data_Offset)
{
  uint8_t att_err = BLE_ATT_ERR_NONE;
	
  if(Attribute_Handle == controlPointHandle + 1)
  {
    aci_gatt_srv_resp(Connection_Handle, Attribute_Handle, att_err, 2, leds_value);
  }
}

/******************* (C) COPYRIGHT 2020 STMicroelectronics *****END OF FILE****/
