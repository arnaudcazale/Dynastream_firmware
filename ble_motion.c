#include "sdk_common.h"
#include "ble_srv_common.h"
#include "ble_motion.h"
#include <string.h>
#include "nrf_gpio.h"
#include "boards.h"
#include "nrf_log.h"

uint32_t ble_cus_init(ble_motion_t * p_motion, const ble_motion_init_t * p_motion_init)
{
    if (p_motion == NULL || p_motion_init == NULL)
    {
        return NRF_ERROR_NULL;
    }

    uint32_t   err_code;
    ble_uuid_t ble_uuid;

    // Initialize service structure
    p_motion->conn_handle               = BLE_CONN_HANDLE_INVALID;

    // Add Custom Service UUID
    ble_uuid128_t base_uuid = {MOTION_SERVICE_UUID_BASE};
    err_code =  sd_ble_uuid_vs_add(&base_uuid, &p_motion->uuid_type);
    VERIFY_SUCCESS(err_code);

    ble_uuid.type = p_motion->uuid_type;
    ble_uuid.uuid = MOTION_SERVICE_UUID;

    // Add the Custom Service
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_motion->service_handle);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

}