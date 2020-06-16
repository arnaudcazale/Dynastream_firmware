#ifndef BLE_MOTION_H__
#define BLE_MOTION_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"


#define MOTION_SERVICE_UUID_BASE         {0x04, 0x00, 0x13, 0xAC, 0x42, 0x02, 0xDE, 0xB3, \
                                          0xEA, 0x11, 0xB2, 0xAF, 0x26, 0x31, 0xAC, 0x2B}

#define MOTION_SERVICE_UUID               0x1400
#define MOTION_VALUE_CHAR_UUID            0x1401

/**@brief   Macro for defining a ble_cus instance.
 *
 * @param   _name   Name of the instance.
 * @hideinitializer
 */
#define BLE_MOTION_DEF(_name)                                                              \
static ble_motion_t _name;                                                                 \


/**@brief Custom Service init structure. This contains all options and data needed for
 *        initialization of the service.*/
typedef struct
{
    uint8_t                       initial_motion_value;           /**< Initial custom value */
    ble_srv_cccd_security_mode_t  motion_value_char_attr_md;     /**< Initial security level for Custom characteristics attribute */
} ble_motion_init_t;

/**@brief Custom Service structure. This contains various status information for the service. */
struct ble_motion_s
{
    uint16_t                      service_handle;                 /**< Handle of Custom Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t      custom_value_handles;           /**< Handles related to the Custom Value characteristic. */
    uint16_t                      conn_handle;                    /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */
    uint8_t                       uuid_type; 
};

// Forward declaration of the ble_cus_t type.
typedef struct ble_motion_s ble_motion_t;

/**@brief Function for initializing the Custom Service.
 *
 * @param[out]  p_motion       Custom Service structure. This structure will have to be supplied by
 *                             the application. It will be initialized by this function, and will later
 *                             be used to identify this particular service instance.
 * @param[in]   p_motion_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
uint32_t ble_cus_init(ble_motion_t * p_motion, const ble_motion_init_t * p_motion_init);

























#endif // NORDIC_COMMON_H__