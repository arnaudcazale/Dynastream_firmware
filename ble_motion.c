#include "sdk_common.h"
#include "ble_srv_common.h"
#include "ble_motion.h"
#include <string.h>
#include "nrf_gpio.h"
#include "boards.h"
#include "nrf_log.h"


uint32_t ble_motion_init(ble_motion_t * p_motion, const ble_motion_init_t * p_motion_init)
{
    if (p_motion == NULL || p_motion_init == NULL)
    {
        return NRF_ERROR_NULL;
    }

    uint32_t   err_code;
    ble_uuid_t ble_uuid;

    // Initialize service structure
    p_motion->evt_handler               = p_motion_init->evt_handler;
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

    // Add Custom Value characteristic
    err_code = acceleration_value_char_add(p_motion, p_motion_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    // Add Custom Value characteristic
    err_code = configuration_char_add(p_motion, p_motion_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
}

static uint32_t configuration_char_add(ble_motion_t * p_motion, const ble_motion_init_t * p_motion_init)
{
    uint32_t            err_code;
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;
 

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read          = 1;
    char_md.char_props.write         = 1;
    char_md.char_props.write_wo_resp = 0;
    char_md.p_char_user_desc         = NULL;
    char_md.p_char_pf                = NULL;
    char_md.p_user_desc_md           = NULL;
    char_md.p_cccd_md                = NULL;
    char_md.p_sccd_md                = NULL;

    ble_uuid.type = p_motion->uuid_type;
    ble_uuid.uuid = CONFIGURATION_CHAR_UUID;
		
    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 1;
    attr_md.vlen       = 1;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = sizeof(ble_motion_config_t);
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = sizeof(ble_motion_config_t);

    err_code = sd_ble_gatts_characteristic_add(p_motion->service_handle, &char_md,
                                               &attr_char_value,
                                               &p_motion->configuration_handles);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    return NRF_SUCCESS;
}

/**@brief Function for adding the Custom Value characteristic.
 *
 * @param[in]   p_cus        Custom Service structure.
 * @param[in]   p_cus_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t acceleration_value_char_add(ble_motion_t * p_motion, const ble_motion_init_t * p_motion_init)
{
    uint32_t            err_code;
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;
    uint8_t             data = {0};

    // Add Custom Value characteristic
    memset(&cccd_md, 0, sizeof(cccd_md));

    //  Read  operation on cccd should be possible without authentication.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    
    //cccd_md.write_perm = p_motion_init->motion_value_char_attr_md.cccd_write_perm;
    cccd_md.vloc       = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));

    //char_md.char_props.read   = 0; 
    //char_md.char_props.write  = 0; 
    char_md.char_props.notify = 1; 
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md;  
    char_md.p_sccd_md         = NULL;

    ble_uuid.type = p_motion->uuid_type;
    ble_uuid.uuid = ACCELERATION_VALUE_CHAR_UUID;
		
    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);

    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = sizeof(uint8_t);
    attr_char_value.init_offs = 0;
    attr_char_value.p_value   = (uint8_t *)&data;
    attr_char_value.max_len   = sizeof(uint16_t)*32*3;

    err_code = sd_ble_gatts_characteristic_add(p_motion->service_handle, &char_md,
                                               &attr_char_value,
                                               &p_motion->acceleration_value_handles);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    return NRF_SUCCESS;
}


/**@brief Function for handling the Connect event.
 *
 * @param[in]   p_cus       Custom Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_connect(ble_motion_t * p_motion, ble_evt_t const * p_ble_evt)
{
    p_motion->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;

    ble_motion_evt_t evt;

    evt.evt_type = BLE_MOTION_EVT_CONNECTED;

    p_motion->evt_handler(p_motion, &evt);
}

/**@brief Function for handling the Disconnect event.
 *
 * @param[in]   p_cus       Custom Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_disconnect(ble_motion_t * p_motion, ble_evt_t const * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_motion->conn_handle = BLE_CONN_HANDLE_INVALID;
}

/**@brief Function for handling the Write event.
 *
 * @param[in]   p_cus       Custom Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_write(ble_motion_t * p_motion, ble_evt_t const * p_ble_evt)
{
    ble_gatts_evt_write_t const * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
    
    // Check if the Custom value CCCD is written to and that the value is the appropriate length, i.e 2 bytes.
    if ((p_evt_write->handle == p_motion->acceleration_value_handles.cccd_handle)
        && (p_evt_write->len == 2)
       )
    {

        // CCCD written, call application event handler
        if (p_motion->evt_handler != NULL)
        {
            ble_motion_evt_t evt;

            if (ble_srv_is_notification_enabled(p_evt_write->data))
            {
                evt.evt_type = BLE_MOTION_EVT_NOTIFICATION_ENABLED;
            }
            else
            {
                evt.evt_type = BLE_MOTION_EVT_NOTIFICATION_DISABLED;
            }
            // Call the application event handler.
            p_motion->evt_handler(p_motion, &evt);
        }

    }
}

static void on_autorize_req(ble_motion_t * p_motion, ble_evt_t const * p_ble_evt)
{
    
    ble_gatts_evt_rw_authorize_request_t * p_evt_rw_authorize_request = &p_ble_evt->evt.gatts_evt.params.authorize_request;
    uint32_t err_code;

    if (p_evt_rw_authorize_request->type  == BLE_GATTS_AUTHORIZE_TYPE_WRITE)
    {
      //NRF_LOG_INFO("BLE_GATTS_AUTHORIZE_TYPE_WRITE");
      if (p_evt_rw_authorize_request->request.write.handle == p_motion->configuration_handles.value_handle)
      {
        ble_gatts_rw_authorize_reply_params_t rw_authorize_reply;
        bool                                  valid_data = true;
        // Check for valid data.
        if(p_evt_rw_authorize_request->request.write.len != sizeof(ble_motion_config_t))
        {
            valid_data = false;
        }
        else
        {
          ble_motion_config_t * p_config = (ble_motion_config_t *)p_evt_rw_authorize_request->request.write.data;
           if ( (p_config->resolution    > LIS2DH_RESOLUTION_MAXVALUE) ||
                (p_config->frequency     > LIS2DH_ODR_MAXVALUE)        ||
                (p_config->scale         > LIS2DH_FS_MAXVALUE) )   
            {
              valid_data = false;
            }
        }

        rw_authorize_reply.type = BLE_GATTS_AUTHORIZE_TYPE_WRITE;

        if (valid_data)
        {
            rw_authorize_reply.params.write.update      = 1;
            rw_authorize_reply.params.write.gatt_status = BLE_GATT_STATUS_SUCCESS;
            rw_authorize_reply.params.write.p_data      = p_evt_rw_authorize_request->request.write.data;
            rw_authorize_reply.params.write.len         = p_evt_rw_authorize_request->request.write.len;
            rw_authorize_reply.params.write.offset      = p_evt_rw_authorize_request->request.write.offset;
        }
        else
        {
            rw_authorize_reply.params.write.update      = 0;
            rw_authorize_reply.params.write.gatt_status = BLE_GATT_STATUS_ATTERR_WRITE_NOT_PERMITTED;
        }

        err_code = sd_ble_gatts_rw_authorize_reply(p_ble_evt->evt.gatts_evt.conn_handle,
                                                   &rw_authorize_reply);
        APP_ERROR_CHECK(err_code);

        if ( valid_data && (p_motion->evt_handler != NULL))
            {
                ble_motion_evt_t evt;
                evt.evt_type = BLE_MOTION_EVT_CONFIG_RECEIVED;
                evt.p_data = p_evt_rw_authorize_request->request.write.data;
                evt.length = p_evt_rw_authorize_request->request.write.len;
                p_motion->evt_handler(p_motion, &evt);
            }
      }
    }

    if (p_evt_rw_authorize_request->type  == BLE_GATTS_AUTHORIZE_TYPE_READ)
    {
      NRF_LOG_INFO("BLE_GATTS_AUTHORIZE_TYPE_READ");
    }
    
}

void ble_motion_on_ble_evt( ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code;
    ble_motion_t * p_motion = (ble_motion_t *) p_context;

    //NRF_LOG_INFO("BLE event received. Event type = 0x%X\r\n", p_ble_evt->header.evt_id); 
    
    if (p_motion == NULL || p_ble_evt == NULL)
    {
        return;
    }
    
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_motion, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_motion, p_ble_evt);
            break;

        case BLE_GATTS_EVT_WRITE:
            //NRF_LOG_INFO("BLE_GATTS_EVT_WRITE"); 
            on_write(p_motion, p_ble_evt);
           break;

        case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
            on_autorize_req(p_motion, p_ble_evt);
           break;

        default:
            // No implementation needed.
            break;
    }
}

uint32_t ble_motion_acceleration_value_update(ble_motion_t * p_motion, int16_t * acceleration_value){

    //NRF_LOG_INFO("In ble_motion_custom_value_update. \r\n"); 

    //Serialize
    int16_t buffer_serialized[32*3];
    int16_t (*buff)[3] = acceleration_value;
    
    for ( int i = 0 ; i < 32 ; i++ ) 
    {
//      NRF_LOG_INFO("Acceleration mG x= %d mg - y= %d mg - z= %d mg", buff[i][0], buff[i][1], buff[i][2]);
//      NRF_LOG_FLUSH();
      buffer_serialized[3*i] = buff[i][0];
      buffer_serialized[(3*i)+1] = buff[i][1];
      buffer_serialized[(3*i)+2] = buff[i][2];
    }

//    for ( int i = 0 ; i < 32*3 ; i++ ) 
//    {
//      NRF_LOG_INFO("buffer_serialized %d", buffer_serialized[i]);
//      NRF_LOG_FLUSH();
//    }
//    NRF_LOG_INFO("\r\n");
//    NRF_LOG_FLUSH();


    if (p_motion == NULL)
    {
        return NRF_ERROR_NULL;
    }

    uint32_t err_code = NRF_SUCCESS;
    ble_gatts_value_t gatts_value;

    // Initialize value struct.
    memset(&gatts_value, 0, sizeof(gatts_value));

    gatts_value.len     = sizeof(uint16_t)*(32*3);
    gatts_value.offset  = 0;
    gatts_value.p_value = buffer_serialized;

    // Update database.
    err_code = sd_ble_gatts_value_set(p_motion->conn_handle,
                                        p_motion->acceleration_value_handles.value_handle,
                                        &gatts_value);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Send value if connected and notifying.
    if ((p_motion->conn_handle != BLE_CONN_HANDLE_INVALID)) 
    {
        ble_gatts_hvx_params_t hvx_params;

        memset(&hvx_params, 0, sizeof(hvx_params));

        hvx_params.handle = p_motion->acceleration_value_handles.value_handle;
        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset = gatts_value.offset;
        hvx_params.p_len  = &gatts_value.len;
        hvx_params.p_data = gatts_value.p_value;

        err_code = sd_ble_gatts_hvx(p_motion->conn_handle, &hvx_params);
    }
    else
    {
        err_code = NRF_ERROR_INVALID_STATE;
    }

    return err_code;
}

uint32_t m_motion_configuration_apply(ble_motion_config_t * p_config)
{
    uint32_t err_code;
    lis2dh_cfg_t motion_cfg;

    if (p_config == NULL)
    {
        return NRF_ERROR_NULL;
    }

    motion_cfg.resolution    = p_config->resolution;
    motion_cfg.frequency    = p_config->frequency;
    motion_cfg.scale         = p_config->scale;

    err_code = lis2dh_config(&motion_cfg);
    APP_ERROR_CHECK(err_code);

    return NRF_SUCCESS;
}

uint32_t m_motion_configuration_default_init()
{
    uint32_t err_code;

    lis2dh_cfg_t motion_cfg;

    motion_cfg.scale         = LIS2DH_FS_SCALE_2G;
    motion_cfg.resolution    = LIS2DH_RESOLUTION_12B;
    motion_cfg.frequency     = LIS2DH_ODR_200HZ;

    err_code = lis2dh_config(&motion_cfg);
    APP_ERROR_CHECK(err_code);

    return NRF_SUCCESS;
}