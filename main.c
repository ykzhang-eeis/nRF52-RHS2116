#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "app_timer.h"
#include "ble_nus.h"
#include "app_uart.h"
#include "app_util_platform.h"
#include "bsp_btn_ble.h"
#include "nrf_pwr_mgmt.h"

#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_queue.h"
#include "nrf_drv_spi.h"
#include "RHS2116.h"
#include "nrf_delay.h"

#define APP_BLE_CONN_CFG_TAG            1                                           /**< A tag identifying the SoftDevice BLE configuration. */

#define DEVICE_NAME                     "Nordic_UART"                               /**< Name of device. Will be included in the advertising data. */
#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_BLE_OBSERVER_PRIO           3                                           /**< Application's BLE observer priority. You shouldn't need to modify this value. */

#define APP_ADV_INTERVAL                64                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */

#define APP_ADV_DURATION                18000                                       /**< The advertising duration (180 seconds) in units of 10 milliseconds. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(10, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                       /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                      /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */


#define SPI_INSTANCE  1 /**< SPI instance index. */
static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);  /**< SPI instance. */
//SPI传输完成标志
static volatile bool spi_xfer_done; 
static uint8_t       m_tx_buf[] = {0,0,0,0};           /**< TX buffer. */
static uint8_t       m_rx_buf[sizeof(m_tx_buf)];    /**< RX buffer. */
static const uint8_t m_length = sizeof(m_tx_buf);        /**< Transfer length. */

	

static	  uint8_t packet_[70] = {0};
static    uint8_t  send_pack[210] = {0};

BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT);                                   /**< BLE NUS service instance. */
NRF_BLE_GATT_DEF(m_gatt);                                                           /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                             /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                                 /**< Advertising module instance. */

static uint16_t   m_conn_handle          = BLE_CONN_HANDLE_INVALID;                 /**< Handle of the current connection. */
static uint16_t   m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3;            /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */
static ble_uuid_t m_adv_uuids[]          =                                          /**< Universally unique service identifier. */
{
    {BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}
};


/**
** custom code
**/

#define APP_QUEUE
void ble_data_send_with_queue(void);
typedef struct {
    uint8_t  buf[210];
    uint16_t length;
} buffer_t;

NRF_QUEUE_DEF(buffer_t, m_buf_queue, 30, NRF_QUEUE_MODE_NO_OVERFLOW);

APP_TIMER_DEF(m_timer_speed);
//uint8_t m_data_array[6300];
uint32_t m_len_sent;
uint32_t m_drop_cnts ;
uint32_t m_cnt_7ms;
bool connect_flag =false;

ret_code_t err_code;
static buffer_t buf_;

void Gather_start(void);
void hexArrayToString(unsigned char* hexArray, int length, char* str);
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**@brief Function for initializing the timer module.
 */
static void timers_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of
 *          the device. It also sets the permissions and appearance.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *) DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}

static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

static void nus_data_handler(ble_nus_evt_t * p_evt)
{

    if (p_evt->type == BLE_NUS_EVT_RX_DATA)
    {
        uint32_t err_code;

        NRF_LOG_DEBUG("Received data from BLE NUS. Writing data on UART.");
        NRF_LOG_HEXDUMP_DEBUG(p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);

        for (uint32_t i = 0; i < p_evt->params.rx_data.length; i++)
        {
            do
            {
                err_code = app_uart_put(p_evt->params.rx_data.p_data[i]);
                if ((err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_BUSY))
                {
                    NRF_LOG_ERROR("Failed receiving NUS message. Error 0x%x. ", err_code);
                    APP_ERROR_CHECK(err_code);
                }
            } while (err_code == NRF_ERROR_BUSY);
        }
        if (p_evt->params.rx_data.p_data[p_evt->params.rx_data.length - 1] == '\r')
        {
            while (app_uart_put('\n') == NRF_ERROR_BUSY);
        }
    }
		else if (p_evt->type == BLE_NUS_EVT_COMM_STARTED)
		{
			connect_flag=true;
			APP_ERROR_CHECK(app_timer_start(m_timer_speed, APP_TIMER_TICKS(1000),NULL));			
		}
		else if (p_evt->type == BLE_NUS_EVT_TX_RDY)
		{
#ifndef APP_QUEUE			
			ret_code_t err_code;
			uint16_t length;	
			
			//sending code lines
			length = m_ble_nus_max_data_len;	
			do
			{					
				err_code = ble_nus_data_send(&m_nus, m_data_array, &length, m_conn_handle);
				if ( (err_code != NRF_ERROR_INVALID_STATE) && (err_code != NRF_ERROR_RESOURCES) &&
						 (err_code != NRF_ERROR_NOT_FOUND) )
				{
						APP_ERROR_CHECK(err_code);
				}
				if (err_code == NRF_SUCCESS)
				{
					m_len_sent += length; 	
					m_data_array[0]++;
					m_data_array[length-1]++;	
				}
			} while (err_code == NRF_SUCCESS);
#else
      ble_data_send_with_queue();
#endif
		}

}
/**@snippet [Handling the data received over BLE] */


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t           err_code;
    ble_nus_init_t     nus_init;
    nrf_ble_qwr_init_t qwr_init = {0};

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

    // Initialize NUS.
    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;

    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
}

static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}

static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;
        case BLE_ADV_EVT_IDLE:
            sleep_mode_enter();
            break;
        default:
            break;
    }
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    uint32_t err_code;
	
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected");
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
            
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_2MBPS,
                .tx_phys = BLE_GAP_PHY_2MBPS,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            connect_flag =false;
            NRF_LOG_INFO("Disconnected");
            // LED indication will be changed when advertising starts.
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
						APP_ERROR_CHECK(app_timer_stop(m_timer_speed));	
						m_len_sent = 0;
						m_cnt_7ms = 0;
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_2MBPS,
                .tx_phys = BLE_GAP_PHY_2MBPS,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for the SoftDevice initialization.
 *
 * @details This function initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);
    
    ble_cfg_t ble_cfg;
    
    memset(&ble_cfg, 0, sizeof(ble_cfg));
    ble_cfg.conn_cfg.conn_cfg_tag                     = APP_BLE_CONN_CFG_TAG;
    ble_cfg.conn_cfg.params.gatts_conn_cfg.hvn_tx_queue_size   = 2;

    err_code = sd_ble_cfg_set(BLE_CONN_CFG_GATTS, &ble_cfg, ram_start);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("sd_ble_cfg_set() returned %s when attempting to set BLE_CONN_CFG_GAP.",
                      nrf_strerror_get(err_code));
    }
    
    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
    {
        m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
        NRF_LOG_INFO("Data len is set to 0x%X(%d)", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
    }
    NRF_LOG_DEBUG("ATT MTU exchange completed. central 0x%x peripheral 0x%x",
                  p_gatt->att_mtu_desired_central,
                  p_gatt->att_mtu_desired_periph);
}


/**@brief Function for initializing the GATT library. */
void gatt_init(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
{
    uint32_t err_code;
    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break;

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BSP_EVENT_WHITELIST_OFF:
            if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
            {
                err_code = ble_advertising_restart_without_whitelist(&m_advertising);
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
            break;

        default:
            break;
    }
}


/**@brief   Function for handling app_uart events.
 *
 * @details This function will receive a single character from the app_uart module and append it to
 *          a string. The string will be be sent over BLE when the last character received was a
 *          'new line' '\n' (hex 0x0A) or if the string has reached the maximum data length.
 */
/**@snippet [Handling the data received over UART] */
void uart_event_handle(app_uart_evt_t * p_event)
{
    static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
    static uint8_t index = 0;
    uint32_t       err_code;

    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY:
            UNUSED_VARIABLE(app_uart_get(&data_array[index]));
            index++;

            if ((data_array[index - 1] == '\n') ||
                (data_array[index - 1] == '\r') ||
                (index >= m_ble_nus_max_data_len))
            {
                if (index > 1)
                {
                    NRF_LOG_DEBUG("Ready to send data over BLE NUS");
                    NRF_LOG_HEXDUMP_DEBUG(data_array, index);

                    do
                    {
                        uint16_t length = (uint16_t)index;
                        err_code = ble_nus_data_send(&m_nus, data_array, &length, m_conn_handle);
                        if ((err_code != NRF_ERROR_INVALID_STATE) &&
                            (err_code != NRF_ERROR_RESOURCES) &&
                            (err_code != NRF_ERROR_NOT_FOUND))
                        {
                            APP_ERROR_CHECK(err_code);
                        }
                    } while (err_code == NRF_ERROR_RESOURCES);
                }

                index = 0;
            }
            break;

        case APP_UART_COMMUNICATION_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
}

static void advertising_init(void)
{
    uint32_t               err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    init.advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance = false;
    init.advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;

    init.srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.srdata.uuids_complete.p_uuids  = m_adv_uuids;

    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;
    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}

uint32_t get_rtc_counter(void)
{
    return NRF_RTC1->COUNTER;
}

static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(get_rtc_counter);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}

static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}



static void advertising_start(void)
{
    uint32_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);
}

buffer_t m_buf;
void ble_data_send_with_queue(void)
{
	uint32_t err_code;
	uint16_t length_ = 0;
	static bool retry = false;
	if(!connect_flag){
        return;
    }
	if (retry)
	{
		length_ = m_buf.length;
		err_code = ble_nus_data_send(&m_nus, m_buf.buf, &length_, m_conn_handle);
        if(err_code == NRF_ERROR_RESOURCES)
        {
             return;
        }
		if ( (err_code != NRF_ERROR_INVALID_STATE) && (err_code != NRF_ERROR_RESOURCES) &&
				 (err_code != NRF_ERROR_NOT_FOUND) )
		{
				APP_ERROR_CHECK(err_code);
		}
		if (err_code == NRF_SUCCESS)
		{
			m_len_sent += length_;
			retry = false;

		}
        
	}
	
	while (!nrf_queue_is_empty(&m_buf_queue) && !retry )
	{		

		err_code = nrf_queue_pop(&m_buf_queue, &m_buf);
        if(err_code){
            break;
        }

		length_ = m_buf.length;
											
		err_code = ble_nus_data_send(&m_nus, m_buf.buf, &m_buf.length, m_conn_handle);
		if ( (err_code != NRF_ERROR_INVALID_STATE) && (err_code != NRF_ERROR_RESOURCES) &&
				 (err_code != NRF_ERROR_NOT_FOUND) )
		{
				APP_ERROR_CHECK(err_code);
		}
		if (err_code == NRF_SUCCESS)
		{
			m_len_sent += length_;
			retry = false;
		}
		else
		{
			retry = true;
			break;
		}
	}
        
}

static void throughput_timer_handler(void * p_context)
{
	NRF_LOG_INFO("==**Speed: %d B/s,drop:%d**==", m_len_sent,m_drop_cnts);
	m_drop_cnts = 0;
	m_len_sent = 0;
}

static void callCommand(void){

    spi_xfer_done = false;

     APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, m_tx_buf, m_length, m_rx_buf, m_length));
	  //nrf_drv_spi_transfer(&spi, m_tx_buf, m_length, m_rx_buf, m_length);
    while (!spi_xfer_done)
    {
        __WFE(); // wait for event 等待事件，即下一次事件发生前都在此hold住不干活，
		// 执行这条语句后CPU功耗会降低，通常用这条语句来省电。
    }
}

void Convert(uint8_t channel) //D flag = 1 in each
{
	m_tx_buf[0] = 0x08;
	m_tx_buf[1] = 0x00 + channel;
	m_tx_buf[2] = 0x00;
	m_tx_buf[3] = 0x00;
	callCommand();
}

void TransReg(uint8_t tx) {
    SPI_CS_LOW;
    m_tx_buf[0] = 0x80;
    m_tx_buf[1] = tx;
    m_tx_buf[2] = 0xFF;
    m_tx_buf[3] = 0xFF;
    callCommand();
    SPI_CS_HIGH;
}

void TransRegAndConv(uint8_t value, uint8_t tx_1, uint8_t tx_2, uint32_t* dat){
	Convert(value);
	TransReg(tx_1);
	TransReg(tx_2);
	dat[value] |= m_rx_buf[0] << 24;
	dat[value] |= m_rx_buf[1] << 16;
	dat[value] |= m_rx_buf[2] << 8;
	dat[value] |= m_rx_buf[3];
}

void transfer (void) {
	
	//Read Register 255
	m_tx_buf[0] = 0xC0;
	m_tx_buf[1] = 0xFF;
	m_tx_buf[2] = 0x00;
	m_tx_buf[3] = 0x00;
	callCommand();
	
    //Write Register 32 -> 0000
 
	m_tx_buf[0] = 0x80;
	m_tx_buf[1] = 0x20;
	m_tx_buf[2] = 0x00;
	m_tx_buf[3] = 0x00;
	callCommand();
	
	//Write Register 33 -> 0000

	m_tx_buf[0] = 0x80;
	m_tx_buf[1] = 0x21;
	m_tx_buf[2] = 0x00;
	m_tx_buf[3] = 0x00;
	callCommand();
		
	//Write Register 38 -> FFFF
 
    m_tx_buf[0] = 0x80;
	m_tx_buf[1] = 0x26;
	m_tx_buf[2] = 0xFF;
	m_tx_buf[3] = 0xFF;
	callCommand();
		
		//Clear
 
    m_tx_buf[0] = 0x6A;
	m_tx_buf[1] = 0x00;
	m_tx_buf[2] = 0x00;
	m_tx_buf[3] = 0x00;
	callCommand();
		
		//Write Register 0 -> 00C7
 
    m_tx_buf[0] = 0x80;
	m_tx_buf[1] = 0x00;
	m_tx_buf[2] = 0x00;
	m_tx_buf[3] = 0xc7;
	callCommand();
		
		//Write Register 1 -> 051A
 
    m_tx_buf[0] = 0x80;
	m_tx_buf[1] = 0x01;
	m_tx_buf[2] = 0x05;
	m_tx_buf[3] = 0x1A;
	callCommand();
		
		//Write Register 2 -> 0040
 
    m_tx_buf[0] = 0x80;
	m_tx_buf[1] = 0x02;
	m_tx_buf[2] = 0x00;
	m_tx_buf[3] = 0x40;
	callCommand();
		
		//Write Register 3 -> 0080
 
    m_tx_buf[0] = 0x80;
	m_tx_buf[1] = 0x03;
	m_tx_buf[2] = 0x00;
	m_tx_buf[3] = 0x80;
	callCommand();

		//Write Register 4 -> 0016
 
    m_tx_buf[0] = 0x80;
	m_tx_buf[1] = 0x04;
	m_tx_buf[2] = 0x00;
	m_tx_buf[3] = 0x16;
	callCommand();
		
		//Write Register 5 -> 0017
 
    m_tx_buf[0] = 0x80;
	m_tx_buf[1] = 0x05;
	m_tx_buf[2] = 0x00;
	m_tx_buf[3] = 0x17;
	callCommand();
	
		//Write Register 6 -> 00A8
 
    m_tx_buf[0] = 0x80;
	m_tx_buf[1] = 0x06;
	m_tx_buf[2] = 0x00;
	m_tx_buf[3] = 0xA8;
	callCommand();
		
		//Write Register 7 -> 000A
 
    m_tx_buf[0] = 0x80;
	m_tx_buf[1] = 0x07;
	m_tx_buf[2] = 0x00;
	m_tx_buf[3] = 0x0A;
	callCommand();
		
		//Write Register 8 -> FFFF
 
    m_tx_buf[0] = 0x80;
	m_tx_buf[1] = 0x08;
	m_tx_buf[2] = 0xFF;
	m_tx_buf[3] = 0xFF;
	callCommand();
		
		//Write Register 10 -> 0000 UFlag = 1
 
    m_tx_buf[0] = 0xA0;
	m_tx_buf[1] = 0x0A;
	m_tx_buf[2] = 0x00;
	m_tx_buf[3] = 0x00;
	callCommand();
		
		//Write Register 12 -> FFFF UFlag = 1
 
    m_tx_buf[0] = 0xA0;
	m_tx_buf[1] = 0x0C;
	m_tx_buf[2] = 0xFF;
	m_tx_buf[3] = 0xFF;
	callCommand();
		
		//Write Register 34 -> 00E2
 
    m_tx_buf[0] = 0x80;
	m_tx_buf[1] = 0x22;
	m_tx_buf[2] = 0x00;
	m_tx_buf[3] = 0xE2;
	callCommand();
		
		//Write Register 35 -> 00AA
 
    m_tx_buf[0] = 0x80;
	m_tx_buf[1] = 0x23;
	m_tx_buf[2] = 0x00;
	m_tx_buf[3] = 0xAA;
	callCommand();
	
		//Write Register 36 -> 0080
 
    m_tx_buf[0] = 0x80;
	m_tx_buf[1] = 0x24;
	m_tx_buf[2] = 0x00;
	m_tx_buf[3] = 0x80;
	callCommand();
	
		//Write Register 37 -> 4F00
 
    m_tx_buf[0] = 0x80;
	m_tx_buf[1] = 0x25;
	m_tx_buf[2] = 0x4F;
	m_tx_buf[3] = 0x00;
	callCommand();
		
		//Write Register 42 -> 0000 UFlag = 1
 
    m_tx_buf[0] = 0xA0;
	m_tx_buf[1] = 0x2A;
	m_tx_buf[2] = 0x00;
	m_tx_buf[3] = 0x00;
	callCommand();
	
		//Write Register 44 -> 0000 UFlag = 1
 
    m_tx_buf[0] = 0xA0;
	m_tx_buf[1] = 0x2C;
	m_tx_buf[2] = 0x00;
	m_tx_buf[3] = 0x00;
	callCommand();
	
		//Write Register 46 -> 0000 UFlag = 1
 
    m_tx_buf[0] = 0xA0;
	m_tx_buf[1] = 0x2E;
	m_tx_buf[2] = 0x00;
	m_tx_buf[3] = 0x00;
	callCommand();
		
		//Write Register 48 -> 0000 UFlag = 1
 
    m_tx_buf[0] = 0xA0;
	m_tx_buf[1] = 0x30;
	m_tx_buf[2] = 0x00;
	m_tx_buf[3] = 0x00;
	callCommand();
	
	//Write Register 64 ~ 79 -> 0000 UFlag = 1
	for(uint8_t i = 0x40; i<=0x4F; i++)
	{
		m_tx_buf[0] = 0xA0;
		m_tx_buf[1] = i;
		m_tx_buf[2] = 0x80;
		m_tx_buf[3] = 0x00;
		callCommand();
	}			
	
		//Write Register 96 ~ 111 -> 0000 UFlag = 1
	for(uint8_t i = 0x60; i<=0x6F; i++)
	{
		m_tx_buf[0] = 0xA0;
		m_tx_buf[1] = i;
		m_tx_buf[2] = 0x80;
		m_tx_buf[3] = 0x00;
		callCommand();
	}

		//Write Register 32 -> AAAA
 
    m_tx_buf[0] = 0x80;
	m_tx_buf[1] = 0x20;
	m_tx_buf[2] = 0xAA;
	m_tx_buf[3] = 0xAA;
	callCommand();
	
		//Write Register 33 -> 00FF
 
    m_tx_buf[0] = 0x80;
	m_tx_buf[1] = 0x21;
	m_tx_buf[2] = 0x00;
	m_tx_buf[3] = 0xFF;
	callCommand();
		
		//Read Register 255 MFlag = 1
	m_tx_buf[0] = 0xD0;
	m_tx_buf[1] = 0xFF;
	m_tx_buf[2] = 0x00;
	m_tx_buf[3] = 0x00;
	callCommand();
}

uint16_t crc16(const uint8_t* data, size_t length) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (size_t j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}



void packageData(const uint32_t* data, uint8_t* packet) {
    //Data packet format: [Header (2 bytes)][Length (2 bytes)][Data (64 bytes)][CRC16 (2 bytes)]

    uint8_t* dataBytes = (uint8_t*)data;
    
    packet[0] = 0xAA;
    packet[1] = 0xBB;
    
    packet[2] = 0x00; 
    packet[3] = 0x40; 

    for (size_t i = 0; i < 64; i++) {
        packet[4 + i] = dataBytes[i];
    }

    uint16_t crc = crc16(packet + 2, 66); 
    packet[68] = (uint8_t)(crc >> 8);    
    packet[69] = (uint8_t)(crc & 0xFF);
}

void spi_event_handler(nrf_drv_spi_evt_t const * p_event,void * p_context)
{   
    spi_xfer_done = true;   
}

void SPI_Init(void)
{
	  nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
    spi_config.ss_pin   = SPI_SS_PIN;
    spi_config.miso_pin = SPI_MISO_PIN;
    spi_config.mosi_pin = SPI_MOSI_PIN;
    spi_config.sck_pin  = SPI_SCK_PIN;
    APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler, NULL));
}

void hexArrayToString(unsigned char* hexArray, int length, char* str) {
    for (int i = 0; i < length; i++) {
        sprintf(&str[i * 2], "%02X", hexArray[i]);
    }
}
void Gather_start()
{
	  memset(send_pack,0,sizeof(send_pack)); //将send_pack中的数据清零

	  for(int i=0;i<3;i++)
	  {  
					uint32_t dat_[16] = {0};	   
					
					memset(packet_,0,sizeof(packet_));						
					TransRegAndConv(0,0x27,0x29,dat_);
					TransRegAndConv(1,0x2B,0x2D,dat_);
					TransRegAndConv(2,0x2F,0x31,dat_);
					TransRegAndConv(3,0x33,0x34,dat_);
					TransRegAndConv(4,0x35,0x36,dat_);
					TransRegAndConv(5,0x37,0x38,dat_);
					TransRegAndConv(6,0x39,0x3A,dat_);
					TransRegAndConv(7,0x3B,0x3C,dat_);
					TransRegAndConv(8,0x3D,0x3E,dat_);
					TransRegAndConv(9,0x50,0x51,dat_);
					TransRegAndConv(10,0x52,0x53,dat_);
					TransRegAndConv(11,0x54,0x55,dat_);
					TransRegAndConv(12,0x56,0x57,dat_);
					TransRegAndConv(13,0x58,0x59,dat_);
					TransRegAndConv(14,0x5A,0x5B,dat_);
					TransRegAndConv(15,0x5C,0x5D,dat_);
					packageData(dat_, packet_);
			     
			    memcpy(&send_pack[(i%10)*70],packet_,70); //使用memcpy函数将packet的70字节数据复制到send_pack，复制3次，共210字节
				
	    }	
		
		  if(packet_[0]==0xAA && packet_[1]==0xBB)
			{
                memcpy(buf_.buf,send_pack,210);
				buf_.length = 210;
				err_code = nrf_queue_push(&m_buf_queue, &buf_);
				
				if(err_code == NRF_ERROR_NO_MEM && connect_flag)
				{
                    m_drop_cnts++;
				}
						
			}					
}

void throughput_test()
{
	ret_code_t err_code;
	err_code = app_timer_create(&m_timer_speed, APP_TIMER_MODE_REPEATED, throughput_timer_handler);
	APP_ERROR_CHECK(err_code);	
}
int main(void)
{
	SPI_Init();
    log_init();
	transfer();
    timers_init();
    power_management_init();
    ble_stack_init();
    gap_params_init();
    gatt_init();
    services_init();
    advertising_init();
    conn_params_init();
    NRF_LOG_INFO("Debug logging for UART over RTT started.");
    advertising_start();
	throughput_test();

    for (;;)
    {
        Gather_start();//SPI采集
        ble_data_send_with_queue();
        idle_state_handle();
    }
}
