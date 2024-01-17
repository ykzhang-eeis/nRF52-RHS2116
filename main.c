#include <stdint.h>
#include <string.h>
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

#define DEVICE_NAME                     "Nordic_UART"                               /**< Name of device. Will be included in the advertising data. */

#define APP_BLE_CONN_CFG_TAG            1                                           /**< A tag identifying the SoftDevice BLE configuration. */
#define APP_BLE_OBSERVER_PRIO           3                                           /**< Application's BLE observer priority. You shouldn't need to modify this value. */

#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_ADV_INTERVAL                64                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
#define APP_ADV_DURATION                18000                                       /**< The advertising duration (180 seconds) in units of 10 milliseconds. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(8, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(16, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                       /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                      /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */


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


#define SPI_INSTANCE  1 /**< SPI instance index. */
static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);  /**< SPI instance. */
static volatile bool spi_xfer_done; 
static uint8_t       m_tx_buf[] = {0,0,0,0};           /**< TX buffer. */
static uint8_t       m_rx_buf[sizeof(m_tx_buf)];    /**< RX buffer. */
static const uint8_t m_length = sizeof(m_tx_buf);        /**< Transfer length. */

static	  uint8_t packet_[70] = {0};
static    uint8_t  send_pack[210] = {0};

#define APP_QUEUE
void ble_data_send_with_queue(void);
typedef struct {
    uint8_t  buf[210];
    uint16_t length;
} buffer_t;

NRF_QUEUE_DEF(buffer_t, m_buf_queue, 30, NRF_QUEUE_MODE_NO_OVERFLOW);

APP_TIMER_DEF(m_timer_speed);
uint32_t m_len_sent;
uint32_t m_drop_cnts ;
uint32_t m_cnt_7ms;
bool connect_flag =false;

ret_code_t err_code;
static buffer_t buf_;

void Gather_start(void);
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

static void timers_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}

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

static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

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

static void sleep_mode_enter(void)
{
    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}

static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            break;
        case BLE_ADV_EVT_IDLE:
            sleep_mode_enter();
            break;
        default:
            break;
    }
}


static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    uint32_t err_code;
	
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected");
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



void gatt_init(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);
}



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

uint32_t RHS2116_RW_WORD(uint32_t dat_32)
{
		  
	uint8_t ret1,ret2,ret3,ret4;
	uint32_t ret_32=0;
	
	ret1=(uint8_t)((dat_32>>24)&0xff);
	ret2=(uint8_t)((dat_32>>16)&0xff);
	ret3=(uint8_t)((dat_32>>8)&0xff);
	ret4=(uint8_t)((dat_32)&0xff);
	m_tx_buf[0]=ret1;
	m_tx_buf[1]=ret2;
	m_tx_buf[2]=ret3;
	m_tx_buf[3]=ret4;
	spi_xfer_done = false;	
	APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, m_tx_buf, m_length, m_rx_buf, m_length));
	while (!spi_xfer_done);					
	ret_32=(m_rx_buf[0]<<24)+(m_rx_buf[1]<<16)+(m_rx_buf[2]<<8)+(m_rx_buf[3]);	
	return ret_32;
}

uint32_t CMD_CONVERT_REG(uint8_t channel)
{
	uint32_t cmd_32=0x08000000;
	uint32_t ret_32;
	cmd_32|=((0x00ff0000)&(channel<<16));
	
	SPI_CS_LOW;
	RHS2116_RW_WORD(cmd_32);
	SPI_CS_HIGH;
	
	SPI_CS_LOW;
	RHS2116_RW_WORD(0xffffffff);
	SPI_CS_HIGH;
	
	SPI_CS_LOW;
	ret_32=RHS2116_RW_WORD(0xffffffff);
	SPI_CS_HIGH;
	
	return ret_32;
}

    // Write data D to register R
uint8_t CMD_WRITE_REG(uint8_t U,uint8_t M,uint8_t R,uint16_t D)
{
	uint32_t cmd_32=0x80000000;
	uint32_t ret_32;
	if(U!=0) cmd_32|=0x20000000;
	if(M!=0) cmd_32|=0x10000000;
	
	cmd_32|=((0x00ff0000)&(R<<16));
	cmd_32|=((0x0000ffff)&(D));
	
	SPI_CS_LOW;
	RHS2116_RW_WORD(cmd_32);
	SPI_CS_HIGH;
	
	SPI_CS_LOW;
	RHS2116_RW_WORD(0xffffffff);
	SPI_CS_HIGH;
	
	SPI_CS_LOW;
	ret_32=RHS2116_RW_WORD(0xffffffff);
	SPI_CS_HIGH;
	
	if((ret_32&0xffff0000)!=0xffff0000) 
	{return 255;}
	
	if((ret_32&0x0000ffff)!=D) 
	{return 255;}
	
	return 0;
}

void CMD_READ_REG(uint8_t U,uint8_t M,uint8_t R,uint16_t* D)
{
	uint32_t cmd_32=0xc0000000;
	uint32_t ret_32;
	if(U!=0) cmd_32|=0x20000000;
	if(M!=0) cmd_32|=0x10000000;
	
	cmd_32|=((0x00ff0000)&(R<<16));
	
	SPI_CS_LOW;
	RHS2116_RW_WORD(cmd_32);
	SPI_CS_HIGH;
	
	SPI_CS_LOW;
	RHS2116_RW_WORD(0xffffffff);
	SPI_CS_HIGH;
	
	SPI_CS_LOW;
	ret_32=RHS2116_RW_WORD(0xffffffff);
	SPI_CS_HIGH;
	
	*D=(uint16_t)(ret_32&0x0000ffff);
}

void CMD_CLEAR_ADC(void)
{
	uint32_t cmd_32=0x6a000000;
	
	SPI_CS_LOW;
	RHS2116_RW_WORD(cmd_32);
	SPI_CS_HIGH;
	
	SPI_CS_LOW;
	RHS2116_RW_WORD(0xffffffff);
	SPI_CS_HIGH;
	
	SPI_CS_LOW;
	RHS2116_RW_WORD(0xffffffff);
	SPI_CS_HIGH;
}

uint8_t rhs2116_init(void)
{
	uint16_t val;
	
	CMD_READ_REG(0,0,255,&val);
	
	CMD_WRITE_REG(0,0,32,0x0000);
	CMD_WRITE_REG(0,0,33,0x0000);
	CMD_WRITE_REG(0,0,38,0xffff);
	CMD_CLEAR_ADC();
	
	CMD_WRITE_REG(0,0,0,0x00c7);
	CMD_WRITE_REG(0,0,1,0x051a);
	CMD_WRITE_REG(0,0,2,0x0040);
	CMD_WRITE_REG(0,0,3,0x0080);
	CMD_WRITE_REG(0,0,4,0x0016);
	CMD_WRITE_REG(0,0,5,0x0017);
	CMD_WRITE_REG(0,0,6,0x00a8);
	CMD_WRITE_REG(0,0,7,0x000a);
	CMD_WRITE_REG(0,0,8,0xffff);
	CMD_WRITE_REG(1,0,10,0x0000);
	CMD_WRITE_REG(1,0,12,0xffff);
	CMD_WRITE_REG(0,0,34,0x00e2);
	CMD_WRITE_REG(0,0,35,0x00aa);
	CMD_WRITE_REG(0,0,36,0x0080);
	CMD_WRITE_REG(0,0,37,0x4f00);
	CMD_WRITE_REG(1,0,42,0x0000);
	CMD_WRITE_REG(1,0,44,0x0000);
	CMD_WRITE_REG(1,0,46,0x0000);
	CMD_WRITE_REG(1,0,48,0x0000);
	
	CMD_WRITE_REG(1,0,64,0x8000);
	CMD_WRITE_REG(1,0,65,0x8000);
	CMD_WRITE_REG(1,0,66,0x8000);
	CMD_WRITE_REG(1,0,67,0x8000);
	CMD_WRITE_REG(1,0,68,0x8000);
	CMD_WRITE_REG(1,0,69,0x8000);
	CMD_WRITE_REG(1,0,70,0x8000);
	CMD_WRITE_REG(1,0,71,0x8000);
	CMD_WRITE_REG(1,0,72,0x8000);
	CMD_WRITE_REG(1,0,73,0x8000);
	CMD_WRITE_REG(1,0,74,0x8000);
	CMD_WRITE_REG(1,0,75,0x8000);
	CMD_WRITE_REG(1,0,76,0x8000);
	CMD_WRITE_REG(1,0,77,0x8000);
	CMD_WRITE_REG(1,0,78,0x8000);
	CMD_WRITE_REG(1,0,79,0x8000);
	CMD_WRITE_REG(1,0,96,0x8000);
	CMD_WRITE_REG(1,0,97,0x8000);
	CMD_WRITE_REG(1,0,98,0x8000);
	CMD_WRITE_REG(1,0,99,0x8000);
	CMD_WRITE_REG(1,0,100,0x8000);
	CMD_WRITE_REG(1,0,101,0x8000);
	CMD_WRITE_REG(1,0,102,0x8000);
	CMD_WRITE_REG(1,0,103,0x8000);
	CMD_WRITE_REG(1,0,104,0x8000);
	CMD_WRITE_REG(1,0,105,0x8000);
	CMD_WRITE_REG(1,0,106,0x8000);
	CMD_WRITE_REG(1,0,107,0x8000);
	CMD_WRITE_REG(1,0,108,0x8000);
	CMD_WRITE_REG(1,0,109,0x8000);
	CMD_WRITE_REG(1,0,110,0x8000);
	CMD_WRITE_REG(1,0,111,0x8000);
	
	CMD_WRITE_REG(0,0,32,0xaaaa);
	CMD_WRITE_REG(0,0,33,0x00ff);
	
	CMD_READ_REG(0,1,255,&val);
	
	return 0;
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


void packageData(const uint32_t* data, size_t dataLength, uint8_t* packet) {
    // Data packet format: [Header (2 bytes)][Length (2 bytes)][Data (Variable length)][CRC16 (2 bytes)]

    uint8_t* dataBytes = (uint8_t*)data;
    
    packet[0] = 0xAA; // Header byte 1
    packet[1] = 0xBB; // Header byte 2
    
    // Assuming dataLength is the number of bytes in the data array.
    packet[2] = (uint8_t)((dataLength >> 8) & 0xFF); // Length high byte
    packet[3] = (uint8_t)(dataLength & 0xFF);       // Length low byte

    for (size_t i = 0; i < dataLength && i < 64; i++) { // Copy up to 64 bytes of data
        packet[4 + i] = dataBytes[i];
    }

    // Assuming packet length for CRC is length of header, length, and data
    uint16_t crc = crc16(packet + 2, 2 + dataLength); 
    packet[4 + dataLength] = (uint8_t)(crc >> 8);    // CRC high byte
    packet[5 + dataLength] = (uint8_t)(crc & 0xFF);  // CRC low byte
}


void Gather_start_repeat_3_times()
{
    memset(send_pack,0,sizeof(send_pack));

    for(int i=0;i<3;i++)
    {  
        uint32_t dat_[16] = {0};
				
				size_t data_length = sizeof(dat_);
                
        memset(packet_,0,sizeof(packet_));
        
        for(int j=0;j<16;j++)
        {
            dat_[j]=CMD_CONVERT_REG(j);
        }
        packageData(dat_, data_length, packet_);
            
        memcpy(&send_pack[(i%10)*(data_length+6)],packet_,(data_length+6));
				
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

void Gather_start_no_repeat()
{
    memset(send_pack,0,sizeof(send_pack));

			uint32_t dat_[16] = {0};
			
			size_t data_length = sizeof(dat_);
							
			memset(packet_,0,sizeof(packet_));
			
			for(int j=0;j<16;j++)
			{
					dat_[j]=CMD_CONVERT_REG(j);
			}
			packageData(dat_, data_length, packet_);
            

		
    if(packet_[0]==0xAA && packet_[1]==0xBB)
    {
        memcpy(buf_.buf,packet_,data_length+6);
        buf_.length = data_length+6;
        err_code = nrf_queue_push(&m_buf_queue, &buf_);
    
        if(err_code == NRF_ERROR_NO_MEM && connect_flag)
        {
            m_drop_cnts++;
        }
                
    }					
}

static void throughput_timer_handler(void * p_context)
{
	NRF_LOG_INFO("==**Speed: %d B/s,drop:%d**==", m_len_sent,m_drop_cnts);
	m_drop_cnts = 0;
	m_len_sent = 0;
}

void throughput_test()
{
	ret_code_t err_code;
	err_code = app_timer_create(&m_timer_speed, APP_TIMER_MODE_REPEATED, throughput_timer_handler);
	APP_ERROR_CHECK(err_code);	
}

void spi_event_handler(nrf_drv_spi_evt_t const * p_event,
                       void *                    p_context)
{
	spi_xfer_done = true;
}

void spi_init(void)
{
	nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
    spi_config.ss_pin   = SPI_SS_PIN;
    spi_config.miso_pin = SPI_MISO_PIN;
    spi_config.mosi_pin = SPI_MOSI_PIN;
    spi_config.sck_pin  = SPI_SCK_PIN;
    APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler, NULL));
}

int main(void)
{
    spi_init();
    rhs2116_init();
    log_init();
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
        Gather_start_no_repeat();
        ble_data_send_with_queue();
        idle_state_handle();
    }
}
